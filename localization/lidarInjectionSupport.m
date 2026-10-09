classdef lidarInjectionSupport
% lidarInjectionSupport LiDAR measurement channel of the full observer.
% Static methods filter registration information, reevaluate frozen residual
% geometry at the observer prediction, predict calibrated pose-error moments
% and compute the Lyapunov-matched continuous or implicit correction.
% Example: [delta,audit] = lidarInjectionSupport.computeLidarMatchedCorrection(m,pose,G,design,cfg).

    methods (Static)
        function filter = filterLidarPoseInformation(information,cfg,frameReliability,directionReliability,poseErrorVariance)
        % filterLidarPoseInformation Bounded weighting of supported LiDAR directions.
        % INFORMATION is an unregularized physical [X,Y,psi] information surrogate.
        % Direction reliabilities refer to ascending normalized information eigenvalues.
        % Repeated eigenspaces receive their minimum requested reliability, avoiding
        % dependence on the arbitrary eigenvector basis. No count or eigenvalue floor
        % is added to geometric information. PSD rank loss and complete rejection are
        % valid. Reliability must come from association/geometry checks upstream.
        % A calibrated channel replaces the geometric regularizer with predicted
        % pose-error second moments [horizontal per axis; heading], in m^2/rad^2.
        % Its F need not be symmetric: F*H=S is the required affine identity.
            arguments
                information (3,3) double {mustBeReal,mustBeFinite}
                cfg (1,1) struct
                frameReliability (1,1) double {mustBeReal,mustBeFinite} = 1
                directionReliability (3,1) double {mustBeReal,mustBeFinite} = ones(3,1)
                poseErrorVariance double = []
            end
            scales=cfg.lidar.poseScales(:);calibrated=isfield(cfg.lidar,'errorCalibration');
            validateattributes(scales,{'double'},{'real','finite','positive','numel',3});
            if calibrated
                assert(~isfield(cfg.lidar,'gainInformationScale'),'VehicleLocalization:AmbiguousLidarCalibration', ...
                    'A calibrated channel has no geometric information-scale parameter.');
                assert(isequal(scales,ones(3,1)),'VehicleLocalization:UnsupportedFullIssScaling', ...
                    'The calibrated channel uses physical metres and radians.');
                reference=cfg.lidar.errorCalibration.referenceVariance(:);
                validateattributes(reference,{'double'},{'real','finite','positive','numel',2});
                lidarInjectionSupport.predictLidarPoseErrorVariance(zeros(2,1),cfg.lidar.errorCalibration);
                lambda=NaN;
            else
                lambda=cfg.lidar.gainInformationScale;
                validateattributes(lambda,{'numeric'},{'real','finite','positive','scalar'});
            end
            assert(frameReliability>=0 && frameReliability<=1 && ...
                all(directionReliability>=0 & directionReliability<=1), ...
                'VehicleLocalization:InvalidLidarReliability','Reliabilities must lie in [0,1].');
            assert(norm(information-information.','fro')<=1e-10*max(1,norm(information,'fro')), ...
                'VehicleLocalization:InvalidLidarInformation','Information must be symmetric.');
            D=diag(scales);I=D*((information+information.')/2)*D;
            assert(all(isfinite(I),'all'),'VehicleLocalization:InvalidLidarInformation', ...
                'Normalized information must be finite.');
            [U,E]=eig(I);[values,order]=sort(diag(E));U=U(:,order);
            tolerance=1e-10*max(1,max(abs(values)));
            assert(min(values)>=-tolerance,'VehicleLocalization:InvalidLidarInformation', ...
                'Information must be positive semidefinite.');
            values=max(values,0); % Remove negative roundoff only; do not fill nullspaces.
            if calibrated,tolerance=3*eps(max(values));end % Numerical rank, independent of loss units.
            first=1;
            while first<=3
                last=first;
                while last<3 && values(last+1)-values(first)<=tolerance,last=last+1;end
                directionReliability(first:last)=min(directionReliability(first:last));
                first=last+1;
            end
            rho=frameReliability*directionReliability;
            if calibrated
                active=values>tolerance;
                if any(active)
                    validateattributes(poseErrorVariance,{'double'},{'real','finite','positive','numel',2});
                    authority=min(1,reference./poseErrorVariance(:));
                    % Covariance is expressed per position axis and in yaw radians.
                    % The empirical mean second moment is the unit of reliability.
                    % Project both sides so no correction enters an unobserved mode.
                    V=U(:,active);B=V*diag(sqrt(rho(active)))*V.';
                    S=B*diag(authority([1,1,2]))*B;
                    inverse=V*diag(1./values(active))*V.';F=S*inverse;
                else
                    S=zeros(3);F=zeros(3);
                end
                s=eig((S+S.')/2);
                interpretation="Empirically calibrated conditional pose-error second moment; geometry supplies the admitted subspace";
            else
                f=rho./(values+lambda);s=rho.*(values./(values+lambda));
                F=U*diag(f)*U.';S=U*diag(s)*U.';
                F=(F+F.')/2;
                interpretation="Weighted geometric surrogate; not calibrated covariance";
            end
            informationTrace=sum(values);dimension=0;condition=Inf;
            if values(end)>0
                relative=values/values(end);
                dimension=sum(relative)^2/sum(relative.^2);
                if values(1)>0,condition=values(end)/values(1);end
            end
            filter=struct('F',F,'S',(S+S.')/2,'D',D, ...
                'normalizedInformation',U*diag(values)*U.', ...
                'eigenvectors',U,'eigenvalues',values,'reliability',rho, ...
                'strengths',s,'rank',nnz(values>tolerance), ...
                'trace',informationTrace,'minimumEigenvalue',values(1), ...
                'conditionNumber',condition,'effectiveDimension',dimension, ...
                'regularizer',lambda,'informationInterpretation',interpretation, ...
                'poseErrorVariance',poseErrorVariance,'calibrated',calibrated);
        end

        function [correction,audit] = computeLidarMatchedCorrection(measurement,pose,G,design,cfg,options)
        % computeLidarMatchedCorrection Continuous LiDAR injection and its discretization.
        % C=D\(G*T), xi=-F*D'*J'*W*r or S*(D\(qL-pose)); the continuous
        % correction is kappa*T*(P\(C'*xi)). P is a Lyapunov matrix, not covariance.
        % Supply pose/information, residual/jacobian/weights/linearizationPose, or
        % gradient/information/linearizationPose. Residuals and physical pose
        % Jacobians MUST be evaluated at POSE, with frozen associations; a final
        % optimizer gradient is not the predicted-pose residual. A gradient
        % measurement is the pose gradient of a scalar cost at POSE with a PSD
        % curvature (for example evaluateOverlapGradient); xi=-F*D'*gradient,
        % restricted to the directions with positive information.
        % WEIGHTS is an N-vector of diagonal weights or an N-by-N PSD precision matrix.
        % Optional frameReliability and directionReliability lie in [0,1].
        % Continuous mode returns a state derivative. Discrete modes return a state
        % increment for the locally affine channel, with geometry frozen at POSE.
        % Implicit: (I+h*P\Q)^-1 is P-nonexpansive for every h>=0.
        % Explicit: h is capped to h*lambdaMax(P^-1/2*Q*P^-1/2)<=1.8<2.
        % Neither jump certificate certifies nonlinear associations or baseline flow.
            arguments
                measurement (1,1) struct
                pose (3,1) double {mustBeReal,mustBeFinite}
                G double {mustBeReal,mustBeFinite}
                design (1,1) struct
                cfg (1,1) struct
                options.StepSize (1,1) double {mustBeReal,mustBeFinite,mustBeNonnegative} = 0
                options.Discretization (1,1) string {mustBeMember(options.Discretization,["continuous","implicit","explicit"])} = "continuous"
            end
            P=design.P;T=design.T;kappa=design.kappa;n=size(P,1);
            assert(isreal(P) && isequal(size(P),[n,n]) && all(isfinite(P),'all') && ...
                norm(P-P.','fro')<=1e-12*max(1,norm(P,'fro')), ...
                'VehicleLocalization:InvalidLidarMetric','P must be finite symmetric positive definite.');
            [L,flag]=chol((P+P.')/2,'lower');
            assert(flag==0 && isequal(size(T),[n,n]) && isreal(T) && all(isfinite(T),'all') ...
                && rcond(T)>eps && isequal(size(G),[3,n]), ...
                'VehicleLocalization:InvalidLidarMetric','Require SPD P, nonsingular fixed T and a 3-by-n pose Jacobian.');
            validateattributes(kappa,{'numeric'},{'real','finite','nonnegative','scalar'});
            residualMode=isfield(measurement,'residual');gradientMode=isfield(measurement,'gradient');
            unsupportedGradient=0;
            assert(~(residualMode && gradientMode),'VehicleLocalization:AmbiguousLidarMeasurement', ...
                'Choose residual or gradient input.');
            if gradientMode
                assert(~isfield(measurement,'pose') && isfield(measurement,'information'), ...
                    'VehicleLocalization:AmbiguousLidarMeasurement','A gradient measurement carries information and no pose.');
                assert(isfield(measurement,'linearizationPose') && ...
                    isequal(measurement.linearizationPose(:),pose), ...
                    'VehicleLocalization:StaleLidarLinearization','Evaluate the cost gradient at the supplied predicted pose.');
                gradient=measurement.gradient(:);information=measurement.information;
                assert(numel(gradient)==3 && isreal(gradient) && all(isfinite(gradient)), ...
                    'VehicleLocalization:InvalidLidarGradient','The pose gradient must contain three finite values.');
            elseif residualMode
                assert(~isfield(measurement,'pose') && ~isfield(measurement,'information'), ...
                    'VehicleLocalization:AmbiguousLidarMeasurement','Choose residual or pose information input.');
                assert(isfield(measurement,'linearizationPose') && ...
                    isequal(measurement.linearizationPose(:),pose), ...
                    'VehicleLocalization:StaleLidarLinearization','Evaluate scan residuals at the supplied predicted pose.');
                r=measurement.residual(:);J=measurement.jacobian;W=measurement.weights;
                assert(isreal(r) && all(isfinite(r)) && isequal(size(J),[numel(r),3]) && ...
                    isreal(J) && all(isfinite(J),'all') && isreal(W) && all(isfinite(W),'all'), ...
                    'VehicleLocalization:InvalidLidarResidual','Invalid residual, Jacobian or weights.');
                if isvector(W) && numel(W)==numel(r)
                    assert(all(W>=0),'VehicleLocalization:InvalidLidarResidual','Diagonal weights must be nonnegative.');
                    WJ=W(:).*J;Wr=W(:).*r;
                else
                    assert(isequal(size(W),[numel(r),numel(r)]) && ...
                        norm(W-W.','fro')<=1e-10*max(1,norm(W,'fro')), ...
                        'VehicleLocalization:InvalidLidarResidual','Precision must be symmetric and aligned.');
                    W=(W+W.')/2;[V,E]=eig(W);ev=diag(E);
                    assert(all(ev>=-1e-10*max(1,norm(W,2))), ...
                        'VehicleLocalization:InvalidLidarResidual','Precision must be PSD.');
                    W=V*diag(max(ev,0))*V.';WJ=W*J;Wr=W*r;
                end
                information=J.'*WJ;gradient=J.'*Wr;
            else
                assert(isfield(measurement,'pose') && isfield(measurement,'information'), ...
                    'VehicleLocalization:InvalidLidarMeasurement','Supply pose and physical information.');
                information=measurement.information;
            end
            frame=1;direction=ones(3,1);
            if isfield(measurement,'frameReliability'),frame=measurement.frameReliability;end
            if isfield(measurement,'directionReliability'),direction=measurement.directionReliability(:);end
            variance=[];
            if isfield(measurement,'poseErrorVariance'),variance=measurement.poseErrorVariance;end
            filter=lidarInjectionSupport.filterLidarPoseInformation(information,cfg,frame,direction,variance);
            C=filter.D\(G*T);
            if residualMode
                xi=-filter.F*filter.D.'*gradient;representation="predictedPoseResidual";
            elseif gradientMode
                % Like a residual gradient J'*W*r, inject only where the cost has
                % positive curvature; unsupported directions receive no correction.
                normalized=filter.D.'*gradient;
                active=filter.eigenvalues>1e-10*max(1,max(filter.eigenvalues));
                U=filter.eigenvectors(:,active);supported=U*(U.'*normalized);
                xi=-filter.F*supported;representation="predictedPoseGradient";
                unsupportedGradient=norm(normalized-supported);
            else
                innovation=measurement.pose(:)-pose;
                assert(numel(innovation)==3 && isreal(innovation) && all(isfinite(innovation)), ...
                    'VehicleLocalization:InvalidLidarMeasurement','Pose must contain three finite values.');
                innovation(3)=atan2(sin(innovation(3)),cos(innovation(3)));
                xi=filter.S*(filter.D\innovation);representation="localPoseInformation";
            end
            Q=kappa*(C.'*filter.S*C);Q=(Q+Q.')/2;
            Z=L\Q/L.';Z=(Z+Z.')/2;
            eigenvalues=max(eig(Z),0);largest=max(eigenvalues);
            step=options.StepSize;v=kappa*(P\(C.'*xi));
            if options.Discretization=="continuous"
                correction=T*v;transition=eye(n);gainBound=kappa*norm(filter.F,2);
                if ~filter.calibrated,gainBound=kappa/filter.regularizer;end
            else
                if options.Discretization=="explicit" && largest>0,step=min(step,1.8/largest);end
                if options.Discretization=="implicit"
                    A=eye(n)+step*(P\Q);scaled=A\(step*v);transition=A\eye(n);
                else
                    scaled=step*v;transition=eye(n)-step*(P\Q);
                end
                correction=T*scaled;gainBound=NaN;
            end
            contraction=L.'*transition/L.';
            energyRatio=norm(contraction,2)^2;
            audit=struct('filter',filter,'C',C,'dissipationMatrix',Q, ...
                'continuousCorrection',T*v,'effectiveStrengths',kappa*filter.strengths, ...
                'maximumDissipationEigenvalue',largest,'requestedStep',options.StepSize, ...
                'appliedStep',step,'explicitStepProduct',step*largest, ...
                'jumpMaximumEnergyRatio',energyRatio, ...
                'jumpNonexpansive',energyRatio<=1+1e-10, ...
                'jumpCertificateApplies',options.Discretization~="continuous", ...
                'residualNoiseCoefficientBound',gainBound,'representation',representation, ...
                'unsupportedGradientNorm',unsupportedGradient, ...
                'scope',"Fixed-metric locally affine LiDAR channel only; no full-system or outage ISS claim");
        end

        function measurement=evaluateLidarRegistrationResidual(frozen,pose)
        % evaluateLidarRegistrationResidual Reevaluate admitted scan geometry at prediction.
        % FROZEN is the serializable lidarResidualModel exported by registration.
        % Associations, robust influence, latent support covariance and admitted pose
        % subspace stay fixed; residual and physical Jacobian are reevaluated together.
        % A directional model evaluates anchor+projector*(pose-anchor), preserving its
        % rejected directions in both innovation and information. Gaussian source
        % scatter still rotates, using the same analytic residuals as registration.
        % Between-hypothesis pose uncertainty is marginalized by a low-rank residual
        % whitening. It only reduces information and applies to BOTH r and J. The
        % resulting surrogate is conditional on associations and any position aiding.
            arguments
                frozen (1,1) struct
                pose (3,1) double {mustBeReal,mustBeFinite}
            end
            assert(isfield(frozen,'kind') && string(frozen.kind)=="frozen-semantic-lidar-v1", ...
                'VehicleLocalization:InvalidLidarResidualModel','Unknown frozen registration model.');
            anchor=frozen.anchorPose(:);projector=frozen.poseProjector;
            difference=pose-anchor;difference(3)=atan2(sin(difference(3)),cos(difference(3)));
            effective=anchor+projector*difference;
            local=[effective(1:2).'-frozen.origin(:).',effective(3)];
            method=string(frozen.method);
            switch method
                case "supportD2D"
                    [r,J]=registrationSupport.supportRegistrationResiduals(frozen.sourceMean,frozen.sourceCovariance, ...
                        frozen.targetMean,frozen.targetCovariance,frozen.sourceAxis, ...
                        frozen.targetNormal,frozen.directionScale,local,frozen.noiseStandardDeviation);
                case "anisotropicD2D"
                    [r,J]=registrationSupport.gaussianRegistrationResiduals(frozen.sourceMean,frozen.sourceCovariance, ...
                        frozen.targetMean,frozen.targetCovariance,local,frozen.noiseStandardDeviation);
                case "geometricD2D"
                    [r,J]=lineAndPointResiduals(frozen,local);
                otherwise
                    error('VehicleLocalization:InvalidLidarResidualModel','Unsupported frozen registration method.');
            end
            J=reshape(permute(J,[1,3,2]),[],3)*projector;
            influence=repelem(frozen.rowInfluence(:),size(r,1),1);
            assert(numel(influence)==numel(r) && isreal(influence) && all(isfinite(influence) & influence>=0), ...
                'VehicleLocalization:InvalidLidarResidualModel','Frozen influence must be finite and nonnegative.');
            root=sqrt(influence);r=root.*r(:);J=root.*J;
            spread=frozen.poseSpread;
            assert(isequal(size(spread),[3,3]) && isreal(spread) && all(isfinite(spread),'all') && ...
                norm(spread-spread.','fro')<=1e-10*max(1,norm(spread,'fro')), ...
                'VehicleLocalization:InvalidLidarResidualModel','Pose uncertainty must be symmetric PSD.');
            [V,E]=eig((spread+spread.')/2);values=diag(E);
            assert(min(values)>=-1e-10*max(1,norm(spread,2)), ...
                'VehicleLocalization:InvalidLidarResidualModel','Pose uncertainty must be PSD.');
            if any(values>0)
                root=V*diag(sqrt(max(values,0)))*V.';
                [U,S,~]=svd(J*root,'econ');attenuation=1-1./sqrt(1+diag(S).^2);
                r=r-U*(attenuation.*(U.'*r));J=J-U*(attenuation.*(U.'*J));
            end
            measurement=struct('residual',r,'jacobian',J,'weights',ones(size(r)), ...
                'linearizationPose',pose,'conditionedOnPositionAid',frozen.conditionedOnPositionAid, ...
                'informationCalibrated',false,'geometrySource',"Frozen accepted semantic registration");
        end

        function measurement=evaluateOverlapGradient(fixedCloud,movingCloud,pose,cfg)
        % evaluateOverlapGradient Correspondence-free LiDAR gradient at the observer prediction.
        % For every bandwidth sigma of cfg.scaleLadder both mixtures are smoothed by
        % N(0,sigma^2/2*I), which adds sigma^2*I to every pair covariance, so the
        % class-balanced cross energy E_sigma is the expected overlap under an
        % isotropic horizontal prediction error of that size; sigma=0 is the exact
        % score of scoreSemanticProbabilityCloudAlignment. The channel injects
        %   gradient    g = sum_sigma -grad(E_sigma)/E_sigma          (analytic SE(2))
        %   information M = sum_sigma sum_ij pi_ij J_ij' Sigma_ij^-1 J_ij
        % with pi_ij the pair responsibilities at scale sigma: the Gauss-Newton (EM)
        % metric of each -log E_sigma. It is positive semidefinite, exact for one
        % pair at any translation, and does not vanish at a kernel inflection, so
        % the regularized observer step stays bounded and points to the optimum.
        % Summing the scales fuses them in information form: the sharp scale
        % dominates near alignment, the wide ones keep a restoring force where the
        % sharp kernels have decayed. With cfg.evidenceWeights, map view
        % reliability and source temporal stability scale the mixture masses before
        % class balancing, as in registration. No pose is optimized, no
        % correspondence is selected and no acceptance test is applied; zero overlap
        % at every scale returns available=false. similarity reports the first
        % ladder entry with nonzero overlap. Map means are shifted to POSE before
        % evaluation, which leaves the overlap and its pose derivatives unchanged
        % and avoids UTM-scale cancellation.
        % With cfg.aggregation="landmark" the cost at each scale is instead the
        % per-landmark mixture likelihood -sum_j v_j log(eps + sum_i u_i N_ij) of
        % registrationSupport.semanticLandmarkLikelihood, with registration's support
        % geometry and, with cfg.orientation, its axis kernel. Map masses form one
        % density per class, source masses carry 1/C per class, and the evidence
        % factors multiply after this normalization. similarity is then the share
        % of source mass the map explains.
            arguments
                fixedCloud (1,1) struct
                movingCloud (1,1) struct
                pose (3,1) double {mustBeReal,mustBeFinite}
                cfg (1,1) struct = overlapGradientConfig()
            end
            timer=tic;linearization=pose;pose=pose(:).';
            ladder=cfg.scaleLadder(:).';
            validateattributes(ladder,{'double'},{'real','finite','nonnegative','nonempty','vector'});
            assert(isscalar(cfg.aggregation) && ismember(cfg.aggregation,["frame","landmark"]) && ...
                (~cfg.orientation || cfg.aggregation=="landmark"),'VehicleLocalization:InvalidOverlapAggregation', ...
                'aggregation is "frame" or "landmark"; orientation requires landmark aggregation.');
            validateattributes(cfg.outlierDensity,{'double'},{'real','finite','positive','scalar'});
            measurement=struct('gradient',zeros(3,1),'information',zeros(3),'linearizationPose',linearization, ...
                'available',false,'similarity',0,'minimumInformation',NaN,'scaleLadder',ladder, ...
                'scaleSimilarity',nan(size(ladder)),'mapComponents',0,'sourceComponents',0, ...
                'sharedClasses',strings(1,0),'informationCalibrated',false,'seconds',0, ...
                'geometrySource',"semanticGaussianOverlap scale ladder at the observer prediction; no registration");
            local=registrationSupport.selectLocalProbabilityCloud(fixedCloud,pose,cfg.localMapRadius);
            if local.components.numComponents==0 || movingCloud.components.numComponents==0
                measurement.seconds=toc(timer);return;
            end
            if cfg.viewConditioning,local=registrationSupport.conditionSemanticMapOnView(local,pose);end
            if cfg.aggregation=="landmark"
                measurement=landmarkLikelihoodGradient(local,movingCloud,pose,cfg,measurement);
                measurement.seconds=toc(timer);return;
            end
            [f,m]=registrationSupport.prepareSemanticRegistration(local,movingCloud,cfg.registration);
            if cfg.evidenceWeights
                % Map view reliability and source temporal stability scale the masses
                % before class balancing, so an unsupported or flickering landmark
                % cannot carry a whole class.
                if isfield(f,'viewReliability'),f.mixtureWeight=f.mixtureWeight.*double(f.viewReliability(:));end
                if isfield(m,'temporalStability'),m.mixtureWeight=m.mixtureWeight.*double(m.temporalStability(:));end
            end
            measurement.mapComponents=f.numComponents;measurement.sourceComponents=m.numComponents;
            f.mean(:,1:2)=f.mean(:,1:2)-pose(1:2);
            origin=[0,0,pose(3)];gradient=zeros(3,1);information=zeros(3);
            for k=1:numel(ladder)
                fk=f;mk=m;
                if ladder(k)>0
                    inflation=ladder(k)^2/2*eye(2); % isotropic, hence rotation invariant
                    fk.covariance(1:2,1:2,:)=fk.covariance(1:2,1:2,:)+inflation;
                    mk.covariance(1:2,1:2,:)=mk.covariance(1:2,1:2,:)+inflation;
                end
                [fk,mk]=registrationSupport.balanceSemanticDistributions(fk,mk);
                if k==1
                    shared=intersect(unique(fk.semanticName(fk.mixtureWeight>0)),unique(mk.semanticName(mk.mixtureWeight>0)));
                    measurement.sharedClasses=reshape(string(shared),1,[]);
                end
                [energy,ascent,metric]=registrationSupport.semanticGaussianOverlap(fk,mk,origin);
                if ~(energy>realmin),continue;end
                gradient=gradient-ascent(:)/energy;information=information+metric;
                measurement.scaleSimilarity(k)=min(energy,1);
            end
            evaluated=isfinite(measurement.scaleSimilarity);
            if any(evaluated)
                information=(information+information.')/2;
                measurement.gradient=gradient;measurement.information=information;
                measurement.similarity=measurement.scaleSimilarity(find(evaluated,1));
                measurement.minimumInformation=min(eig(information));
                measurement.available=true;
            end
            measurement.seconds=toc(timer);
        end

        function features=lidarPoseErrorFeatures(model)
        % lidarPoseErrorFeatures Conditional cluster-sandwich pose-error predictors.
        % Evaluate at the fitted measurement, never at the observer prediction. Each
        % matched distribution is one residual cluster. These predictors are NOT a
        % calibrated covariance; a held-out empirical error calibration is required.
        % Objective rescaling cancels between the normal inverse and cluster scores.
            arguments
                model (1,1) struct
            end
            m=lidarInjectionSupport.evaluateLidarRegistrationResidual(model,model.anchorPose(:));
            J=m.jacobian;r=m.residual;H=J.'*J;inverse=pinv((H+H.')/2);
            count=numel(model.rowInfluence);width=numel(r)/count;
            assert(width==fix(width) && count>0,'VehicleLocalization:InvalidLidarResidualModel', ...
                'Residual clusters must correspond to matched distributions.');
            scores=zeros(3,count);
            for k=1:count
                ids=(k-1)*width+(1:width);scores(:,k)=J(ids,:).'*r(ids);
            end
            covariance=inverse*(scores*scores.')*inverse;
            features=max([trace(covariance(1:2,1:2))/2;covariance(3,3)],0);
        end

        function variance=predictLidarPoseErrorVariance(features,calibration)
        % predictLidarPoseErrorVariance Predict uncentered conditional pose error.
        % Variances are per horizontal axis (m^2) and heading (rad^2). The nonnegative
        % intercept retains common errors invisible to within-scan residual scatter.
        % This does not subtract a fitted bias from the actual pose measurement.
            arguments
                features (2,1) double {mustBeReal,mustBeFinite,mustBeNonnegative}
                calibration (1,1) struct
            end
            assert(string(calibration.kind)=="conditional-pose-second-moment-v1", ...
                'VehicleLocalization:InvalidLidarCalibration','Unknown pose error calibration.');
            coefficients=calibration.coefficients;
            validateattributes(coefficients,{'double'},{'real','finite','nonnegative','size',[2,2]});
            variance=coefficients(:,1)+coefficients(:,2).*features;
            assert(all(variance>0),'VehicleLocalization:InvalidLidarCalibration', ...
                'Calibrated error second moments must be strictly positive.');
        end

        function measurement = buildLidarLineMeasurement(points,mapPoints,normals,pose,weights)
        % buildLidarLineMeasurement Frozen planar point-to-line residuals at prediction.
        % Rows are deskewed vehicle-frame scan XY, associated map XY, and map normals.
        % Extrinsics must already be applied. Weights include uncertainty and robust
        % reliability, with a vector for independent residuals or full PSD precision
        % for correlated ones. Associations, map, calibration and tilt are external
        % assumptions; this routine does not estimate nuisance variables or covariance.
            arguments
                points (:,2) double {mustBeReal,mustBeFinite}
                mapPoints (:,2) double {mustBeReal,mustBeFinite}
                normals (:,2) double {mustBeReal,mustBeFinite}
                pose (3,1) double {mustBeReal,mustBeFinite}
                weights double {mustBeReal,mustBeFinite}
            end
            assert(size(points,1)==size(mapPoints,1) && isequal(size(points),size(normals)) ...
                && ~isempty(points) && all(abs(vecnorm(normals,2,2)-1)<1e-8), ...
                'VehicleLocalization:InvalidLidarGeometry','Require aligned points and unit map normals.');
            R=[cos(pose(3)),-sin(pose(3));sin(pose(3)),cos(pose(3))];
            transformed=points*R.'+pose(1:2).';
            derivative=points*(R*[0,-1;1,0]).';
            measurement=struct('residual',sum(normals.*(transformed-mapPoints),2), ...
                'jacobian',[normals,sum(normals.*derivative,2)],'weights',weights, ...
                'linearizationPose',pose);
        end
    end
end

function [r,J]=lineAndPointResiduals(f,pose)
    R=[cos(pose(3)),-sin(pose(3));sin(pose(3)),cos(pose(3))];
    dR=R*[0,-1;1,0];n=size(f.sourceMean,1);
    delta=f.sourceMean*R.'+pose(1:2)-f.targetMean;
    derivative=f.sourceMean*dR.';
    r=zeros(3,n);J=zeros(3,3,n);B=f.precisionRoot;
    r(1:2,:)=reshape(pagemtimes(B,reshape(delta.',2,1,n)),2,n);
    J(1:2,1:2,:)=B;
    J(1:2,3,:)=pagemtimes(B,reshape(derivative.',2,1,n));
    r(3,:)=(sum((f.sourceAxis*R.').*f.targetNormal,2).*f.directionScale).';
    J(3,3,:)=sum((f.sourceAxis*dR.').*f.targetNormal,2).*f.directionScale;
end

function measurement=landmarkLikelihoodGradient(local,movingCloud,pose,cfg,measurement)
% Per-landmark mixture likelihood at every ladder scale (semanticLandmarkLikelihood).
% Map masses form one density per class and source masses carry 1/C per class;
% evidence factors multiply after this normalization, as registration's weights.
% The support geometry (axes, confidences, angular variances) is registration's.
    ladder=measurement.scaleLadder;
    geometry=prepareSemanticRegistrationGeometry(local,movingCloud,pose,cfg.registration);
    f=geometry.fixed;m=geometry.moving; % map means relative to POSE
    f.mean=f.mean(:,1:2);m.mean=m.mean(:,1:2);f.covariance=f.planarCovariance;m.covariance=m.planarCovariance;
    names=intersect(unique(f.semanticName(f.mixtureWeight>0)),unique(m.semanticName(m.mixtureWeight>0)));
    measurement.mapComponents=f.numComponents;measurement.sourceComponents=m.numComponents;
    measurement.sharedClasses=reshape(string(names),1,[]);
    if isempty(names),return;end
    u=zeros(size(f.mixtureWeight));v=zeros(size(m.mixtureWeight));
    for name=names.'
        I=f.semanticName==name & f.mixtureWeight>0;J=m.semanticName==name & m.mixtureWeight>0;
        u(I)=f.mixtureWeight(I)/sum(f.mixtureWeight(I));v(J)=m.mixtureWeight(J)/sum(m.mixtureWeight(J))/numel(names);
    end
    if cfg.evidenceWeights,u=u.*f.viewReliability(:);v=v.*m.temporalStability(:);end
    f.mixtureWeight=u;m.mixtureWeight=v;
    angularFloor=NaN;if cfg.orientation,angularFloor=cfg.registration.support.angularFloor;end
    origin=[0,0,pose(3)];gradient=zeros(3,1);information=zeros(3);
    for k=1:numel(ladder)
        fk=f;mk=m;
        if ladder(k)>0
            inflation=ladder(k)^2/2*eye(2);
            fk.covariance=fk.covariance+inflation;mk.covariance=mk.covariance+inflation;
        end
        [~,g,M,explained]=registrationSupport.semanticLandmarkLikelihood(fk,mk,origin,cfg.outlierDensity,angularFloor);
        if ~(explained>0),continue;end
        gradient=gradient+g;information=information+M;
        measurement.scaleSimilarity(k)=explained/max(sum(v),realmin);
    end
    evaluated=isfinite(measurement.scaleSimilarity);
    if any(evaluated)
        information=(information+information.')/2;
        measurement.gradient=gradient;measurement.information=information;
        measurement.similarity=measurement.scaleSimilarity(find(evaluated,1));
        measurement.minimumInformation=min(eig(information));
        measurement.available=true;
        measurement.geometrySource="semanticLandmarkLikelihood scale ladder at the observer prediction; no registration";
    end
end
