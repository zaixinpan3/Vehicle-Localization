classdef registrationSupport
% registrationSupport: Shared semantic D2D geometry and pose-event helpers.
% Static methods preserve cloud projection, preparation, class balancing,
% Gaussian overlap and accepted-event validation in one implementation.
% Example: cloud = registrationSupport.projectSemanticProbabilityCloud(cloud,3).

    methods (Static)
        function evidence = getHeightEvidence(cloud)
        % Current-acquisition XYZ evidence may accompany a pooled XY cloud.
        % It is not asserted to be the joint distribution of that pooled cloud.
            c=cloud.components;n=c.numComponents;
            evidence=struct('mean',zeros(n,3),'covariance',zeros(3,3,n), ...
                'available',false(n,1));
            if isfield(cloud,'heightEvidence')
                evidence=cloud.heightEvidence;
            elseif size(c.mean,2)==3
                evidence.mean=c.mean;evidence.covariance=c.covariance;
                evidence.available=true(n,1);
            elseif all(isfield(c,{'meanXYZ','covarianceXYZ','heightAvailable'}))
                evidence.mean=c.meanXYZ;evidence.covariance=c.covarianceXYZ;
                evidence.available=logical(c.heightAvailable(:));
            end
            assert(isequal(size(evidence.mean),[n 3]) && ...
                size(evidence.covariance,1)==3 && size(evidence.covariance,2)==3 && ...
                size(evidence.covariance,3)==n && numel(evidence.available)==n, ...
                'VehicleLocalization:InvalidHeightEvidence','Height evidence must align with cloud components.');
            use=logical(evidence.available(:));evidence.available=use;
            assert(isreal(evidence.mean)&&isreal(evidence.covariance) && ...
                all(isfinite(evidence.mean(use,:)),'all') && ...
                all(isfinite(evidence.covariance(:,:,use)),'all'), ...
                'VehicleLocalization:InvalidHeightEvidence','Available height evidence must be finite and real.');
        end

        function status = validateRegistrationCalibration(fixedCloud,movingCloud)
        % Require matching explicit extrinsic provenance for map/source/history.
            assert(isfield(fixedCloud,'frameCalibration') && isfield(movingCloud,'frameCalibration'), ...
                'VehicleLocalization:MissingCalibration','Both clouds require frameCalibration.');
            fixed=validateLidarFrameCalibration(fixedCloud.frameCalibration);
            moving=validateLidarFrameCalibration(movingCloud.frameCalibration);
            same=norm(fixed.rotation-moving.rotation,'fro')<1e-8 && ...
                norm(fixed.translation-moving.translation)<1e-8;
            assert(same,'VehicleLocalization:CalibrationMismatch', ...
                'Map and source require the same frame calibration; rebuild the map with the selected transform.');
            status="verifiedTransform";
        end

        function projected = projectSemanticProbabilityCloud(cloud, dimension)
        % projectSemanticProbabilityCloud: Select XYZ or its exact XY marginal.
        % Mixture mass is unchanged. Height is a normalized conditional density,
        % not a second peak amplitude or an extra length-dependent mixture weight.
            assert(isscalar(dimension) && ismember(dimension,[2 3]), 'Expected dimension 2 or 3.');
            source = mappingSupport.validateSemanticProbabilityCloud(cloud);
            if dimension == 3 && size(source.mean,2)==2
                assert(isfield(source,'heightAvailable') && all(source.heightAvailable), ...
                    'VehicleLocalization:HeightUnavailable','This cloud has no complete height model.');
                means = source.meanXYZ;
                covariance = source.covarianceXYZ;
            else
                means = source.mean(:,1:dimension);
                covariance = source.covariance(1:dimension,1:dimension,:);
            end
            components = struct('semanticName',source.semanticName,'mean',means, ...
                'covariance',covariance,'mixtureWeight',source.mixtureWeight,'numComponents',source.numComponents);
            for name=["semanticProbability","occupancyProbability","supportAmplitude","repeatability", ...
                    "temporalStability","detectionFrameCount","viewReliability"]
                if isfield(source,name), components.(name)=source.(name); end
            end
            projected = struct('components',components,'dimension',dimension);
            if isfield(source,'intrinsicCovariance'),projected.components.intrinsicCovariance=source.intrinsicCovariance;end
            if isfield(cloud,'frameCalibration'), projected.frameCalibration=cloud.frameCalibration; end
            if isfield(cloud,'coordinateFrame'), projected.coordinateFrame=cloud.coordinateFrame; end
            if dimension==2 && isfield(cloud,'landmarkViews'),projected.landmarkViews=cloud.landmarkViews;end
            if dimension==3 && isfield(cloud,'spatialCoordinateFrame')
                projected.coordinateFrame=cloud.spatialCoordinateFrame;
            end
        end

        function [fixed, moving, details] = prepareSemanticRegistration(fixedCloud, movingCloud, cfg)
        % prepareSemanticRegistration: Resolve height availability and its reference.
        % A finite heightTranslation is moving-origin map Z in meters. Standard
        % deviations model vertical translation/tilt uncertainty in the moving cloud.
        % Auto uses the XY marginal if height or the reference is unavailable.
            fixed = mappingSupport.validateSemanticProbabilityCloud(fixedCloud);
            moving = mappingSupport.validateSemanticProbabilityCloud(movingCloud);
            calibrationStatus=registrationSupport.validateRegistrationCalibration(fixedCloud,movingCloud);
            mode = "xy";
            translation = NaN;
            heightSd = 0.20;
            tiltSd = deg2rad(0.5);
            if isfield(cfg,'heightMode'), mode=string(cfg.heightMode); end
            if isfield(cfg,'heightTranslation'), translation=cfg.heightTranslation; end
            if isfield(cfg,'heightStandardDeviation'), heightSd=cfg.heightStandardDeviation; end
            if isfield(cfg,'tiltStandardDeviation'), tiltSd=cfg.tiltStandardDeviation; end
            assert(isscalar(mode) && ismember(mode,["auto","xy","xyz"]), 'Invalid height mode.');
            assert(isnumeric(translation) && isreal(translation) && isscalar(translation) && ...
                (isfinite(translation)||isnan(translation)), 'Invalid height translation.');
            assert(isnumeric(heightSd) && isreal(heightSd) && isscalar(heightSd) && isfinite(heightSd) && heightSd>=0 && ...
                isnumeric(tiltSd) && isreal(tiltSd) && isscalar(tiltSd) && isfinite(tiltSd) && tiltSd>=0, 'Invalid height/tilt uncertainty.');
            available = hasHeight(fixed) && hasHeight(moving);
            useHeight = mode~="xy" && available && isfinite(translation);
            assert(mode~="xyz" || useHeight, 'VehicleLocalization:HeightUnavailable', ...
                'XYZ matching requires height in both clouds and a finite heightTranslation.');
            dimension = 2+useHeight;
            fixed = registrationSupport.projectSemanticProbabilityCloud(fixedCloud,dimension).components;
            moving = registrationSupport.projectSemanticProbabilityCloud(movingCloud,dimension).components;
            details = struct('dimension',dimension,'heightUsed',useHeight, ...
                'calibrationStatus',calibrationStatus, ...
                'heightTranslation',translation,'heightStandardDeviation',heightSd, ...
                'tiltStandardDeviation',tiltSd,'heightReason',"enabled");
            if useHeight
                for k=1:moving.numComponents
                    p=moving.mean(k,:);
                    % First-order gravity-aligned roll/pitch perturbation Jacobian.
                    jacobian=[0 p(3);-p(3) 0;p(2) -p(1)];
                    moving.covariance(:,:,k)=moving.covariance(:,:,k)+tiltSd^2*(jacobian*jacobian.');
                end
                moving.covariance(3,3,:)=moving.covariance(3,3,:)+heightSd^2;
                moving.mean(:,3)=moving.mean(:,3)+translation;
            elseif mode=="xy"
                details.heightReason="explicitXY";
            elseif ~available
                details.heightReason="heightUnavailable";
            else
                details.heightReason="verticalReferenceUnavailable";
            end
        end

        function [fixed, moving] = balanceSemanticDistributions(fixed, moving)
        % balanceSemanticDistributions: Equalize shared class L2 energies for D2D.
        % Each shared semantic field has unit self energy, then receives 1/C of the
        % objective. The resulting overlap is the mean class-conditional normalized
        % overlap. This prevents long curb support from overwhelming sparse landmarks.
        % These internal weights are Hilbert-space scaling, not probability masses.
            names=intersect(unique(fixed.semanticName(fixed.mixtureWeight>0)), ...
                unique(moving.semanticName(moving.mixtureWeight>0)));
            fixedWeights=zeros(size(fixed.mixtureWeight));
            movingWeights=zeros(size(moving.mixtureWeight));
            for name=names.'
                f=fixed; m=moving;
                f.mixtureWeight(f.semanticName~=name)=0;
                m.mixtureWeight(m.semanticName~=name)=0;
                fe=registrationSupport.semanticGaussianOverlap(f,f,[0 0 0]);
                me=registrationSupport.semanticGaussianOverlap(m,m,[0 0 0]);
                fixedWeights=fixedWeights+f.mixtureWeight/sqrt(max(fe*numel(names),realmin));
                movingWeights=movingWeights+m.mixtureWeight/sqrt(max(me*numel(names),realmin));
            end
            fixed.mixtureWeight=fixedWeights;
            moving.mixtureWeight=movingWeights;
        end

        function [energy, gradient, metric] = semanticGaussianOverlap(fixed, moving, pose)
        % semanticGaussianOverlap: Exact same-class integral and SE(2) derivatives.
        % Integral N(x;a,A) N(x;b,B) dx = N(a;b,A+B). Covariance is
        % rotated with the mean; derivatives include the rotating covariance term.
        % The optional third output is the Gauss-Newton metric of -log(energy):
        % the responsibility-weighted sum of pair precisions in pose coordinates,
        % sum_ij pi_ij J_ij' Sigma_ij^-1 J_ij with pi_ij = w_i w_j N_ij / energy and
        % J_ij = d(delta_ij)/d[x y psi], so the yaw block carries the lever arm. It is
        % positive semidefinite and exact for a single pair at any translation.
        % Inputs are component structs already validated at the API boundary.
            assert(nargout<3 || size(fixed.mean,2)==2,'VehicleLocalization:MetricRequiresXY', ...
                'The Gauss-Newton overlap metric is defined for the XY marginal only.');
            if size(fixed.mean,2)==3
                if nargout > 1
                    [energy,gradient]=spatialGaussianOverlap(fixed,moving,pose);
                else
                    energy=spatialGaussianOverlap(fixed,moving,pose);
                end
                return;
            end
            c = cos(pose(3)); s = sin(pose(3));
            rotation = [c -s; s c];
            means = moving.mean*rotation.' + pose(1:2);
            meanDerivative = moving.mean*[-s -c; c -s].';
            a0 = reshape(moving.covariance(1,1,:), [], 1);
            b0 = reshape(moving.covariance(1,2,:), [], 1);
            d0 = reshape(moving.covariance(2,2,:), [], 1);
            ma = c*c*a0 - 2*c*s*b0 + s*s*d0;
            mb = c*s*(a0-d0) + (c*c-s*s)*b0;
            md = s*s*a0 + 2*c*s*b0 + c*c*d0;
            energy = 0; gradient = zeros(1,3); metric = zeros(3);
            fixedActive = fixed.mixtureWeight > 0;
            movingActive = moving.mixtureWeight > 0;
            names = intersect(unique(fixed.semanticName(fixedActive)), unique(moving.semanticName(movingActive)));
            for name = names.'
                f = find(fixed.semanticName == name & fixedActive);
                allMoving = find(moving.semanticName == name & movingActive);
                fa = reshape(fixed.covariance(1,1,f), [], 1);
                fb = reshape(fixed.covariance(1,2,f), [], 1);
                fd = reshape(fixed.covariance(2,2,f), [], 1);
                % Bound temporary pair arrays for large offline maps.
                blockSize = max(1, floor(250000/max(numel(f),1)));
                for start = 1:blockSize:numel(allMoving)
                    m = allMoving(start:min(start+blockSize-1,end));
                    a = fa + ma(m).'; b = fb + mb(m).'; d = fd + md(m).';
                    determinant = a.*d - b.*b;
                    dx = fixed.mean(f,1) - means(m,1).';
                    dy = fixed.mean(f,2) - means(m,2).';
                    qx = (d.*dx-b.*dy)./determinant;
                    qy = (a.*dy-b.*dx)./determinant;
                    kernel = exp(-0.5*(dx.*qx+dy.*qy))./(2*pi*sqrt(determinant));
                    weighted = kernel.*(fixed.mixtureWeight(f)*moving.mixtureWeight(m).');
                    energy = energy + sum(weighted,'all');
                    if nargout > 1
                        da = -2*mb(m).'; db = (ma(m)-md(m)).'; dd = 2*mb(m).';
                        rotationTerm = qx.*meanDerivative(m,1).' + qy.*meanDerivative(m,2).' + ...
                            0.5*(qx.^2.*da+2*qx.*qy.*db+qy.^2.*dd - ...
                            (d.*da+a.*dd-2*b.*db)./determinant);
                        gradient = gradient + [sum(weighted.*qx,'all'), sum(weighted.*qy,'all'), ...
                            sum(weighted.*rotationTerm,'all')];
                    end
                    if nargout > 2
                        lx = meanDerivative(m,1).'; ly = meanDerivative(m,2).';
                        sx = (d.*lx-b.*ly)./determinant; sy = (a.*ly-b.*lx)./determinant;
                        metric = metric + [sum(weighted.*d./determinant,'all'), -sum(weighted.*b./determinant,'all'), sum(weighted.*sx,'all'); ...
                            0, sum(weighted.*a./determinant,'all'), sum(weighted.*sy,'all'); ...
                            0, 0, sum(weighted.*(lx.*sx+ly.*sy),'all')];
                    end
                end
            end
            if nargout > 2
                metric = metric + triu(metric,1).';
                if energy > 0, metric = metric/energy; end
            end
        end

        function [cost, gradient, metric, explained] = semanticLandmarkLikelihood(fixed, moving, pose, outlierDensity, angularFloor)
        % semanticLandmarkLikelihood: Per-landmark mixture likelihood and SE(2) derivatives.
        %   cost = -sum_j v_j log(outlierDensity + sum_i u_i N_ij K_ij)
        % over same-class pairs. N_ij = N(mu_i; R*mu_j+t, A_i+R*B_j*R') is the
        % Gaussian overlap integral, u the fixed masses (one density per class),
        % v the moving masses and outlierDensity (1/m^2) a uniform outlier floor,
        % so an unexplained landmark loses its influence. With a finite
        % angularFloor, components also carry an axis: K_ij = exp(-kappa_ij*s_ij^2/2)
        % with s_ij the sine between the fixed axis and the rotated moving axis and
        % kappa_ij = c_i*c_j/(angularFloor^2+a_i+a_j), from the support geometry of
        % prepareSemanticRegistrationGeometry (orientationNormal or supportTangent,
        % axisConfidence c, angularVariance a). GRADIENT is d(cost)/d[x y psi]
        % (3-by-1) and METRIC the Gauss-Newton metric sum_j v_j sum_i r_ij
        % (J_ij'*Sigma_ij^-1*J_ij + kappa_ij*ds_ij^2*e3*e3'), with responsibilities
        % r_ij = u_i N_ij K_ij/(outlierDensity + sum_i u_i N_ij K_ij). EXPLAINED is
        % the moving mass explained by the fixed mixture, sum_j v_j(1-r_0j).
            assert(size(fixed.mean,2)==2 && size(moving.mean,2)==2,'VehicleLocalization:LikelihoodRequiresXY', ...
                'The landmark likelihood is defined for the XY marginal only.');
            oriented = isfinite(angularFloor);
            c = cos(pose(3)); s = sin(pose(3));
            rotation = [c -s; s c]; rate = [-s -c; c -s];
            means = moving.mean*rotation.' + pose(1:2);
            lever = moving.mean*rate.';
            a0 = reshape(moving.covariance(1,1,:), [], 1);
            b0 = reshape(moving.covariance(1,2,:), [], 1);
            d0 = reshape(moving.covariance(2,2,:), [], 1);
            ma = c*c*a0 - 2*c*s*b0 + s*s*d0;
            mb = c*s*(a0-d0) + (c*c-s*s)*b0;
            md = s*s*a0 + 2*c*s*b0 + c*c*d0;
            if oriented
                axis = moving.supportTangent*rotation.'; axisRate = moving.supportTangent*rate.';
            end
            cost = 0; gradient = zeros(3,1); metric = zeros(3); explained = 0;
            names = intersect(unique(fixed.semanticName(fixed.mixtureWeight>0)), ...
                unique(moving.semanticName(moving.mixtureWeight>0)));
            for name = names.'
                f = find(fixed.semanticName == name & fixed.mixtureWeight > 0);
                allMoving = find(moving.semanticName == name & moving.mixtureWeight > 0);
                u = fixed.mixtureWeight(f);
                fa = reshape(fixed.covariance(1,1,f), [], 1);
                fb = reshape(fixed.covariance(1,2,f), [], 1);
                fd = reshape(fixed.covariance(2,2,f), [], 1);
                blockSize = max(1, floor(250000/max(numel(f),1)));
                for start = 1:blockSize:numel(allMoving)
                    m = allMoving(start:min(start+blockSize-1,end));
                    a = fa + ma(m).'; b = fb + mb(m).'; d = fd + md(m).';
                    determinant = a.*d - b.*b;
                    dx = fixed.mean(f,1) - means(m,1).';
                    dy = fixed.mean(f,2) - means(m,2).';
                    qx = (d.*dx-b.*dy)./determinant;
                    qy = (a.*dy-b.*dx)./determinant;
                    kernel = exp(-0.5*(dx.*qx+dy.*qy))./(2*pi*sqrt(determinant));
                    lx = lever(m,1).'; ly = lever(m,2).';
                    da = -2*mb(m).'; db = (ma(m)-md(m)).'; dd = 2*mb(m).';
                    yaw = qx.*lx + qy.*ly + 0.5*(qx.^2.*da+2*qx.*qy.*db+qy.^2.*dd - ...
                        (d.*da+a.*dd-2*b.*db)./determinant);
                    sx = (d.*lx-b.*ly)./determinant; sy = (a.*ly-b.*lx)./determinant;
                    yawMetric = lx.*sx + ly.*sy;
                    if oriented
                        sine = fixed.orientationNormal(f,:)*axis(m,:).';
                        sineRate = fixed.orientationNormal(f,:)*axisRate(m,:).';
                        kappa = (fixed.axisConfidence(f)*moving.axisConfidence(m).')./ ...
                            (angularFloor^2 + fixed.angularVariance(f) + moving.angularVariance(m).');
                        kernel = kernel.*exp(-0.5*kappa.*sine.^2);
                        yaw = yaw - kappa.*sine.*sineRate;
                        yawMetric = yawMetric + kappa.*sineRate.^2;
                    end
                    v = moving.mixtureWeight(m).';
                    density = u.'*kernel;
                    omega = (u.*kernel)./(outlierDensity+density).*v;
                    cost = cost - sum(v.*log(outlierDensity+density));
                    explained = explained + sum(v.*density./(outlierDensity+density));
                    gradient = gradient - [sum(omega.*qx,'all'); sum(omega.*qy,'all'); sum(omega.*yaw,'all')];
                    metric = metric + [sum(omega.*d./determinant,'all'), -sum(omega.*b./determinant,'all'), sum(omega.*sx,'all'); ...
                        0, sum(omega.*a./determinant,'all'), sum(omega.*sy,'all'); ...
                        0, 0, sum(omega.*yawMetric,'all')];
                end
            end
            metric = metric + triu(metric,1).';
        end

        function measurement = registrationPoseMeasurement(result,timestamp,arrivalTime)
        % registrationPoseMeasurement Export full or explicitly accepted directional data.
        % partialPoseAvailable alone never authorizes export. Optional arrivalTime
        % uses the acquisition clock. Omission is marked as a delivery placeholder.
            assert(~isfield(result,'independentLidarMeasurement') || result.independentLidarMeasurement, ...
                'VehicleLocalization:FusedPoseIsNotLidar','Do not export a fused graph pose as independent LiDAR.');
            assert(isscalar(timestamp) && isfinite(timestamp),'Invalid acquisition time.');
            placeholder=nargin<3 || isempty(arrivalTime);
            if placeholder, arrivalTime=timestamp; end
            assert(isscalar(arrivalTime) && isfinite(arrivalTime) && arrivalTime>=timestamp, ...
                'Invalid delivery time.');
            measurement=[];
            directional=isfield(result,'directionalAccepted') && result.directionalAccepted;
            if ~result.accepted && ~directional, return; end
            assert(isfield(result,'information'), ...
                'VehicleLocalization:RegistrationInformationUnavailable', ...
                'Accepted D2D registration must provide its pose information matrix.');
            information=double(result.information);
            kind="fullPose";rank=3;projector=eye(3);
            if ~result.accepted && directional
                assert(result.supportedConverged && result.observableRank>0 && result.observableRank<3, ...
                    'VehicleLocalization:InvalidRegistrationInformation','Directional result lacks supported convergence.');
                information=double(result.directionalInformation);
                kind="directionalPose";rank=result.observableRank;
                projector=result.physicalObservableProjector;
            end
            assert(isequal(size(information),[3 3]) && isreal(information) && ...
                all(isfinite(information),'all') && ...
                norm(information-information.','fro')<=1e-10*max(1,norm(information,'fro')), ...
                'VehicleLocalization:InvalidRegistrationInformation','Invalid pose information matrix.');
            if kind=="fullPose"
                [~,failure]=chol(information);
                assert(failure==0,'VehicleLocalization:InvalidRegistrationInformation', ...
                    'A full accepted pose must have positive definite information.');
            else
                values=eig((information+information.')/2);tol=1e-10*max(1,norm(information,2));
                assert(min(values)>=-tol && max(values)>tol ...
                    && isequal(size(projector),[3 3]) && all(isfinite(projector),'all') ...
                    && norm(projector*projector-projector,'fro')<1e-8 ...
                    && abs(trace(projector)-rank)<1e-8 ...
                    && norm(information*(eye(3)-projector),'fro')<1e-8*max(1,norm(information,'fro')), ...
                    'VehicleLocalization:InvalidRegistrationInformation','Invalid supported PSD information or projector.');
            end
            measurement=struct('timestamp',double(timestamp),'arrivalTime',double(arrivalTime), ...
                'pose',result.poseXYTheta,'information',information,'measurementType',kind, ...
                'observableRank',rank,'observableProjector',projector, ...
                'observableProjectorCoordinates',"additive map X,Y,psi; oblique projector", ...
                'arrivalTimeIsPlaceholder',placeholder);
            measurement.conditionedOnPositionAid=isfield(result,'positionAiding') && result.positionAiding.used;
            measurement.positionAidInformationAdded=false;
            if isfield(result,'lidarResidualModel')
                measurement.lidarResidualModel=result.lidarResidualModel;
            end
        end

        function [fixed,indices]=selectLocalProbabilityCloud(map,seed,radius)
        % selectLocalProbabilityCloud Select map components without changing their priors.
            c=map.components;keep=sum((c.mean(:,1:2)-seed(1:2)).^2,2)<=radius^2;
            % A local subcloud retains original priors, not whole-map normalization
            % claims such as totalMass/classTotalMass for components outside the crop.
            fixed=struct('components',struct());
            for name=["frameCalibration","coordinateFrame","dimension"]
                if isfield(map,name),fixed.(name)=map.(name);end
            end
            fixed.components.mean=c.mean(keep,:);
            fixed.components.covariance=c.covariance(:,:,keep);
            fixed.components.numComponents=nnz(keep);indices=find(keep);
            if isfield(map,'landmarkViews')
                fixed.landmarkViews=map.landmarkViews;
                fixed.landmarkViews.observations=map.landmarkViews.observations(keep);
            end
            for name=["semanticName","mixtureWeight","repeatability","semanticProbability","occupancyProbability", ...
                    "supportAmplitude","meanXYZ","heightAvailable","componentId","viewReliability"]
                if isfield(c,name),fixed.components.(name)=c.(name)(keep,:);end
            end
            if isfield(c,'covarianceXYZ'),fixed.components.covarianceXYZ=c.covarianceXYZ(:,:,keep);end
            if isfield(c,'intrinsicCovariance'),fixed.components.intrinsicCovariance=c.intrinsicCovariance(:,:,keep);end
            if isfield(map,'heightEvidence')
                e=map.heightEvidence;fixed.heightEvidence=e;
                fixed.heightEvidence.mean=e.mean(keep,:);
                fixed.heightEvidence.covariance=e.covariance(:,:,keep);
                fixed.heightEvidence.available=e.available(keep);
            end
        end

        function result=matchLocalProbabilityCloud(map,source,seed,cfg,positionAid,additionalSeeds)
        % matchLocalProbabilityCloud Crop a semantic map and retain global pair indices.
        % The source is the confirmed coarse horizon, already in current body axes.
            if nargin<5,positionAid=[];end
            if nargin<6,additionalSeeds=zeros(0,3);end
            [fixed,indices]=registrationSupport.selectLocalProbabilityCloud(map,seed,cfg.localMapRadius);
            timer=tic;result=registerSemanticProbabilityCloud(fixed,source,seed,cfg,positionAid,additionalSeeds);
            result.matchingSeconds=toc(timer);
            if isfield(result,'correspondences')
                result.correspondences.globalTarget=indices(result.correspondences.target);
            end
        end

        function [cloud,details]=conditionSemanticMapOnView(cloud,predicted)
        % conditionSemanticMapOnView Evaluate a frozen map at the predicted origin.
        % Relative kernel weights determine mean and within-plus-between scatter.
        % Intrinsic scatter retains within-acquisition shape separately from drift
        % between acquisition centers, for partial sign surface compatibility.
        % Absolute nearest-view distance continuously reduces influence where the
        % mapping drive supplies little view support. It is not a pose observation.
        % The solver freezes these map moments for its solve. Reported information
        % is conditional on this view and is not a calibrated or independent posterior.
            details=struct('enabled',false,'conditionedComponents',0,'queryOrigin',predicted, ...
                'informationCalibrated',false);
            if ~isfield(cloud,'landmarkViews'),return;end
            validateattributes(predicted,{'numeric'},{'vector','numel',3,'finite','real'});
            predicted=double(predicted(:).');model=cloud.landmarkViews;cfg=model.config;
            validateLandmarkViewConfig(cfg);
            n=cloud.components.numComponents;
            assert(model.schemaVersion==1 && iscell(model.observations)&&numel(model.observations)==n, ...
                'VehicleLocalization:InvalidLandmarkViews','Map view arrays must align with component indices.');
            cloud.components.viewReliability=ones(n,1);modeled=ismember(cloud.components.semanticName,cfg.pointClasses);
            cloud.components.intrinsicCovariance=zeros(2,2,n);
            cloud.components.viewReliability(modeled)=0;
            for id=find(modeled).'
                o=model.observations{id};if isempty(o),continue;end
                assert(isnumeric(o)&&isreal(o)&&size(o,2)==9&&all(isfinite(o),'all'), ...
                    'VehicleLocalization:InvalidLandmarkViews','Require finite acquisition-origin and Gaussian observation rows.');
                yaw=atan2(sin(o(:,3)-predicted(3)),cos(o(:,3)-predicted(3)));
                o=o(abs(yaw)<=cfg.maximumHeadingDifference,:);if isempty(o),continue;end
                d2=sum((o(:,1:2)-predicted(1:2)).^2,2);minimum=min(d2);
                w=exp(-.5*(d2-minimum)/cfg.bandwidth^2);w=w/sum(w);
                mu=sum(o(:,4:5).*w,1);delta=o(:,4:5)-mu;
                intrinsic=[sum(w.*o(:,6)),sum(w.*o(:,7));sum(w.*o(:,7)),sum(w.*o(:,8))];
                cloud.components.intrinsicCovariance(:,:,id)=intrinsic;
                S=intrinsic+delta.'*(delta.*w);
                [V,E]=eig((S+S.')/2,'vector');S=V*diag(max(E,cfg.varianceFloor))*V.';
                cloud.components.mean(id,:)=mu;cloud.components.covariance(:,:,id)=(S+S.')/2;
                cloud.components.viewReliability(id)=exp(-.5*minimum/cfg.coverageScale^2);
                details.conditionedComponents=details.conditionedComponents+1;
            end
            details.enabled=true;details.queryOrigin=predicted;
            details.modeledComponents=nnz(modeled);
            details.effectiveSupport=sum(cloud.components.viewReliability(modeled));
            cloud.landmarkViewConditioning=details;
        end

        function [cloud,groups]=canonicalizeSemanticCloud(cloud,radius,classes)
        % canonicalizeSemanticCloud Merge same-class point components closer than radius.
        % Greedy grouping in descending weight order: a component joins the first
        % group whose weighted centroid lies within radius, otherwise it starts one.
        % Each group becomes one Gaussian by moment matching (weighted mean, within
        % plus between scatter, summed weights), so class mass and the first two
        % moments of the mixture are preserved. Quality fields become weighted means,
        % counts maxima, frame masks unions; identifiers and any other per-component
        % field follow the heaviest member. Line classes (curb, facade) are never
        % merged. A radius of zero returns the cloud unchanged. groups lists the
        % original member indices of every output component in output order.
            arguments
                cloud (1,1) struct
                radius (1,1) double {mustBeNonnegative,mustBeFinite}
                classes (1,:) string=["pole","trafficSign"]
            end
            c=cloud.components;n=c.numComponents;groups=num2cell((1:n).');
            if radius<=0 || n<2,return;end
            assert(~isfield(cloud,'landmarkViews'),'VehicleLocalization:ViewMapAlreadyCanonical', ...
                'View-conditioned maps have already grouped point landmarks offline.');
            % Only relative weights matter, so a common scale of the stored prior
            % cannot change the grouping or the merged statistics.
            names=string(c.semanticName(:));w=double(c.mixtureWeight(:));w=max(w/max(max(w),realmin),eps);
            groups=cell(0,1);
            for name=unique(names).'
                idx=find(names==name);
                if ~ismember(name,classes) || numel(idx)<2,groups=[groups;num2cell(idx)];continue;end %#ok<AGROW>
                [~,order]=sort(w(idx),'descend');idx=idx(order);xy=c.mean(idx,1:2);
                centroid=zeros(numel(idx),2);mass=zeros(numel(idx),1);member=cell(numel(idx),1);count=0;
                for i=1:numel(idx)
                    j=0;
                    if count>0
                        [d,j]=min(sum((centroid(1:count,:)-xy(i,:)).^2,2));
                        if d>radius^2,j=0;end
                    end
                    if j>0
                        centroid(j,:)=(centroid(j,:)*mass(j)+xy(i,:)*w(idx(i)))/(mass(j)+w(idx(i)));
                        mass(j)=mass(j)+w(idx(i));member{j}(end+1,1)=idx(i);
                    else
                        count=count+1;centroid(count,:)=xy(i,:);mass(count)=w(idx(i));member{count}=idx(i);
                    end
                end
                groups=[groups;member(1:count)]; %#ok<AGROW>
            end
            [~,order]=sort(cellfun(@min,groups));groups=groups(order);
            m=numel(groups);heavy=zeros(m,1);merged=false(m,1);
            for g=1:m
                [~,j]=max(w(groups{g}));heavy(g)=groups{g}(j);merged(g)=numel(groups{g})>1;
            end
            % Every output component starts as its heaviest member; merged groups are
            % then overwritten with their moment-matched statistics.
            out=struct();
            for field=string(fieldnames(c)).'
                v=c.(field);
                if field=="numComponents",out.(field)=m;
                elseif ndims(v)==3 && size(v,3)==n,out.(field)=v(:,:,heavy);
                elseif ismatrix(v) && size(v,1)==n,out.(field)=v(heavy,:);
                else,out.(field)=v;
                end
            end
            fields=string(fieldnames(c)).';
            for g=find(merged).'
                i=groups{g};wi=w(i);total=sum(wi);
                out.mean(g,:)=sum(c.mean(i,:).*wi,1)/total;
                out.covariance(:,:,g)=momentMatch(c.covariance(:,:,i),c.mean(i,:),out.mean(g,:),wi);
                for field=intersect(["mixtureWeight","classMixtureWeight","mass","referenceMass"],fields)
                    out.(field)(g)=sum(double(c.(field)(i)));
                end
                for field=intersect(["temporalStability","semanticProbability","occupancyProbability"],fields)
                    out.(field)(g)=sum(double(c.(field)(i)).*wi)/total;
                end
                for field=intersect(["repeatability","detectionFrameCount"],fields)
                    out.(field)(g)=max(double(c.(field)(i)));
                end
                if isfield(c,'detectionFrameMask'),out.detectionFrameMask(g,:)=any(c.detectionFrameMask(i,:),1);end
                if isfield(c,'heightAvailable')
                    % The XYZ moments must marginalize exactly to the XY moments, so a
                    % merged component carries height only when every member does.
                    if all(c.heightAvailable(i))
                        out.heightAvailable(g)=true;mu=sum(c.meanXYZ(i,:).*wi,1)/total;mu(1:2)=out.mean(g,1:2);
                        S=momentMatch(c.covarianceXYZ(:,:,i),c.meanXYZ(i,:),mu,wi);S(1:2,1:2)=out.covariance(:,:,g);
                        out.meanXYZ(g,:)=mu;out.covarianceXYZ(:,:,g)=S;
                    else
                        out.heightAvailable(g)=false;out.meanXYZ(g,:)=[out.mean(g,1:2),0];
                        out.covarianceXYZ(:,:,g)=blkdiag(out.covariance(:,:,g),1);
                    end
                end
            end
            cloud.components=out;
            if isfield(cloud,'heightEvidence')
                h=cloud.heightEvidence;e=h;e.mean=h.mean(heavy,:);e.covariance=h.covariance(:,:,heavy);e.available=h.available(heavy);
                for g=find(merged).'
                    i=groups{g};wi=w(i);e.available(g)=all(h.available(i));
                    if e.available(g)
                        mu=sum(h.mean(i,:).*wi,1)/sum(wi);e.mean(g,:)=mu;e.covariance(:,:,g)=momentMatch(h.covariance(:,:,i),h.mean(i,:),mu,wi);
                    else
                        e.mean(g,:)=0;e.covariance(:,:,g)=eye(size(h.covariance,1));
                    end
                end
                cloud.heightEvidence=e;
            end
        end

        function [similarity, details] = scoreSemanticProbabilityCloudAlignment(fixedCloud, movingCloud, poseXYTheta, cfg)
        % scoreSemanticProbabilityCloudAlignment: Mean normalized per-class D2D overlap.
        % Components are planar Gaussians in meters; pose is moving-to-fixed [x,y,yaw]
        % with yaw in radians. Both means and covariances transform. Empty clouds
        % return zero similarity. gradient is with respect to the three pose entries.
            if nargin<4, cfg=distributionRegistrationConfig(); end
            [fixed,moving,heightDetails] = registrationSupport.prepareSemanticRegistration(fixedCloud,movingCloud,cfg);
            [fixed,moving] = registrationSupport.balanceSemanticDistributions(fixed,moving);
            pose = double(poseXYTheta(:).');
            assert(numel(pose)==3 && all(isfinite(pose)), 'Expected finite [x y yaw].');
            [crossEnergy, gradient] = registrationSupport.semanticGaussianOverlap(fixed, moving, pose);
            fixedSelf = registrationSupport.semanticGaussianOverlap(fixed, fixed, [0 0 0]);
            movingSelf = registrationSupport.semanticGaussianOverlap(moving, moving, [0 0 0]);
            normalization = sqrt(max(fixedSelf*movingSelf,0));
            similarity = 0;
            if normalization > 0
                similarity = min(max(crossEnergy/normalization,0),1);
                gradient = gradient/normalization;
            else
                gradient(:) = 0;
            end
            details = struct('crossEnergy',crossEnergy,'fixedSelfEnergy',fixedSelf, ...
                'movingSelfEnergy',movingSelf,'squaredL2Distance',max(fixedSelf+movingSelf-2*crossEnergy,0), ...
                'poseXYTheta',pose,'gradient',gradient);
            details.height=heightDetails;
        end

        function result=selectPositionAidedRegistration(solve,initialPose,aid,cfg,additionalSeeds)
        % selectPositionAidedRegistration Resolve local LiDAR modes with position aid.
        % aid.position/covariance refer to the map/observer point at scan acquisition.
        % Source/map stability remains inside solve. No GNSS term is added to its
        % objective or Hessian. Selection still creates statistical dependence on
        % GNSS; the exported matrix is conditional geometry, not independent evidence.
        % Hypothesis scores are engineering compatibility scores, not calibrated
        % posterior probabilities. Between-mode disagreement can only reduce information.
            initialPose=double(initialPose(:).');
            if nargin<5,additionalSeeds=zeros(0,3);end
            assert(size(additionalSeeds,2)==3 && isreal(additionalSeeds) && all(isfinite(additionalSeeds),'all'), ...
                'VehicleLocalization:InvalidRegistrationSeed','Additional seeds must be finite SE(2) poses.');
            result=solve(initialPose);
            result.positionAiding=struct('used',false,'reason',"unavailable", ...
                'selected',1,'candidateCount',1,'gnssInformationAdded',false, ...
                'selectionDependsOnGnss',false);
            if isempty(aid) || (isfield(aid,'valid') && ~aid.valid),return;end
            a=cfg.positionAid;
            parameters=[a.maximumStandardDeviation,a.standardDeviationFloor,a.minimumSeedSeparation, ...
                a.hypothesisSeparation,a.maximumSquaredInnovation];
            assert(all(isfinite(parameters) & parameters>0), ...
                'VehicleLocalization:InvalidPositionAidConfig','Position-aid thresholds must be finite and positive.');
            validateattributes(aid.position,{'numeric'},{'real','finite','numel',2});
            C=double(aid.covariance);
            assert(isequal(size(C),[2,2]) && all(isfinite(C),'all') && ...
                norm(C-C.','fro')<1e-9 && min(eig(C))>0, ...
                'VehicleLocalization:InvalidPositionAid','Require positive definite position covariance.');
            if sqrt(max(eig(C)))>a.maximumStandardDeviation
                result.positionAiding.reason="uncertainPosition";return;
            end
            C=C+a.standardDeviationFloor^2*eye(2);
            seeds=initialPose;gnssSeed=[double(aid.position(:).'),initialPose(3)];
            candidates={result};
            if norm(gnssSeed(1:2)-initialPose(1:2))>=a.minimumSeedSeparation
                seeds(end+1,:)=gnssSeed;candidates{end+1}=solve(gnssSeed);
            end
            for k=1:size(additionalSeeds,1)
                delta=seeds-additionalSeeds(k,:);delta(:,3)=wrap(delta(:,3));
                if all(vecnorm(delta.*[1 1 cfg.yawLeverArm],2,2)>=a.minimumSeedSeparation)
                    seeds(end+1,:)=additionalSeeds(k,:);candidates{end+1}=solve(additionalSeeds(k,:)); %#ok<AGROW>
                end
            end
            count=numel(candidates);scores=inf(count,1);innovation=scores;
            poses=zeros(count,3);similarity=zeros(count,1);valid=false(count,1);
            for k=1:count
                r=candidates{k};poses(k,:)=r.poseXYTheta;similarity(k)=r.similarity;
                delta=r.poseXYTheta(1:2)-aid.position(:).';innovation(k)=delta/C*delta.';
                valid(k)=r.accepted || r.directionalAccepted;
                if valid(k)
                    scores(k)=-2*log(max(r.similarity,realmin))+innovation(k);
                end
            end
            [~,selected]=min(scores);
            result=candidates{selected};
            distinct=false(count,1);representatives=zeros(0,1);[~,order]=sort(scores);
            for k=order(:).'
                if ~valid(k),continue;end
                delta=poses(representatives,:)-poses(k,:);delta(:,3)=wrap(delta(:,3));
                if all(vecnorm(delta.*[1 1 cfg.yawLeverArm],2,2)>=a.hypothesisSeparation)
                    distinct(k)=true;representatives(end+1,1)=k; %#ok<AGROW>
                end
            end
            probabilities=zeros(count,1);eligible=valid & distinct;
            spread=zeros(3);
            if any(eligible)
                probabilities(eligible)=exp(-.5*(scores(eligible)-min(scores(eligible))));
                probabilities=probabilities/sum(probabilities);
                for k=find(eligible).'
                    delta=poses(k,:)-poses(selected,:);delta(3)=wrap(delta(3));
                    spread=spread+probabilities(k)*(delta.'*delta);
                end
                % Preserve genuine null directions and never increase geometry information.
                result.information=reduceInformation(result.information,spread);
                result.directionalInformation=reduceInformation(result.directionalInformation,spread);
            end
            if isfield(result,'lidarResidualModel')
                % The residual exporter must apply this uncertainty to both the
                % innovation and Jacobian, not undo the information reduction.
                result.lidarResidualModel.poseSpread=spread;
                result.lidarResidualModel.conditionedOnPositionAid=true;
            end
            result.positionAiding=struct('used',true,'reason',"hypothesisSelection", ...
                'selected',selected,'candidateCount',count,'seedPoses',seeds(1:count,:), ...
                'candidatePoses',poses,'geometricallyValid',valid,'similarity',similarity, ...
                'scores',scores,'squaredInnovation',innovation,'relativeSupport',probabilities, ...
                'betweenHypothesisSecondMoment',spread,'gnssInformationAdded',false, ...
                'selectionDependsOnGnss',true,'informationSemantics',"conditionalLiDARGeometryWithModeDisagreement");
            if valid(selected) && innovation(selected)>a.maximumSquaredInnovation
                result.accepted=false;result.directionalAccepted=false;
                result.reason="positionAidConflict";result.positionAiding.reason="selectedOptimumConflictsWithPosition";
            end
        end

        function [mu,covariance,count]=softPointAssociationTarget(means,covariances,cost,priorCost,closest,cfg)
        % softPointAssociationTarget Moment-match nearby ambiguous map components.
        % Compatibility costs include priors and optional height penalties. Temper only
        % the geometry,
        % preserve that nongeometric evidence, and restrict support around the original MAP target.
        % Between-component scatter prevents an ambiguous mean becoming a sharp anchor.
        % Inputs are prevalidated by prepareSemanticRegistrationGeometry.
            nearby=sum((means-means(closest,:)).^2,2)<=cfg.radius^2 & isfinite(cost);
            ids=find(nearby);
            value=(cost(ids)-priorCost(ids))/cfg.temperature+priorCost(ids);
            value=value-min(value);
            keep=value<=9;ids=ids(keep);value=value(keep);
            probability=exp(-.5*value);probability=probability/sum(probability);
            [confidence,mode]=max(probability);
            if confidence>=cfg.minimumPosterior && ids(mode)==closest
                mu=means(closest,:);covariance=covariances(:,:,closest);count=1;return;
            end
            mu=sum(means(ids,:).*probability,1);covariance=zeros(2);count=numel(ids);
            for k=1:count
                delta=means(ids(k),:)-mu;
                covariance=covariance+probability(k)*(covariances(:,:,ids(k))+delta.'*delta);
            end
        end

        function validateSoftPointAssociation(cfg)
        % validateSoftPointAssociation Validate local ambiguity and anchor controls.
            fields={'temperature','radius','minimumPosterior','hardWithUnmergedAnchors'};
            valid=isstruct(cfg)&&isscalar(cfg)&&all(isfield(cfg,fields));
            if valid
                values=cellfun(@(f)cfg.(f),fields,UniformOutput=false);
                valid=all(cellfun(@(v)isnumeric(v)&&isscalar(v)&&isreal(v)&&isfinite(v),values));
            end
            if valid
                valid=cfg.temperature>0 && cfg.radius>0 && cfg.minimumPosterior>.5 && cfg.minimumPosterior<=1 ...
                    && cfg.hardWithUnmergedAnchors>=2 && cfg.hardWithUnmergedAnchors==fix(cfg.hardWithUnmergedAnchors);
            end
            assert(valid,'VehicleLocalization:InvalidSoftPointAssociation', ...
                'Use positive temperature/radius, posterior in (0.5,1], and at least two integer anchors.');
        end

        function [tangent,valid,angularScatter]=sourceLineDirections(c,cfg)
        % sourceLineDirections Estimate local unoriented curb axes from selected means.
        % A direction requires multiple spatially separated, nearly collinear means.
        % No reference pose, map geometry, point labels or frame index is used.
            values=[cfg.radius,cfg.minimumComponents,cfg.minimumAnisotropy,cfg.minimumSpan,cfg.standardDeviation];
            assert(isreal(values)&&all(isfinite(values)&values>0) && cfg.minimumComponents>=3 ...
                && cfg.minimumComponents==fix(cfg.minimumComponents) && cfg.minimumAnisotropy>1 ...
                && cfg.standardDeviation<pi/2,'VehicleLocalization:InvalidLineDirectionConfiguration', ...
                'Line-direction neighborhoods and angular scale must be finite and positive.');
            n=c.numComponents;tangent=zeros(n,2);valid=false(n,1);angularScatter=zeros(n,1);
            for name=["curb","facade"]
                eligible=c.semanticName==name & c.mixtureWeight>0;
                if isfield(c,'quality'),eligible=eligible & c.quality>0;end
                ids=find(eligible);centers=c.mean(ids,1:2);p=unique(centers,'rows');
                for k=1:numel(ids)
                    keep=sum((p-centers(k,:)).^2,2)<=cfg.radius^2;q=p(keep,:);
                    if size(q,1)<cfg.minimumComponents,continue;end
                    d=q-mean(q,1);cov=d.'*d/size(d,1);[v,e]=eig(cov,'vector');[major,j]=max(e);
                    axis=v(:,j);along=d*axis;
                    if major<cfg.minimumAnisotropy*max(min(e),1e-6) || (max(along)-min(along))<cfg.minimumSpan,continue;end
                    tangent(ids(k),:)=axis.';valid(ids(k))=true;
                    angularScatter(ids(k))=sqrt(max(0,min(e))/max(major,eps));
                end
            end
        end

        function model=prepareRelativeHeightAssociation(fixedCloud,movingCloud,origin,initialPairs,cfg)
        % prepareRelativeHeightAssociation Infer a common Z datum from curb evidence.
        % Height changes candidate costs only. The retained SE(2) pose residual and
        % information remain planar. Heights are current-scan observations; the XY
        % cloud can still contain five acquisitions. No reference altitude is used.
            values=[cfg.minimumAnchors,cfg.minimumAnchorSpan,cfg.maximumAnchorDistance, ...
                cfg.noiseStandardDeviation,cfg.tiltStandardDeviation,cfg.weight,cfg.maximumPenalty];
            assert(all(isfinite(values))&&all(values>0)&&cfg.minimumAnchors>=2 && ...
                cfg.minimumAnchors==fix(cfg.minimumAnchors),'VehicleLocalization:InvalidHeightConfiguration', ...
                'Relative-height bounds must be positive, with at least two anchors.');
            assert(isscalar(string(cfg.candidateModel)) && ismember(string(cfg.candidateModel),["marginal","conditional"]), ...
                'VehicleLocalization:InvalidHeightConfiguration','Use marginal or conditional height evidence.');
            f=registrationSupport.getHeightEvidence(fixedCloud);
            m=registrationSupport.getHeightEvidence(movingCloud);
            m.semanticName=string(movingCloud.components.semanticName);
            f.mean(:,1:2)=f.mean(:,1:2)-origin(1:2);
            n=numel(f.available);f.slope=zeros(n,2);f.conditionalVariance=zeros(n,1);
            for k=find(f.available).'
                covariance=f.covariance(:,:,k);beta=covariance(1:2,1:2)\covariance(1:2,3);
                f.slope(k,:)=beta.';
                f.conditionalVariance(k)=max(0,covariance(3,3)-covariance(3,1:2)*beta);
            end
            details=struct('enabled',false,'reason',"insufficientHeightAnchors", ...
                'anchorCount',0,'anchorSpan',0,'offset',NaN,'offsetScatter',NaN, ...
                'sourceHeightCount',nnz(m.available),'mapHeightCount',nnz(f.available), ...
                'verticalReference',"mapCurbConsensus",'referenceAltitudeUsed',false, ...
                'planarForce',false,'weight',cfg.weight,'candidateModel',string(cfg.candidateModel));
            model=struct('details',details,'cost',[]);
            s=initialPairs.source;t=initialPairs.target;
            select=initialPairs.semanticName=="curb" & f.available(t) & m.available(s);
            s=s(select);t=t(select);
            r=rotation(origin(3));world=m.mean(s,1:2)*r.';
            delta=world-f.mean(t,1:2);
            % The XY normal residual selects local curb support without demanding
            % agreement along an extended line component's tangent.
            distance=zeros(numel(s),1);
            for k=1:numel(s)
                [v,d]=eig(f.covariance(1:2,1:2,t(k)),'vector');[~,small]=min(d);
                distance(k)=abs(delta(k,:)*v(:,small));
            end
            [~,order]=sort(distance);s=s(order);t=t(order);world=world(order,:);distance=distance(order);
            [~,uniqueTarget]=unique(t,'stable');
            use=uniqueTarget(distance(uniqueTarget)<=cfg.maximumAnchorDistance);
            s=s(use);t=t(use);world=world(use,:);
            dz=f.mean(t,3)+sum((world-f.mean(t,1:2)).*f.slope(t,:),2)-m.mean(s,3);
            if numel(dz)<cfg.minimumAnchors,return;end
            center=median(dz);scatter=1.4826*median(abs(dz-center));
            inlier=abs(dz-center)<=3*max(scatter,cfg.noiseStandardDeviation);
            dz=dz(inlier);world=world(inlier,:);
            details.anchorCount=numel(dz);
            if isempty(dz),model.details=details;return;end
            details.anchorSpan=norm(max(world,[],1)-min(world,[],1));
            details.offset=median(dz);
            % Scatter includes common anchor/model disagreement and is not divided
            % by the number of correlated curb observations.
            details.offsetScatter=1.4826*median(abs(dz-details.offset));
            details.enabled=numel(dz)>=cfg.minimumAnchors && details.anchorSpan>=cfg.minimumAnchorSpan;
            if details.enabled,details.reason="enabled";end
            model.details=details;
            if details.enabled
                model.cost=@(source,target,pose) candidateCost(f,m,source,target,pose,details,cfg);
            end
        end

        function [residual,J,precision,shapeUsed]=gaussianRegistrationResiduals(sourceMean,sourceCov,targetMean,targetCov,pose,noiseStd)
        % gaussianRegistrationResiduals Normalized Gaussian overlap in planar SE(2).
        % Squared residual = delta'/(A+B)*delta + log(det(A+B)/(4*sqrt(det(A)*det(B)))).
        % A and B each receive half the isotropic noise variance. Both covariance
        % axes contribute continuously; isotropic shapes provide no rotation factor.
        % J differentiates the rotated covariance as well as the transformed center.
            n=size(sourceMean,1);r=[cos(pose(3)) -sin(pose(3));sin(pose(3)) cos(pose(3))];
            rotated=pagemtimes(pagemtimes(r,sourceCov),r.');
            ax=reshape(rotated(1,1,:),[],1);bx=reshape(rotated(1,2,:),[],1);dx=reshape(rotated(2,2,:),[],1);
            at=reshape(targetCov(1,1,:),[],1);bt=reshape(targetCov(1,2,:),[],1);dt=reshape(targetCov(2,2,:),[],1);
            noise=noiseStd^2/2;
            a=ax+at+2*noise;b=bx+bt;d=dx+dt+2*noise;detS=a.*d-b.^2;
            delta=sourceMean*r.'+pose(1:2)-targetMean;
            l11=sqrt(a);l21=b./l11;l22=sqrt(d-l21.^2);
            rx=delta(:,1)./l11;ry=(delta(:,2)-l21.*rx)./l22;
            detA=(ax+noise).*(dx+noise)-bx.^2;detB=(at+noise).*(dt+noise)-bt.^2;
            shape=max(0,log(detS)-log(4)-.5*(log(detA)+log(detB)));
            angle=.5*(atan2(2*bx,ax-dx)-atan2(2*bt,at-dt));
            gapA=hypot(ax-dx,2*bx);gapB=hypot(at-dt,2*bt);
            traceSum=a+d;alignedDet=(traceSum+gapA+gapB).*(traceSum-gapA-gapB)/4;
            kappa=gapA.*gapB./alignedDet;u=kappa.*sin(angle).^2;orientationCost=log1p(u);
            % Separate pose-invariant scale mismatch from smooth angular residuals.
            % A single sqrt(total shape cost) loses Gauss-Newton curvature whenever
            % unequal cloud scales leave a nonzero residual at aligned axes.
            ratio=ones(n,1);active=u>1e-12;ratio(active)=orientationCost(active)./u(active);
            orientation=sin(angle).*sqrt(kappa.*ratio);
            residual=[rx,ry,orientation,sqrt(max(0,shape-orientationCost))].';
            precision=zeros(2,2,n);precision(1,1,:)=1./l11;
            precision(2,1,:)=-l21./l11./l22;precision(2,2,:)=1./l22;
            J=zeros(4,3,n);J(1:2,1:2,:)=precision;
            meanYaw=sourceMean*[r(:,2),-r(:,1)].';
            ap=-2*bx;bp=ax-dx;dp=2*bx;
            l11p=ap./(2*l11);l21p=bp./l11-l21.*l11p./l11;
            l22p=(dp-2*l21.*l21p)./(2*l22);
            rxp=meanYaw(:,1)./l11-rx.*l11p./l11;
            ryp=(meanYaw(:,2)-l21p.*rx-l21.*rxp)./l22-ry.*l22p./l22;
            J(1,3,:)=rxp;J(2,3,:)=ryp;
            shapeYaw=sqrt(kappa).*cos(angle);active=orientationCost>1e-12;
            shapeYaw(active)=sign(sin(angle(active))).*kappa(active).*sin(angle(active)).*cos(angle(active))./ ...
                ((1+u(active)).*sqrt(orientationCost(active)));
            J(3,3,:)=shapeYaw;
            shapeUsed=gapA.*gapB>0;
        end

        function [residual,J,precision]=supportRegistrationResiduals(sourceMean,sourceCov,targetMean,targetCov,sourceAxis,targetNormal,directionScale,pose,noise)
        % supportRegistrationResiduals Partial-support position and shape orientation.
        % The latent sliding covariance is frozen in the map frame; source scatter
        % rotates analytically. Angular information vanishes for round distributions.
            [residual,J,precision]=registrationSupport.gaussianRegistrationResiduals(sourceMean,sourceCov,targetMean,targetCov,pose,noise);
            residual=residual(1:3,:);J=J(1:3,:,:);
            r=[cos(pose(3)) -sin(pose(3));sin(pose(3)) cos(pose(3))];
            residual(3,:)=(sum((sourceAxis*r.').*targetNormal,2).*directionScale).';
            J(3,:,:)=0;
            J(3,3,:)=sum((sourceAxis*[r(:,2),-r(:,1)].').*targetNormal,2).*directionScale;
        end
    end
end

function available=hasHeight(components)
    available=size(components.mean,2)==3 || ...
        (isfield(components,'heightAvailable') && all(components.heightAvailable));
end

function [energy, gradient] = spatialGaussianOverlap(fixed, moving, pose)
% spatialGaussianOverlap: Exact 3D Gaussian overlap and planar-pose gradient.
% Z origins are already aligned. Covariance derivatives include xz/yz terms.
    c=cos(pose(3)); s=sin(pose(3));
    rotation=[c -s 0;s c 0;0 0 1];
    means=moving.mean*rotation.'+[pose(1:2) 0];
    meanDerivative=moving.mean*[-s -c 0;c -s 0;0 0 0].';
    a0=reshape(moving.covariance(1,1,:),[],1);
    b0=reshape(moving.covariance(1,2,:),[],1);
    d0=reshape(moving.covariance(2,2,:),[],1);
    c0=reshape(moving.covariance(1,3,:),[],1);
    e0=reshape(moving.covariance(2,3,:),[],1);
    ma=c*c*a0-2*c*s*b0+s*s*d0;
    mb=c*s*(a0-d0)+(c*c-s*s)*b0;
    md=s*s*a0+2*c*s*b0+c*c*d0;
    mc=c*c0-s*e0; me=s*c0+c*e0;
    mf=reshape(moving.covariance(3,3,:),[],1);
    energy=0; gradient=zeros(1,3);
    fixedActive=fixed.mixtureWeight>0;
    movingActive=moving.mixtureWeight>0;
    names=intersect(unique(fixed.semanticName(fixedActive)),unique(moving.semanticName(movingActive)));
    for name=names.'
        f=find(fixed.semanticName==name & fixedActive);
        allMoving=find(moving.semanticName==name & movingActive);
        fa=reshape(fixed.covariance(1,1,f),[],1); fb=reshape(fixed.covariance(1,2,f),[],1);
        fc=reshape(fixed.covariance(1,3,f),[],1); fd=reshape(fixed.covariance(2,2,f),[],1);
        fe=reshape(fixed.covariance(2,3,f),[],1); ff=reshape(fixed.covariance(3,3,f),[],1);
        blockSize=max(1,floor(100000/max(numel(f),1)));
        for start=1:blockSize:numel(allMoving)
            m=allMoving(start:min(start+blockSize-1,end));
            a=fa+ma(m).'; b=fb+mb(m).'; cc=fc+mc(m).';
            d=fd+md(m).'; e=fe+me(m).'; h=ff+mf(m).';
            aa=d.*h-e.^2; ab=cc.*e-b.*h; ac=b.*e-cc.*d;
            ad=a.*h-cc.^2; ae=b.*cc-a.*e; af=a.*d-b.^2;
            determinant=a.*aa+b.*ab+cc.*ac;
            assert(all(determinant>0,'all'),'VehicleLocalization:InvalidCovariance','Invalid summed XYZ covariance.');
            dx=fixed.mean(f,1)-means(m,1).';
            dy=fixed.mean(f,2)-means(m,2).';
            dz=fixed.mean(f,3)-means(m,3).';
            qx=(aa.*dx+ab.*dy+ac.*dz)./determinant;
            qy=(ab.*dx+ad.*dy+ae.*dz)./determinant;
            qz=(ac.*dx+ae.*dy+af.*dz)./determinant;
            kernel=exp(-0.5*(dx.*qx+dy.*qy+dz.*qz))./((2*pi)^1.5*sqrt(determinant));
            weighted=kernel.*(fixed.mixtureWeight(f)*moving.mixtureWeight(m).');
            energy=energy+sum(weighted,'all');
            if nargout > 1
                da=-2*mb(m).'; db=(ma(m)-md(m)).'; dc=-me(m).';
                dd=2*mb(m).'; de=mc(m).';
                rotationTerm=qx.*meanDerivative(m,1).'+qy.*meanDerivative(m,2).'+ ...
                    0.5*(qx.^2.*da+2*qx.*qy.*db+2*qx.*qz.*dc+qy.^2.*dd+2*qy.*qz.*de- ...
                    (aa.*da+2*ab.*db+2*ac.*dc+ad.*dd+2*ae.*de)./determinant);
                gradient=gradient+[sum(weighted.*qx,'all'),sum(weighted.*qy,'all'),sum(weighted.*rotationTerm,'all')];
            end
        end
    end
end

function S=momentMatch(covariance,means,mu,w)
% Weighted within-plus-between scatter about mu over the leading dimensions.
    d=size(covariance,1);S=zeros(d);
    for k=1:numel(w)
        delta=(means(k,1:d)-mu(1:d)).';S=S+w(k)*(covariance(:,:,k)+delta*delta.');
    end
    S=S/sum(w);S=(S+S.')/2;
end

function information=reduceInformation(information,spread)
    I=(information+information.')/2;
    [V,D]=eig(I);root=V*diag(sqrt(max(0,diag(D))))*V.';
    information=root*((eye(3)+root*spread*root)\root);
    information=(information+information.')/2;
end

function x=wrap(x)
    x=atan2(sin(x),cos(x));
end

function [cost,residual]=candidateCost(f,m,source,target,pose,details,cfg)
    r=rotation(pose(3));mean=m.mean(source,1:2)*r.'+pose(1:2);
    beta=f.slope(target,:);mapVariance=f.conditionalVariance(target);
    if string(cfg.candidateModel)=="marginal"
        % A narrow vertical structure does not define a reliable Z surface
        % over XY. Do not extrapolate its height from a small lateral offset.
        beta(:)=0;mapVariance=reshape(f.covariance(3,3,target),[],1);
    end
    dx=mean(:,1).'-f.mean(target,1);dy=mean(:,2).'-f.mean(target,2);
    residual=f.mean(target,3)+beta(:,1).*dx+beta(:,2).*dy-m.mean(source,3).'-details.offset;
    r3=blkdiag(r,1);c=pagemtimes(pagemtimes(r3,m.covariance(:,:,source)),r3.');
    xx=reshape(c(1,1,:),1,[]);xy=reshape(c(1,2,:),1,[]);yy=reshape(c(2,2,:),1,[]);
    xz=reshape(c(1,3,:),1,[]);yz=reshape(c(2,3,:),1,[]);zz=reshape(c(3,3,:),1,[]);
    variance=mapVariance+zz+beta(:,1).^2.*xx+beta(:,2).^2.*yy ...
        +2*beta(:,1).*beta(:,2).*xy-2*beta(:,1).*xz-2*beta(:,2).*yz ...
        +cfg.noiseStandardDeviation^2+details.offsetScatter^2 ...
        +cfg.tiltStandardDeviation^2*sum(m.mean(source,1:2).^2,2).';
    q=residual.^2./max(variance,eps);
    cost=cfg.weight*cfg.maximumPenalty*q./(cfg.maximumPenalty+q);
    available=f.available(target)&m.available(source).' & ismember(m.semanticName(source),cfg.semanticNames).';
    cost(~available)=0;residual(~available)=NaN;
end

function r=rotation(yaw)
    r=[cos(yaw) -sin(yaw);sin(yaw) cos(yaw)];
end
