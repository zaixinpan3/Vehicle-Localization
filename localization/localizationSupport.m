classdef localizationSupport
% localizationSupport Shared implementation for the current localization pipeline.
% Static methods are grouped by responsibility below. File-local helpers use
% group-specific names to preserve the original lexical scopes.
% The public runtime entry remains runFullLocalizationObserver.
    methods (Static)

        %% Alignment and measurement support
        function [design,cfg]=designFullObserverGains(cfg)
        % Solve missing temporary gains, then verify before discretization.
            arguments
                cfg (1,1) struct=mncavFullObserverConfig()
            end
            assert(isfield(cfg,'invariantObserver') ...
                && ~isfield(cfg,'hgo') && ~isfield(cfg,'thetaFramework'), ...
                'VehicleLocalization:RemovedFullObserverConfig', ...
                'Only the intrinsic eight-state observer is supported; regenerate old replay configurations.');
            assert(~isfield(cfg,'timing') || string(cfg.timing)=="synchronous", ...
                'VehicleLocalization:ObsoleteFullConfig','Only the common frame clock is supported.');
            assert(isfield(cfg,'iss'),'VehicleLocalization:MissingFullIssDomain','Declare the current continuous design domain.');
            validateattributes(cfg.gnss.gainInformationScale,{'numeric'},{'real','finite','positive','scalar'});
            if ~isfield(cfg,'invariantDesign')
                [~,cfg]=synthesizeMncavInvariantDesign(cfg);
            end
            design=localizationSupport.runtime(cfg);
        end

        function [aligned,lateral,metadata]=synchronizeLocalizationInputs(data,lateralInput,cfg)
        % synchronizeLocalizationInputs Align existing observations on LiDAR frames.
        % Offline bracket interpolation uses real endpoints only, without motion
        % extrapolation. Invalid endpoints or excessive gaps withdraw that channel.
        % The future-endpoint wait is reported separately from perception delay.
            arguments
                data (1,1) struct
                lateralInput (1,1) struct
                cfg (1,1) struct=mncavFullObserverConfig()
            end
            h=data.highRate;overlapChannel=isfield(data,'lidarOverlap');
            if overlapChannel
                assert(~isfield(data,'lidar'),'VehicleLocalization:AmbiguousLidarInput', ...
                    'Supply one LiDAR channel when synchronizing inputs.');
                t=data.lidarOverlap.time(:);
            else
                t=data.lidar.time(:);
            end
            assert(all(diff(t)>0) && all(isfinite(t)),'VehicleLocalization:InvalidSyncClock','Invalid LiDAR clock.');
            selected=t>=h.time(1) & t<=h.time(end);t=t(selected);
            assert(numel(t)>=2,'VehicleLocalization:SyncCoverage','Need two covered localization frames.');
            [left,right,fraction,covered]=fullBrackets(h.time,t,cfg.synchronization.motionMaximumBracket);
            assert(all(covered),'VehicleLocalization:SyncCoverage','Motion does not cover the frame clock.');
            aligned=struct('highRate',struct('time',t));
            for name=string(fieldnames(h)).'
                if name=="time",continue;end
                if any(name==["bodyVelocityValid","bodyAccelerationValid","yawRateValid"])
                    aligned.highRate.(name)=logical(h.(name)(left)) & logical(h.(name)(right));
                else
                    aligned.highRate.(name)=fullBlend(h.(name),left,right,fraction);
                end
            end
            assert(isequal(lateralInput.time,h.time),'VehicleLocalization:SyncCoverage','Lateral and motion clocks differ.');
            lateral=struct('time',t);
            assert(all(isfield(lateralInput,{'lateralVelocity','sideSlipAngle','sideSlipAngleRate'})), ...
                'VehicleLocalization:AlignedLateralRequired','Supply the upstream lateral angle/rate pair before synchronization.');
            for name=["lateralVelocity","sideSlipAngle","sideSlipAngleRate"]
                lateral.(name)=fullBlend(lateralInput.(name),left,right,fraction);
            end
            if isfield(lateralInput,'velocityValid')
                valid=lateralInput.velocityValid(:);
                assert(numel(valid)==numel(h.time) && all(ismember(valid,[0,1])), ...
                    'VehicleLocalization:InvalidMotionValidity','Invalid lateral velocity validity mask.');
                lateral.velocityValid=logical(valid(left)) & logical(valid(right));
            end
            if overlapChannel
                aligned.lidarOverlap=data.lidarOverlap;aligned.lidarOverlap.time=t;
                aligned.lidarOverlap.sources=data.lidarOverlap.sources(selected);
                if isfield(data.lidarOverlap,'valid'),aligned.lidarOverlap.valid=data.lidarOverlap.valid(selected);end
                for name=["frameReliability","directionReliability"]
                    if isfield(data.lidarOverlap,name)
                        aligned.lidarOverlap.(name)=data.lidarOverlap.(name)(selected,:);
                    end
                end
            else
                aligned.lidar=data.lidar;aligned.lidar.time=t;
                aligned.lidar.pose=data.lidar.pose(selected,:);
                aligned.lidar.valid=data.lidar.valid(selected);
                aligned.lidar.information=data.lidar.information(:,:,selected);
                for name=["frameReliability","directionReliability","poseErrorVariance"]
                    if isfield(data.lidar,name),aligned.lidar.(name)=data.lidar.(name)(selected,:);end
                end
                if isfield(data.lidar,'evaluateResidual')
                    originalIndex=find(selected);provider=data.lidar.evaluateResidual;
                    aligned.lidar.evaluateResidual=@(k,pose)provider(originalIndex(k),pose);
                end
                if isfield(data.lidar,'residualModels'),aligned.lidar.residualModels=data.lidar.residualModels(selected);end
            end
            motionWait=h.time(right)-t;gnssWait=zeros(size(t));gnssBracket=zeros(size(t));
            if isfield(data,'gnss')
                g=data.gnss;
                [a,b,w,ok]=fullBrackets(g.time,t,cfg.synchronization.gnssMaximumBracket);
                ok=ok & g.valid(a) & g.valid(b);
                position=nan(numel(t),2);information=nan(2,2,numel(t));
                for k=find(ok).'
                    position(k,:)=(1-w(k))*g.position(a(k),:)+w(k)*g.position(b(k),:);
                    information(:,:,k)=(1-w(k))*g.information(:,:,a(k))+w(k)*g.information(:,:,b(k));
                end
                gnssWait(ok)=g.time(b(ok))-t(ok);gnssBracket(ok)=g.time(b(ok))-g.time(a(ok));
                aligned.gnss=struct('time',t,'position',position,'information',information,'valid',ok,'delay',0, ...
                    'sourceLeftTime',g.time(a),'sourceRightTime',g.time(b));
            end
            metadata=struct('clock',"Native LiDAR frames; no synthetic inter-frame localization outputs", ...
                'nominalRateHz',1/median(diff(t)),'frames',numel(t), ...
                'motionModelMeasurementExtrapolation',false,'offlineBracketInterpolation',true, ...
                'maximumGnssBracketSeconds',max(gnssBracket),'maximumGnssWaitSeconds',max(gnssWait), ...
                'maximumMotionWaitSeconds',max(motionWait),'frameWaitSeconds',max(motionWait,gnssWait), ...
                'inputScope',"Wheel/lateral estimates retain their native preprocessing; only aligned frame values enter localization");
        end

        function support=lidarPoseSupport(filter,cfg)
        % lidarPoseSupport Local normalized pose support of admitted LiDAR alone.
        % Respect the raw filter's admitted rank before inspecting weighted strength.
        % A positive yaw diagonal can constrain only a position/yaw combination.
        % Project a PSD factor's yaw column off its translation column space to
        % measure heading after free translation, without Schur subtraction.
        % Normalize nonzero translation columns before SVD: even a tiny column
        % may absorb yaw when translation is free. These are local geometric
        % diagnostics; neither source freshness nor correct alignment is tested.
        % Strengths above 1e-10 count as numerical support. sectorQualified also
        % requires the configured position, heading and coupling bounds in cfg.iss.
            arguments
                filter (1,1) struct
                cfg (1,1) struct
            end
            U=filter.eigenvectors(:,4-filter.rank:3);
            S=filter.S;weighted=U.'*S*U;
            [V,E]=eig((weighted+weighted.')/2);
            strengths=max(diag(E),0);threshold=1e-10;
            active=strengths>threshold;dimension=nnz(active);
            B=diag(sqrt(strengths(active)))*V(:,active).'*U.';
            admitted=B.'*B;
            minimumStrength=0;
            if dimension==3,minimumStrength=min(strengths);end
            position=B(:,1:2);heading=B(:,3);
            columnNorm=vecnorm(position,2,1);nonzero=columnNorm>0;
            if any(nonzero)
                position=position(:,nonzero)./columnNorm(nonzero);
                [Q,values,~]=svd(position,'econ','vector');
                Q=Q(:,values>max(size(position))*eps(max(values)));
                heading=heading-Q*(Q.'*heading);
            end
            headingStrength=norm(heading)^2;
            support=struct('rank',dimension,'minimumStrength',minimumStrength, ...
                'fullPoseSupported',dimension==3,'headingStrengthAfterPosition',headingStrength, ...
                'headingSupported',headingStrength>threshold, ...
                'sectorQualified',dimension==3 ...
                    && min(eig(admitted(1:2,1:2)))>=cfg.iss.minimumPositionStrength ...
                    && admitted(3,3)>=cfg.iss.minimumHeadingStrength ...
                    && norm(admitted(1:2,3))<=cfg.iss.maximumPositionHeadingCoupling);
        end

        function [position,information]=correctGnssOutputPoint(position,information,yaw,alignment)
        % correctGnssOutputPoint Express a receiver position at the observer point.
        % bodyOffset is receiver minus observer point, in forward/left coordinates.
        % An empirical output-point calibration is not a surveyed installation lever.
        % Its covariance and declared heading uncertainty are conservative additions;
        % unreported correlations with the receiver solution remain unknown.
            offset=alignment.bodyOffset(:);C=alignment.bodyCovariance;
            assert(numel(offset)==2 && all(isfinite(offset)) && isequal(size(C),[2,2]) && ...
                all(isfinite(C),'all') && norm(C-C.','fro')<1e-12 && min(eig(C))>=0 && ...
                isscalar(alignment.headingStdRad) && isfinite(alignment.headingStdRad) && alignment.headingStdRad>=0, ...
                'VehicleLocalization:InvalidGnssAlignment','Invalid output-point calibration.');
            if all(offset==0) && all(C==0,'all'),return;end
            R=[cos(yaw),-sin(yaw);sin(yaw),cos(yaw)];J=[0,-1;1,0];
            position=position-(R*offset).';
            covariance=information\eye(2)+R*C*R.'+alignment.headingStdRad^2*(R*J*offset)*(R*J*offset).';
            information=covariance\eye(2);information=(information+information.')/2;
        end

        %% Intrinsic observer and gain synthesis
        function [next,q]=predict(state,dt,sideslipRate)
            q=state(8)+sideslipRate;
            A=zeros(8);A(1,2)=1;A(2,3)=1;A(4,5)=1;A(5,6)=1;A(7,8)=1;
            A([3,6],[2,5])=q^2*eye(2);A([3,6],[3,6])=2*q*[0,-1;1,0];
            next=(eye(8)-dt*A)\state;
        end

        function [value,H]=outputs(state,beta,cfg)
            v=state([2,5]);a=state([3,6]);
            cross=v(1)*a(2)-v(2)*a(1);angle=state(7)+beta;
            direction=[-sin(angle);cos(angle)];
            % Physical outputs; numerical scales belong to the feedback metric.
            value=[v.'*v;v.'*a;cross;direction.'*v;state(8)];
            if nargout>1
                [~,H]=localizationSupport.secants(state,state,beta,0,cfg);
            end
        end

        function D=outputNormalization(cfg,theta)
        % Fixed, invertible numerical scaling; never divides by actual speed.
            if nargin<2,theta=cfg.invariantObserver.theta;end
            V=cfg.iss.maximumSpeed;ell=cfg.invariantObserver.headingLengthM;
            D=diag([1/(2*V),1/V,1/V,1,ell].*theta.^-[2,3,3,2,2]);
        end

        function packet=measurements(speed,vy,ax,ay,gyro,beta,velocityValid,accelerationValid,gyroValid,cfg)
        % Five outputs only. Sideslip rate belongs to the predictor, not this packet.
            velocityValid=velocityValid && cfg.invariantObserver.velocityAvailable;
            accelerationValid=accelerationValid && cfg.invariantObserver.accelerationAvailable;
            value=zeros(5,1);valid=[velocityValid;velocityValid&&accelerationValid; ...
                velocityValid&&accelerationValid;velocityValid;gyroValid];
            if valid(1),value(1)=speed^2+vy^2;end
            if valid(2),value(2)=speed*ax+vy*ay;value(3)=speed*ay-vy*ax;end
            if gyroValid,value(5)=gyro;end
            if ~velocityValid,beta=0;end
            packet=struct('value',value,'valid',valid,'beta',beta);
        end

        function [next,audit]=correct(state,packet,dt,design,cfg)
            [value,H]=localizationSupport.outputs(state,packet.beta,cfg);
            D=localizationSupport.outputNormalization(cfg,design.theta);
            C=D*H*design.toPhysical;
            weights=localizationSupport.weights(design,packet.value(1));
            k=[weights([1,2,2,3]);0].*double(packet.valid);
            residual=D*(packet.value-value);weighted=C.'*diag(k);
            Q=design.theta*weighted*C;rhs=dt*design.theta*(design.P\(weighted*residual));
            delta=(eye(8)+dt*(design.P\Q))\rhs;next=state+design.toPhysical*delta;
            if packet.valid(5)
                % The gyro innovation acts on its estimated rate state only.
                % Its own scalar LMI supplies the gain; heading is untouched.
                measuredRate=packet.value(5);
                gain=design.theta*design.gyroGainNormalized;
                next(8)=(next(8)+dt*gain*measuredRate)/(1+dt*gain);
            end
            audit=struct('innovation',packet.value-value,'valid',packet.valid, ...
                'allAuxiliaryOutputsAvailable',all(packet.valid), ...
                'predicted',value,'normalizedCorrection',delta,'auxiliaryWeights',weights, ...
                'frozenInvariantMetricNonexpansive',min(eig((Q+Q.')/2))>=-1e-10*max(1,norm(Q,2)));
        end

        function inputs=invariantInputs(cfg)
        % Version every mathematical design input; reject retired designs.
            z=cfg.invariantObserver;
            assert(~isfield(cfg,'gainSelection') && ~any(isfield(z, ...
                {'redundancy','regionalRadiusCap','maximumMotionPositionNoiseRatio'})), ...
                'VehicleLocalization:ObsoleteFusionDesign', ...
                'Regenerate the nominal fusion configuration; legacy searches and certificates are removed.');
            if isfield(z,'provenance'),z=rmfield(z,'provenance');end
            inputs=jsondecode(jsonencode(struct('architecture',"intrinsic-nominal-polytope-v11", ...
                'physicalOutputs',"squared-dot-cross-direction-rate", ...
                'observer',z,'iss',cfg.iss,'courseRateBound',cfg.maximumTrackAngleRate, ...
                'gnssBodyOffset',cfg.gnss.outputPoint.bodyOffset(:).','poseScales',cfg.lidar.poseScales(:).', ...
                'informationScale',cfg.lidar.gainInformationScale)));
        end

        function design=synthesize(cfg)
        % One SDP: nominal fusion, motion response, gain caps and noise cost.
            inputs=localizationSupport.invariantInputs(cfg);
            assert(exist('sdpvar','file')==2,'VehicleLocalization:MissingSolver','Add YALMIP and SeDuMi before synthesis.');
            m=invariantModel(cfg);z=cfg.invariantObserver;
            assert(m.motionAvailable,'VehicleLocalization:MotionOutputsRequired', ...
                'The nominal fusion design requires all five auxiliary outputs.');
            yalmip('clear');p=sdpvar(1);R=sdpvar(2,2);Ph=sdpvar(2,2);
            % A fixed coordinate shear makes the motion Schur complement R
            % affine while retaining a full motion metric. The acceleration
            % term in the shear reduces pose-driven acceleration at weak LiDAR.
            U=[1,-1,1;0,1,0;0,0,1];Pc=U.'*blkdiag(p,R)*U;
            P=blkdiag(Pc,Pc,Ph);kL=sdpvar(1);kG=kL;kI=sdpvar(3,1);
            constraints=[P>=eye(8),kL>=z.margin,kI>=z.margin];
            if z.angularMetricDiagonal,constraints=[constraints,Ph(1,2)==0];end
            if z.balancedPoseMetric,constraints=[constraints,Ph<=Pc(1:2,1:2)];end
            if isfield(z,'headingGainLimits')
                constraints=[constraints,P-kL/z.headingGainLimits(1)*(m.e7*m.e7.')>=0, ...
                    P-kI(3)/z.headingGainLimits(2)*(m.e7*m.e7.')>=0];
            end
            if isfield(z,'matchedGainLimits')
                limits=z.matchedGainLimits;
                constraints=[constraints,P-kL/limits(1)*(m.e1*m.e1.')>=0, ...
                    P-kG/limits(2)*(m.e1*m.e1.')>=0, ...
                    P-kI(3)/limits(3)*(m.e5*m.e5.')>=0];
            end
            candidate=struct('P',P,'kappaL',kL,'kappaG',kG,'auxiliaryWeights',kI, ...
                'gyroGainNormalized',m.gyroGainNormalized);
            fusion=localizationSupport.fusionBlocks(candidate,cfg);
            response=localizationSupport.responseBlocks(R,kI,cfg);
            for j=1:numel(fusion)
                constraints=[constraints,fusion{j}<=-z.margin*eye(8)]; %#ok<AGROW>
            end
            if z.motionResponse.enabled
                headingFloor=cfg.iss.minimumHeadingStrength-cfg.iss.maximumPositionHeadingCoupling-z.fusion.lidarOperatorMismatch;
                positionFloor=cfg.iss.minimumPositionStrength-cfg.iss.maximumPositionHeadingCoupling-z.fusion.lidarOperatorMismatch;
                nominalSpeed=(z.fusion.minimumCourseSpeed+cfg.iss.maximumSpeed)/2;
                directionSensitivity=(nominalSpeed/(z.theta*z.headingLengthM))^2;
                constraints=[constraints,kL*positionFloor>=Pc(1,1)/(z.theta*z.motionResponse.timeConstantS), ...
                    kL*headingFloor>=Ph(1,1)/(z.theta*z.motionResponse.timeConstantS), ...
                    kI(3)*directionSensitivity>=Ph(1,1)/(z.theta*z.motionResponse.timeConstantS)];
                for j=1:numel(response)
                    constraints=[constraints,response{j}<=-z.margin*eye(2)]; %#ok<AGROW>
                end
            end
            PG=[-kL*m.C.'*m.S*m.DL,-kG*z.nominalGnssWeight*m.Cp.'*m.DG];
            [cost,blocks]=invariantNoiseEnergy(PG,P);constraints=[constraints,blocks];
            for motion=m.motion
                weights=kI;weights(1:2)=motion.scheduleFactor*kI(1:2);
                PG=-z.sigma*motion.H.'*diag([weights([1,2,2,3]);0])*motion.DI;
                PG(:,5)=PG(:,5)-P*m.e8*(m.gyroGainNormalized*z.headingLengthM*z.noise.gyroRadps/z.sigma);
                [bound,blocks]=invariantNoiseEnergy(PG,P);
                constraints=[constraints,blocks];cost=cost+bound/numel(m.motion); %#ok<AGROW>
            end
            [bound,blocks]=invariantNoiseEnergy(P*m.B*m.Dd,P);
            constraints=[constraints,blocks];cost=cost+bound;
            options=sdpsettings('solver',char(z.solver),'verbose',0);
            if z.solver=="sedumi",options.sedumi.stepdif=2;end
            result=optimize(constraints,cost,options);
            residual=min(check(constraints));
            assert(ismember(result.problem,[0,4]) && isfinite(residual) && residual>=-1e-6, ...
                'VehicleLocalization:InfeasibleInvariantDesign','Nominal fusion SDP failed: %s (residual %.9g).',result.info,residual);
            % Homogeneous scaling preserves feedback ratios and creates a
            % visible margin for the independently checked lambda-I condition.
            Pv=1.01*value(P);Pv=(Pv+Pv.')/2;
            design=struct('architecture',"intrinsic-nominal-polytope-v11",'sigma',z.sigma,'P',Pv, ...
                'kappaL',1.01*value(kL),'kappaG',1.01*value(kG),'auxiliaryWeights',1.01*value(kI), ...
                'gyroGainNormalized',m.gyroGainNormalized,'gyroDesign',m.gyroDesign, ...
                'lambda',z.lambda,'meanSquareBound',1.01*value(cost),'solverStatus',result.problem, ...
                'solverInfo',string(result.info),'inputs',inputs, ...
                'motionGainSchedule',struct('kind',"bounded-fusion-speed-v2", ...
                    'minimumSpeedMps',z.fusion.minimumCourseSpeed,'maximumSpeedMps',cfg.iss.maximumSpeed), ...
                'solveAudit',struct('optimizationCalls',1,'fusionVertices',numel(fusion), ...
                    'responseVertices',numel(response)*double(z.motionResponse.enabled), ...
                    'minimumSdpResidual',residual,'referenceUsed',false));
            design.verification=localizationSupport.verifyInvariantDesign(design,cfg);
            assert(design.verification.certified,'VehicleLocalization:InfeasibleInvariantDesign', ...
                'Independent nominal fusion/response verification failed (worst fusion %.9g).', ...
                design.verification.worstVertexEigenvalue);
        end

        function gyro=scalarGyroDesign(cfg)
        % Closed-form minimizer of the scalar gyro noise LMI; no extra solve.
            gyro=invariantGyroDesign(cfg.invariantObserver);
        end

        function v=verifyInvariantDesign(design,cfg)
        % Recompute every vertex numerically; stored solver claims are ignored.
            validateattributes(design.P,{'double'},{'real','finite','size',[8,8]});
            validateattributes(design.auxiliaryWeights,{'double'},{'real','finite','column','numel',3});
            m=invariantModel(cfg,design.gyroDesign);z=cfg.invariantObserver;
            P=(design.P+design.P.')/2;
            structure=norm(P(1:3,1:3)-P(4:6,4:6),'fro')+norm(P(1:6,7:8),'fro') ...
                +norm(P(1:3,4:6),'fro')+abs(P(1,2)+P(1,1)) ...
                +abs(P(1,3)-P(1,1));
            balanced=~z.balancedPoseMetric || max(eig(P(7:8,7:8)-P(1:2,1:2)))<=1e-8*max(1,norm(P,'fro'));
            diagonal=~z.angularMetricDiagonal || abs(P(7,8))<=1e-8*max(1,norm(P,'fro'));
            blocks=localizationSupport.fusionBlocks(design,cfg);worst=sensorWorstEigenvalue(blocks);
            authority=invariantFeedbackAuthority(design,cfg);
            response=localizationSupport.verifyMotionResponse(design,cfg);
            valid=worst<0 && min(eig(P))>=1-1e-6 && all(isfinite([design.kappaL,design.kappaG])) ...
                && design.kappaL>0 && design.kappaG>0 && all(design.auxiliaryWeights>0) ...
                && abs(design.kappaL-design.kappaG)<=1e-8*max(1,design.kappaL) ...
                && authority.verified && m.motionAvailable && response.verified && response.scheduleVerified ...
                && design.sigma==z.sigma && design.lambda==z.lambda ...
                && norm(design.P-design.P.','fro')<=1e-10*max(1,norm(P,'fro')) ...
                && balanced && diagonal && structure<1e-8*max(1,norm(P,'fro')) ...
                && abs(design.gyroGainNormalized-m.gyroGainNormalized)<1e-8*max(1,m.gyroGainNormalized);
            scope="Nominal continuous multi-source fusion on the declared positive-speed domain, with all auxiliary outputs, informative LiDAR and nominal isotropic GNSS information. Nonlinear local use assumes a compact invariant operating set and sufficiently small errors/disturbances (Bessafa Remark 3). Invariance, a numerical attraction radius, outages, actual overlap maps, the lateral cascade and sampling are not certified.";
            v=struct('certified',valid,'nominalFusionLmiVerified',worst<0, ...
                'worstVertexEigenvalue',worst,'vertices',numel(blocks), ...
                'minimumEigenvalueP',min(eig(P)),'maximumEigenvalueP',max(eig(P)), ...
                'intrinsicRateState',true,'zeroSpeedDivision',false,'zeroSpeedCertified',false, ...
                'actualOverlapCertified',false,'finiteInvariantSecantsCertified',false, ...
                'conditionalInvariantLocalClosureEstablished',false,'numericalInvariantRadiusCertified',false, ...
                'operatingSet',struct('assumedInvariant',true,'invarianceVerified',false, ...
                    'speedInterval',[z.fusion.minimumCourseSpeed,cfg.iss.maximumSpeed], ...
                    'nonlinearConclusionConditional',true), ...
                'nominalFusion',struct('certified',worst<0,'vertices',numel(blocks), ...
                    'coversGnssAbsent',z.nominalGnssWeight==0,'coversZeroLidarInformation',false), ...
                'feedbackAuthority',authority,'motionResponse',response, ...
                'gyroAvailabilityRequired',true,'allAuxiliaryOutputsRequired',true, ...
                'motionAvailabilityConfigured',m.motionAvailable,'scope',scope);
        end

        function runtime=runtime(cfg)
            h=cfg.invariantDesign;theta=cfg.invariantObserver.theta;ell=cfg.invariantObserver.headingLengthM;
            assert(isfield(h,'inputs') && isequaln(h.inputs,localizationSupport.invariantInputs(cfg)), ...
                'VehicleLocalization:StaleInvariantDesign','Temporary gains use different inputs; solve after configuring this run.');
            assert(theta==h.sigma,'VehicleLocalization:InvalidInvariantDesign','The nominal design requires theta=sigma.');
            v=localizationSupport.verifyInvariantDesign(h,cfg);
            assert(v.certified,'VehicleLocalization:InfeasibleInvariantDesign','Temporary design fails nominal fusion/response verification.');
            toPhysical=diag(theta.^[1,2,3,1,2,3,1,2]./[ones(1,6),ell,ell]);
            C=zeros(3,8);C(1,1)=1;C(2,4)=1;C(3,7)=1;Cp=C(1:2,:);
            matched=struct('P',h.P,'T',toPhysical,'theta',theta,'scalingExponents',[1;2;3;1;2;3;1;2], ...
                'kappa',h.kappaL/theta,'baselineFullStateCertified',false,'metricSource',"Nominal fusion polytope LMI");
            runtime=struct('framework',"invariant",'kind',cfg.kind,'P',h.P,'theta',theta,'toPhysical',toPhysical, ...
                'auxiliaryWeights',h.auxiliaryWeights(:),'gyroGainNormalized',h.gyroGainNormalized, ...
                'motionGainSchedule',h.motionGainSchedule,'LG',toPhysical*h.kappaG*(h.P\Cp.'), ...
                'lidarMatched',matched,'continuousCertificate',struct('design',h,'verification',v), ...
                'fullContinuousStateCertified',false,'surrogateContinuousLmiVerified',v.certified, ...
                'nominalFusionLmiVerified',v.nominalFusionLmiVerified, ...
                'designRoute',"One nominal fusion polytope SDP with simultaneous motion response", ...
                'baselineFullStateCertified',false,'actualOverlapCertified',false,'sampledSystemCertified',false,'scope',v.scope);
        end

        %% Finite nonlinear increment identities
        function [F,H,Hgnss]=secants(x,xhat,beta,betaRate,cfg)
        % f(x)-f(xhat)=F*(x-xhat), likewise for all nonlinear outputs.
        % These identities are finite differences, including at equal states.
            arguments
                x (8,1) double {mustBeReal,mustBeFinite}
                xhat (8,1) double {mustBeReal,mustBeFinite}
                beta (1,1) double {mustBeReal,mustBeFinite}
                betaRate (1,1) double {mustBeReal,mustBeFinite}
                cfg (1,1) struct
            end
            e=x-xhat;m=(x+xhat)/2;v=m([2,5]);a=m([3,6]);
            J=[0,-1;1,0];q=m(8)+betaRate;half=e(7)/2;
            cardinal=1;if half~=0,cardinal=sin(half)/half;end
            c=cos(half);d=[cos(m(7));sin(m(7))];n=J*d;
            dc=[cos(m(7)+beta);sin(m(7)+beta)];nc=J*dc;
            F=zeros(8);F(1,2)=1;F(2,3)=1;F(4,5)=1;F(5,6)=1;F(7,8)=1;
            F([3,6],[2,5])=(q^2+e(8)^2/4)*eye(2);
            F([3,6],[3,6])=2*q*J;F([3,6],8)=2*q*v+2*J*a;
            H=zeros(5,8);H(1,[2,5])=2*v.';
            H(2,[2,5])=a.';H(2,[3,6])=v.';
            H(3,[2,5])=(-J*a).';H(3,[3,6])=(J*v).';
            H(4,[2,5])=c*nc.';H(4,7)=-cardinal*dc.'*v;
            H(5,8)=1;
            Hgnss=zeros(2,8);Hgnss(:,[1,4])=eye(2);
            Hgnss(:,7)=cardinal*[d,n]*J*cfg.gnss.outputPoint.bodyOffset(:);
        end

        function N=measurementSecant(v,a,vm,am)
        % Packet increment from ONE primitive [dv(2); da(2); dgyro] vector.
        % The zero-valued kinematic outputs have no independent sensor noise.
            arguments
                v (2,1) double {mustBeReal,mustBeFinite}
                a (2,1) double {mustBeReal,mustBeFinite}
                vm (2,1) double {mustBeReal,mustBeFinite}
                am (2,1) double {mustBeReal,mustBeFinite}
            end
            v=(v+vm)/2;a=(a+am)/2;J=[0,-1;1,0];
            N=zeros(5,5);N(1,1:2)=2*v.';
            N(2,1:2)=a.';N(2,3:4)=v.';
            N(3,1:2)=(-J*a).';N(3,3:4)=(J*v).';
            N(5,5)=1;
        end

        %% Nominal fusion coefficient polytope
        function [vertices,bounds]=fusionVertices(cfg)
        % Analytic coefficient intervals, not endpoint samples of polynomials.
        % The affine nominal error matrix is enclosed by their Cartesian
        % vertices (the convexity step used after the DMVT in Lemma 1).
            s0=cfg.invariantObserver.fusion.minimumCourseSpeed;V=cfg.iss.maximumSpeed;
            validateattributes(s0,{'double'},{'real','finite','positive','scalar','<=',V});
            validateattributes([cfg.iss.maximumAcceleration,cfg.maximumTrackAngleRate], ...
                {'double'},{'real','finite','nonnegative'});
            A=cfg.iss.maximumAcceleration;Q=min(cfg.maximumTrackAngleRate,A/s0);
            crossBound=min(Q,A/(2*s0));
            bounds=[-Q,Q;0,Q^2;-A,A;.5,V^2/(s0^2+V^2); ...
                -A/(2*s0),A/(2*s0);-crossBound,crossBound;s0,V;s0^2,V^2];
            vertices=localizationSupport.intervalVertices(bounds);
        end

        function vertices=intervalVertices(bounds)
        % Vertices of an affine coefficient box; collapse degenerate axes.
            validateattributes(bounds,{'double'},{'real','finite','2d','ncols',2});
            assert(all(bounds(:,1)<=bounds(:,2)),'VehicleLocalization:InvalidFusionDomain','Invalid coefficient intervals.');
            active=find(bounds(:,2)>bounds(:,1));vertices=repmat(bounds(:,1).',2^numel(active),1);
            for j=1:numel(active)
                k=active(j);vertices(:,k)=bounds(k,1)+(bounds(k,2)-bounds(k,1))*double(bitget((0:size(vertices,1)-1).',j));
            end
        end

        function f=fusionFeatures(speed,courseRate,tangentialAcceleration,cfg)
        % Coefficients at a physical nominal point, useful for independent checks.
            denominator=cfg.invariantObserver.fusion.minimumCourseSpeed^2+speed^2;
            f=[courseRate,courseRate^2,tangentialAcceleration,speed^2/denominator, ...
                speed*tangentialAcceleration/denominator,speed^2*courseRate/denominator,speed,speed^2];
        end

        function [A,speed,pair,direction]=fusionMatrices(cfg,f,gyroGain)
        % The omitted velocity diagonal (|a|^2/denominator) of the pair Gram
        % is PSD. Dropping it is a global Loewner lower bound, with no split
        % parameter or acceleration-domain multiplier. All other terms remain.
            t=cfg.invariantObserver.theta;ell=cfg.invariantObserver.headingLengthM;
            E=eye(8);e2=E(:,2);e3=E(:,3);e5=E(:,5);e6=E(:,6);e7=E(:,7);
            A=zeros(8);A(1,2)=1;A(2,3)=1;A(4,5)=1;A(5,6)=1;A(7,8)=1;
            A([3,6],[2,5])=f(2)/t^2*eye(2);A([3,6],[3,6])=2*f(1)/t*[0,-1;1,0];
            A(6,8)=2*f(3)/(t^2*ell);A(8,8)=-gyroGain;
            speed=f(4)*(e2*e2.');T=e2*e3.'-e5*e6.';N=e2*e6.'+e5*e3.';
            pair=f(4)*(e3*e3.'+e6*e6.')+f(5)/t*(T+T.')+f(6)/t*(N+N.');
            direction=e5*e5.'-f(7)/(t*ell)*(e5*e7.'+e7*e5.')+f(8)/(t*ell)^2*(e7*e7.');
        end

        function blocks=fusionBlocks(d,cfg)
        % Single simultaneous-source condition, uniform over motion vertices.
            z=cfg.invariantObserver;E=eye(8);C=E([1,4,7],:);Cp=C(1:2,:);
            % ||translation-heading coupling|| <= b implies this lower bound.
            S=diag([cfg.iss.minimumPositionStrength*[1,1],cfg.iss.minimumHeadingStrength]) ...
                -(cfg.iss.maximumPositionHeadingCoupling+z.fusion.lidarOperatorMismatch)*eye(3);
            rho=norm(cfg.gnss.outputPoint.bodyOffset)/z.headingLengthM;
            % Young's inequality encloses every lever orientation for nominal
            % isotropic GNSS weight w. No claim about arbitrary anisotropic W.
            sensor=-2*d.kappaL*C.'*S*C-d.kappaG*z.nominalGnssWeight*(Cp.'*Cp) ...
                +d.kappaG*z.nominalGnssWeight*rho^2*(E(:,7)*E(:,7).');
            vertices=localizationSupport.fusionVertices(cfg);blocks=cell(1,size(vertices,1));
            for j=1:numel(blocks)
                [A,speed,pair,direction]=localizationSupport.fusionMatrices(cfg,vertices(j,:),d.gyroGainNormalized);
                M=d.P*A-d.auxiliaryWeights(1)*speed-d.auxiliaryWeights(2)*pair-d.auxiliaryWeights(3)*direction;
                blocks{j}=M+M.'+sensor+z.lambda*eye(8);
            end
        end

        %% Simultaneous nominal motion response
        function blocks=responseBlocks(R,k,cfg)
        % R is the motion Schur complement of the fixed-shear metric. It is
        % an independent affine SDP variable, so these constraints are LMIs.
            z=cfg.invariantObserver;t=z.theta;s0=z.fusion.minimumCourseSpeed;V=cfg.iss.maximumSpeed;
            rate=1/z.motionResponse.timeConstantS;A=[0,t;0,0];
            f=localizationSupport.intervalVertices([.5,V^2/(s0^2+V^2); ...
                -cfg.iss.maximumAcceleration/(2*s0),cfg.iss.maximumAcceleration/(2*s0)]);
            blocks=cell(1,size(f,1));
            for j=1:numel(blocks)
                speed=[f(j,1),0;0,0];pair=[0,f(j,2)/t;f(j,2)/t,f(j,1)];
                blocks{j}=R*A+A.'*R+2*rate*R-2*t*(k(1)*speed+k(2)*pair);
            end
        end

        function v=verifyMotionResponse(d,cfg)
            z=cfg.invariantObserver;
            consistent=isfield(d,'motionGainSchedule');
            if consistent
                g=d.motionGainSchedule;
                consistent=g.kind=="bounded-fusion-speed-v2" && g.minimumSpeedMps==z.fusion.minimumCourseSpeed ...
                    && g.maximumSpeedMps==cfg.iss.maximumSpeed;
            end
            inverse=d.P\eye(8);R=inverse(2:3,2:3)\eye(2);R=(R+R.')/2;
            blocks=localizationSupport.responseBlocks(R,d.auxiliaryWeights,cfg);
            worst=sensorWorstEigenvalue(blocks)/max(1,norm(R,2));
            headingFloor=cfg.iss.minimumHeadingStrength-cfg.iss.maximumPositionHeadingCoupling-z.fusion.lidarOperatorMismatch;
            headingRate=z.theta*d.kappaL*headingFloor/d.P(7,7);
            positionFloor=cfg.iss.minimumPositionStrength-cfg.iss.maximumPositionHeadingCoupling-z.fusion.lidarOperatorMismatch;
            positionRate=z.theta*d.kappaL*positionFloor/d.P(1,1);
            nominalSpeed=(z.fusion.minimumCourseSpeed+cfg.iss.maximumSpeed)/2;
            directionRate=d.auxiliaryWeights(3)*nominalSpeed^2/(z.theta*z.headingLengthM^2*d.P(7,7));
            v=struct('required',z.motionResponse.enabled, ...
                'verified',~z.motionResponse.enabled || (worst<0 && min([positionRate,headingRate,directionRate])>=1/z.motionResponse.timeConstantS*(1-1e-6)), ...
                'minimumDirectLidarPositionRate',positionRate,'minimumDirectLidarHeadingRate',headingRate,'nominalDirectDirectionHeadingRate',directionRate, ...
                'nominalCourseSpeedMps',nominalSpeed, ...
                'scheduleVerified',consistent,'timeConstantS',z.motionResponse.timeConstantS, ...
                'decayRatePerSecond',1/z.motionResponse.timeConstantS,'worstNormalizedControlEigenvalue',worst, ...
                'vertices',numel(blocks),'metric',R,'speedInterval',[z.fusion.minimumCourseSpeed,cfg.iss.maximumSpeed], ...
                'longitudinalPoseFeedbackRequired',false,'headingAuthorityRequiresLidar',true, ...
                'fullNonlinearResponseCertified',false, ...
                'scope',"Nominal straight-course longitudinal velocity/acceleration contraction without pose feedback, exact packets and zero heading/interface error. Direct LiDAR position/heading and midpoint-speed direction-heading authority use the same time target; these scalar bounds do not establish full-pose settling time. No nonlinear or sampled response assertion.");
        end

        function k=weights(d,speedSquared)
        % One bounded schedule, synthesized directly; no baseline/augmentation.
            validateattributes(speedSquared,{'double'},{'real','finite','nonnegative','scalar'});
            g=d.motionGainSchedule;k=d.auxiliaryWeights(:);
            k(1:2)=k(1:2)*g.maximumSpeedMps^2/(g.minimumSpeedMps^2+speedSquared);
        end

        function k=maximumWeights(d)
            k=localizationSupport.weights(d,0);
        end

        %% LiDAR measurement and correction
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
                localizationSupport.predictLidarPoseErrorVariance(zeros(2,1),cfg.lidar.errorCalibration);
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
            filter=localizationSupport.filterLidarPoseInformation(information,cfg,frame,direction,variance);
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
                    [r,J]=localizationSupport.supportRegistrationResiduals(frozen.sourceMean,frozen.sourceCovariance, ...
                        frozen.targetMean,frozen.targetCovariance,frozen.sourceAxis, ...
                        frozen.targetNormal,frozen.directionScale,local,frozen.noiseStandardDeviation);
                case "anisotropicD2D"
                    [r,J]=localizationSupport.gaussianRegistrationResiduals(frozen.sourceMean,frozen.sourceCovariance, ...
                        frozen.targetMean,frozen.targetCovariance,local,frozen.noiseStandardDeviation);
                case "geometricD2D"
                    [r,J]=injectionLineAndPointResiduals(frozen,local);
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
        % the regularized observer step stays bounded. For mixtures this is not
        % the true cost Hessian: pair-score covariance can cancel its apparent
        % curvature. Full rank does not certify a restoring sector or exclude
        % an incorrect local alignment.
        % Summing the scales fuses them in information form: the sharp scale
        % dominates near alignment, the wide ones keep a restoring force where the
        % sharp kernels have decayed; they do not guarantee a capture radius.
        % With cfg.evidenceWeights, map view
        % reliability and source temporal stability scale the mixture masses before
        % class balancing, as in registration. No pose is optimized, no
        % correspondence is selected and no acceptance test is applied; zero overlap
        % at every scale returns available=false. similarity reports the first
        % ladder entry with nonzero overlap. Map means are shifted to POSE before
        % evaluation, which leaves the overlap and its pose derivatives unchanged
        % and avoids UTM-scale cancellation.
        % With cfg.aggregation="landmark" the cost at each scale is instead the
        % per-landmark mixture likelihood -sum_j v_j log(eps + sum_i u_i N_ij) of
        % localizationSupport.semanticLandmarkLikelihood, with registration's support
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
            assert(isfield(cfg,'geometry') && ~isfield(cfg,'registration'), ...
                'VehicleLocalization:ObsoleteOverlapConfig','Rebuild overlap settings with overlapGradientConfig.');
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
            local=localizationSupport.selectLocalProbabilityCloud(fixedCloud,pose,cfg.localMapRadius);
            if local.components.numComponents==0 || movingCloud.components.numComponents==0
                measurement.seconds=toc(timer);return;
            end
            if cfg.viewConditioning,local=localizationSupport.conditionSemanticMapOnView(local,pose);end
            if cfg.aggregation=="landmark"
                measurement=injectionLandmarkLikelihoodGradient(local,movingCloud,pose,cfg,measurement);
                measurement.seconds=toc(timer);return;
            end
            [f,m]=localizationSupport.prepareSemanticRegistration(local,movingCloud,cfg.geometry);
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
                [fk,mk]=localizationSupport.balanceSemanticDistributions(fk,mk);
                if k==1
                    shared=intersect(unique(fk.semanticName(fk.mixtureWeight>0)),unique(mk.semanticName(mk.mixtureWeight>0)));
                    measurement.sharedClasses=reshape(string(shared),1,[]);
                end
                [energy,ascent,metric]=localizationSupport.semanticGaussianOverlap(fk,mk,origin);
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
            m=localizationSupport.evaluateLidarRegistrationResidual(model,model.anchorPose(:));
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

        %% Semantic mixture geometry
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
            calibrationStatus=localizationSupport.validateRegistrationCalibration(fixedCloud,movingCloud);
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
            available = geometryHasHeight(fixed) && geometryHasHeight(moving);
            useHeight = mode~="xy" && available && isfinite(translation);
            assert(mode~="xyz" || useHeight, 'VehicleLocalization:HeightUnavailable', ...
                'XYZ matching requires height in both clouds and a finite heightTranslation.');
            dimension = 2+useHeight;
            fixed = localizationSupport.projectSemanticProbabilityCloud(fixedCloud,dimension).components;
            moving = localizationSupport.projectSemanticProbabilityCloud(movingCloud,dimension).components;
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
                fe=localizationSupport.semanticGaussianOverlap(f,f,[0 0 0]);
                me=localizationSupport.semanticGaussianOverlap(m,m,[0 0 0]);
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
                    [energy,gradient]=geometrySpatialGaussianOverlap(fixed,moving,pose);
                else
                    energy=geometrySpatialGaussianOverlap(fixed,moving,pose);
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
        % localizationSupport.prepareSemanticOverlapGeometry (orientationNormal or supportTangent,
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
                out.covariance(:,:,g)=geometryMomentMatch(c.covariance(:,:,i),c.mean(i,:),out.mean(g,:),wi);
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
                        S=geometryMomentMatch(c.covarianceXYZ(:,:,i),c.meanXYZ(i,:),mu,wi);S(1:2,1:2)=out.covariance(:,:,g);
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
                        mu=sum(h.mean(i,:).*wi,1)/sum(wi);e.mean(g,:)=mu;e.covariance(:,:,g)=geometryMomentMatch(h.covariance(:,:,i),h.mean(i,:),mu,wi);
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
            if nargin<4, cfg=semanticGeometryConfig(); end
            [fixed,moving,heightDetails] = localizationSupport.prepareSemanticRegistration(fixedCloud,movingCloud,cfg);
            [fixed,moving] = localizationSupport.balanceSemanticDistributions(fixed,moving);
            pose = double(poseXYTheta(:).');
            assert(numel(pose)==3 && all(isfinite(pose)), 'Expected finite [x y yaw].');
            [crossEnergy, gradient] = localizationSupport.semanticGaussianOverlap(fixed, moving, pose);
            fixedSelf = localizationSupport.semanticGaussianOverlap(fixed, fixed, [0 0 0]);
            movingSelf = localizationSupport.semanticGaussianOverlap(moving, moving, [0 0 0]);
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
            [residual,J,precision]=localizationSupport.gaussianRegistrationResiduals(sourceMean,sourceCov,targetMean,targetCov,pose,noise);
            residual=residual(1:3,:);J=J(1:3,:,:);
            r=[cos(pose(3)) -sin(pose(3));sin(pose(3)) cos(pose(3))];
            residual(3,:)=(sum((sourceAxis*r.').*targetNormal,2).*directionScale).';
            J(3,:,:)=0;
            J(3,3,:)=sum((sourceAxis*[r(:,2),-r(:,1)].').*targetNormal,2).*directionScale;
        end

        %% Sensor tilt and extrinsic calibration
        function tilt=estimateLidarImuTilt(imu,wheel,queryTime,cfg)
        % estimateLidarImuTilt Replay the streaming filter using past packets only.
        % imu arrivalTime/deviceTime, specificForce, angularVelocity; wheel
        % arrivalTime/speed. Query times are packet-arrival clock times, in seconds.
            if nargin<4,cfg=lidarImuTiltConfig();end
            queryTime=queryTime(:);n=numel(queryTime);
            assert(all(diff(queryTime)>0),'VehicleLocalization:InvalidTiltClock','Queries must increase.');
            assert(all(diff(imu.arrivalTime)>0) && all(diff(imu.deviceTime)>0) && ...
                all(diff(wheel.arrivalTime)>0),'VehicleLocalization:InvalidTiltClock','Sensor clocks must increase.');
            rotation=repmat(eye(3),1,1,n);angles=zeros(n,2);valid=false(n,1);aligned=valid;
            age=nan(n,1);sourceIndex=zeros(n,1);state=[];i=0;j=0;latest=eye(3);last=struct();
            for k=1:n
                while i<numel(imu.arrivalTime) && imu.arrivalTime(i+1)<=queryTime(k)
                    i=i+1;
                    while j<numel(wheel.arrivalTime) && wheel.arrivalTime(j+1)<=imu.arrivalTime(i),j=j+1;end
                    if j==0,continue;end
                    assert(imu.arrivalTime(i)-wheel.arrivalTime(j)<=cfg.maximumWheelAgeSeconds, ...
                        'VehicleLocalization:TiltWheelGap','Tilt compensation requires a current wheel sample.');
                    sample=struct('time',imu.deviceTime(i),'specificForce',imu.specificForce(i,:), ...
                        'angularVelocity',imu.angularVelocity(i,:));
                    [latest,state,last]=localizationSupport.updateLidarImuTilt(sample,wheel.speed(j),state,cfg);
                end
                if ~isempty(state)
                    age(k)=queryTime(k)-imu.arrivalTime(i);
                    assert(age(k)<=cfg.maximumImuAgeSeconds,'VehicleLocalization:TiltImuGap','Latest IMU sample is stale.');
                    rotation(:,:,k)=latest;angles(k,:)=[last.roll,last.pitch];valid(k)=true;aligned(k)=last.aligned;sourceIndex(k)=i;
                end
            end
            tilt=struct('rotation',rotation,'angles',angles,'valid',valid,'aligned',aligned, ...
                'ageSeconds',age,'sourceIndex',sourceIndex,'time',queryTime,'configuration',cfg, ...
                'schemaVersion',1,'source',"front-ouster-raw-imu",'referencePoseUsed',false,'causal',true);
        end

        function [rotation,state,diagnostic]=updateLidarImuTilt(sample,speed,state,cfg)
        % updateLidarImuTilt One causal six-axis IMU update; no pose/reference input.
        % specificForce and angularVelocity are columns in stored point axes. time
        % is native IMU time. speed is a currently available wheel measurement.
        % Only stationary measurements initialize/update gyro bias. The gravity
        % direction is propagated on the unit sphere and corrected by specific force
        % minus the planar wheel-acceleration estimate. No yaw is observed/injected.
            if nargin<4,cfg=lidarImuTiltConfig();end
            f=sample.specificForce(:);w=sample.angularVelocity(:);t=sample.time;
            assert(numel(f)==3 && numel(w)==3 && all(isfinite([t;f;w;speed])), ...
                'VehicleLocalization:InvalidTiltImu','Require finite six-axis IMU and wheel speed.');
            stationary=abs(speed)<cfg.stationarySpeedMps && norm(w)<cfg.stationaryGyroRadps;
            if isempty(state)
                assert(stationary && norm(f)>cfg.gravityMps2/2, ...
                    'VehicleLocalization:TiltAlignmentRequired','Start with stationary six-axis IMU and measured wheel speed.');
                state=struct('time',t,'up',f/norm(f),'gyroBias',w,'biasCount',1, ...
                    'stationarySeconds',0,'speed',speed,'initialized',false);
                acceleration=0;
            else
                dt=t-state.time;
                assert(dt>0 && dt<=cfg.maximumImuStepSeconds, ...
                    'VehicleLocalization:TiltImuGap','IMU timestamps must increase without an excessive gap.');
                filtered=state.speed+(1-exp(-dt/cfg.speedTimeSeconds))*(speed-state.speed);
                acceleration=(filtered-state.speed)/dt;state.speed=filtered;
                if stationary
                    state.biasCount=state.biasCount+1;
                    state.gyroBias=state.gyroBias+(w-state.gyroBias)/state.biasCount;
                    state.stationarySeconds=state.stationarySeconds+dt;
                end
                angular=w-state.gyroBias;angle=norm(angular)*dt;
                if angle>0
                    axis=-angular/norm(angular);u=state.up;
                    state.up=u*cos(angle)+cross(axis,u)*sin(angle)+axis*dot(axis,u)*(1-cos(angle));
                end
                state.time=t;
                state.initialized=state.stationarySeconds>=cfg.minimumAlignmentSeconds;
            end
            angular=w-state.gyroBias;
            gravity=f-[acceleration;state.speed*angular(3);0];
            accepted=abs(norm(gravity)-cfg.gravityMps2)<=cfg.maximumGravityNormErrorMps2;
            if accepted
                measured=gravity/norm(gravity);
                if stationary && ~state.initialized
                    alpha=1/state.biasCount;
                elseif exist('dt','var')
                    alpha=1-exp(-dt/cfg.correctionTimeSeconds);
                else
                    alpha=0;
                end
                state.up=state.up+alpha*(measured-state.up);state.up=state.up/norm(state.up);
            end
            roll=atan2(state.up(2),state.up(3));pitch=atan2(-state.up(1),hypot(state.up(2),state.up(3)));
            cr=cos(roll);sr=sin(roll);cp=cos(pitch);sp=sin(pitch);
            rotation=[cp,sp*sr,sp*cr;0,cr,-sr;-sp,cp*sr,cp*cr];
            diagnostic=struct('roll',roll,'pitch',pitch,'aligned',state.initialized, ...
                'gravityAccepted',accepted,'gravityNormMps2',norm(gravity),'gyroBias',state.gyroBias);
        end

        function assertSensorOnlyLidarTilt(tilt)
        % assertSensorOnlyLidarTilt Reject legacy caches carrying reference-derived tilt.
            assert(isstruct(tilt) && all(isfield(tilt, ...
                {'schemaVersion','source','referencePoseUsed','causal','configuration'})) && ...
                tilt.schemaVersion==1 && ~tilt.referencePoseUsed && tilt.causal && ...
                ismember(string(tilt.source),["front-ouster-raw-imu","disabled"]), ...
                'VehicleLocalization:ReferenceTiltCache','Regenerate source inputs using sensor-only tilt; legacy reference-attitude caches are not allowed.');
        end

        function [calibration, report] = fitLidarPitchCalibration(featureData, options)
        % fitLidarPitchCalibration: Offline pitch-only frame increment from static features.
        % Compare the first frame with later frames using fixed nearest-XY pairs.
        % Fit one rotation about body Y by minimizing squared per-frame median Z
        % residuals, with equal frame weight. This is a dataset calibration candidate,
        % not a six-axis extrinsic solution or an online localization state. Recorded
        % pose error and mounting error cannot be separated by this fit alone.
            arguments
                featureData (1,1) struct
                options.FeatureName (1,1) string = "curb"
                options.Identifier (1,1) string = "offlinePitchCandidate"
                options.MaximumPairDistance (1,1) double {mustBePositive} = .3
                options.MaximumPitchDegrees (1,1) double {mustBePositive} = 8
                options.MinimumForwardTravel (1,1) double {mustBePositive} = 1
                options.MaximumHeadingChangeDegrees (1,1) double {mustBePositive} = 5
            end
            if isfield(featureData,'frameCalibration')
                existing=validateLidarFrameCalibration(featureData.frameCalibration);
                assert(norm(existing.rotation-eye(3),'fro')<1e-10 && norm(existing.translation)<1e-10, ...
                    'VehicleLocalization:CalibrationAlreadyApplied','Fit from identity-calibrated observations.');
            end
            feature=find(string(featureData.featureNames)==options.FeatureName,1);
            assert(~isempty(feature),'Requested calibration feature is absent.');
            poses=featureData.framePoseTable;
            [p0,r0]=calibrationPoseGeometry(poses(1,:));
            reference=featureData.pointsByFeatureFrame{feature,1};
            pairs=cell(0,1); pairFrames=zeros(0,1); travels=zeros(0,1); counts=zeros(0,1);
            for j=2:height(poses)
                [position,rotation]=calibrationPoseGeometry(poses(j,:));
                travel=(position-p0)*r0;
                heading=acos(max(-1,min(1,dot(r0(:,1),rotation(:,1)))));
                points=featureData.pointsByFeatureFrame{feature,j};
                if isempty(reference) || isempty(points) || abs(travel(1))<options.MinimumForwardTravel || ...
                        heading>deg2rad(options.MaximumHeadingChangeDegrees), continue; end
                % Chunked search needs no Statistics and Machine Learning Toolbox.
                index=zeros(size(reference,1),1); distance=inf(size(index));
                for first=1:256:size(reference,1)
                    selection=first:min(first+255,size(reference,1));
                    [distance(selection),index(selection)]=min( ...
                        (reference(selection,1)-points(:,1).').^2+(reference(selection,2)-points(:,2).').^2,[],2);
                end
                keep=distance<options.MaximumPairDistance^2;
                if nnz(keep)<10, continue; end
                pair=struct('reference',(reference(keep,:)-p0)*r0,'moving',(points(index(keep),:)-position)*rotation, ...
                    'referenceRotation',r0,'movingRotation',rotation,'heightDifference',position(3)-p0(3));
                pairs{end+1,1}=pair; %#ok<AGROW>
                pairFrames(end+1,1)=featureData.frameIndices(j); %#ok<AGROW>
                travels(end+1,1)=travel(1); counts(end+1,1)=nnz(keep); %#ok<AGROW>
            end
            assert(numel(pairs)>=3,'VehicleLocalization:InsufficientCalibrationExcitation', ...
                'Need at least three translated, overlapping frame pairs with similar headings.');
            bound=deg2rad(options.MaximumPitchDegrees);
            objective=@(angle) mean(calibrationHeightResidual(pairs,angle).^2);
            angle=fminbnd(objective,-bound,bound,optimset('TolX',1e-9,'Display','off'));
            assert(abs(angle)<.99*bound,'VehicleLocalization:CalibrationSearchBoundary', ...
                'Pitch estimate reached its allowed range; inspect frame and pose conventions.');
            calibration=lidarFrameCalibrationConfig();
            calibration.rotation=calibrationPitchRotation(angle); calibration.identifier=options.Identifier;
            before=calibrationHeightResidual(pairs,0); after=calibrationHeightResidual(pairs,angle);
            report=struct('pitchIncrementDegrees',-rad2deg(angle),'fittedAngleDegrees',rad2deg(angle), ...
                'referenceFrame',featureData.frameIndices(1),'feature',options.FeatureName, ...
                'parametersEstimated',"pitchOnly",'translationEstimated',false,'onlineState',"X,Y,psi", ...
                'rmsFrameMedianBeforeM',sqrt(mean(before.^2)),'rmsFrameMedianAfterM',sqrt(mean(after.^2)), ...
                'interpretation',"Offline calibration candidate; requires validation on frames not used in fitting.");
            report.pairs=table(pairFrames,travels,counts,before,after, ...
                'VariableNames',{'mapFrame','forwardTravelM','pairCount','medianBeforeM','medianAfterM'});
        end

        function [calibration,report]=fitLidarTranslationCalibration(queryPoses,fixedPoses,matchedPoses,options)
        % fitLidarTranslationCalibration Estimate a planar stored-to-reference offset.
        % matchedPoses registers an unshifted query to an unshifted fixed-frame cloud
        % projected using fixedPoses. All poses are [map X, map Y, yaw radians].
        % Pair selection/registration is offline; no evaluation pose is used online.
        % Planar motion does not identify vertical translation or mounting rotation.
            arguments
                queryPoses (:,3) double {mustBeFinite,mustBeReal}
                fixedPoses (:,3) double {mustBeFinite,mustBeReal}
                matchedPoses (:,3) double {mustBeFinite,mustBeReal}
                options.HuberScaleM (1,1) double {mustBePositive}=.10
                options.MinimumPairs (1,1) double {mustBeInteger,mustBePositive}=8
                options.MinimumAngularExcitation (1,1) double {mustBePositive}=.08
            end
            n=size(queryPoses,1);
            assert(isequal(size(queryPoses),size(fixedPoses),size(matchedPoses)) && n>=options.MinimumPairs, ...
                'VehicleLocalization:InsufficientCalibrationPairs','Expected matching arrays with sufficient pairs.');
            A=zeros(2*n,2);delta=reshape((matchedPoses(:,1:2)-queryPoses(:,1:2)).',[],1);
            for k=1:n
                A(2*k-1:2*k,:)=calibrationRotation(queryPoses(k,3))-calibrationRotation(fixedPoses(k,3));
            end
            singular=svd(A);
            assert(min(singular)/sqrt(n)>=options.MinimumAngularExcitation, ...
                'VehicleLocalization:InsufficientCalibrationExcitation','Turning motion is required to estimate the offset.');
            offset=A\delta;
            for k=1:20
                residual=reshape(A*offset-delta,2,[]).';
                weight=min(1,options.HuberScaleM./max(vecnorm(residual,2,2),eps));
                rootWeight=repelem(sqrt(weight),2);
                offset=(A.*rootWeight)\(delta.*rootWeight);
            end
            calibration=lidarFrameCalibrationConfig();
            calibration.translation=[offset.',0];calibration.identifier="offlinePlanarReferencePointFit";
            errors=reshape(A*offset-delta,2,[]).';
            report=struct('pairs',n,'translationXYM',offset.','rotationEstimated',false,'verticalTranslationEstimated',false, ...
                'normalizedSingularValues',singular.'/sqrt(n),'huberScaleM',options.HuberScaleM, ...
                'beforeRmseM',sqrt(mean(sum(reshape(delta,2,[]).^2,1))), ...
                'afterRmseM',sqrt(mean(sum(errors.^2,2))),'pairResidualM',vecnorm(errors,2,2), ...
                'interpretation',"Effective stored-axis to recorded-INS-reference translation; not a surveyed mounting position");
        end

        %% Recorded pose transforms
        function [pose, tiltRotation, heightTranslation] = poseRowToPlanarPose(row)
        % poseRowToPlanarPose Split the selected mapping pose into SE(2) and tilt.
        % Uses exactly the transform consumed by registerPointsToGlobalFrame.
        % R_global = R_yaw * tiltRotation. Height is external mapping metadata,
        % not a registration state; it is NaN for legacy records without height.
            [rotation,translation]=localizationSupport.poseRowToRigidTransform(row);
            yaw=atan2(rotation(2,1),rotation(1,1));
            r=[cos(yaw),-sin(yaw),0;sin(yaw),cos(yaw),0;0,0,1];
            tiltRotation=r.'*rotation;
            pose=[translation(1:2),yaw];
            heightTranslation=translation(3);
        end

        function [rotation,translation] = poseRowToRigidTransform(row)
        % poseRowToRigidTransform Resolve explicit mapping poses or legacy ODOM rows.
        % Canonical pose_* fields take precedence and must be complete and finite.
        % Legacy records retain their original quaternion or yaw-only interpretation.
            names=string(row.Properties.VariableNames);
            assert(height(row)==1,'VehicleLocalization:PoseRow','Exactly one pose row is required.');
            canonical=any(startsWith(names,"pose_"));
            if canonical
                fields=["pose_x_m","pose_y_m","pose_z_m","pose_qw","pose_qx","pose_qy","pose_qz"];
                assert(all(ismember(fields,names)),'VehicleLocalization:IncompletePose','Canonical pose is incomplete.');
                values=double(row{1,cellstr(fields)});
                assert(all(isfinite(values)),'VehicleLocalization:InvalidPose','Canonical pose must be finite.');
                translation=values(1:3);q=values(4:7);
                assert(norm(q)>0,'VehicleLocalization:InvalidPose','Quaternion must be nonzero.');
            else
                translation=[double(row.odom_x_m(1)),double(row.odom_y_m(1)),NaN];
                if ismember("odom_z_m",names),translation(3)=double(row.odom_z_m(1));end
                q=[];fields=["odom_qw","odom_qx","odom_qy","odom_qz"];
                if all(ismember(fields,names))
                    candidate=double(row{1,cellstr(fields)});
                    if all(isfinite(candidate)) && norm(candidate)>0,q=candidate;end
                end
                if isempty(q)
                    yaw=deg2rad(90-double(row.azimuth_deg(1)));
                    rotation=[cos(yaw),-sin(yaw),0;sin(yaw),cos(yaw),0;0,0,1];
                    return;
                end
            end
            q=q/norm(q);w=q(1);x=q(2);y=q(3);z=q(4);
            rotation=[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w); ...
                2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w); ...
                2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)];
        end

        %% Wheel velocity
        function estimate = estimateWheelLongitudinalSpeed(data,cfg)
        % estimateWheelLongitudinalSpeed Fuse four wheel rates on a causal time grid.
        % Motion fields: time, steeringAngle, yawRate, longitudinalAcceleration.
        % wheels.time and wheels.angularVelocity (FL, FR, RL, RR; rad/s) are native
        % packets. NaN wheels are unavailable. No reference, GNSS, or twist is read.
        % Geometry projects rolling speed onto the body x axis, assuming small tire
        % lateral slip. A finite wheel-spread gate cannot detect common-mode slip.
            arguments
                data (1,1) struct
                cfg (1,1) struct = wheelSpeedObserverConfig()
            end
            t=data.time(:);wt=data.wheels.time(:);w=data.wheels.angularVelocity;
            assert(numel(t)>=2 && all(isfinite(t)) && all(diff(t)>0) && ...
                all(isfinite(wt)) && all(diff(wt)>0) && size(w,1)==numel(wt) && size(w,2)==4, ...
                'VehicleLocalization:InvalidWheelInput','Require increasing times and four wheel columns.');
            for field=["steeringAngle","yawRate","longitudinalAcceleration"]
                assert(numel(data.(field))==numel(t) && all(isfinite(data.(field))), ...
                    'VehicleLocalization:InvalidWheelInput','Require aligned finite motion.');
            end
            settings=[cfg.wheelbase,cfg.frontTrack,cfg.rearTrack,cfg.rearCgDistance,cfg.filterTimeConstant, ...
                cfg.maximumWheelAge,cfg.consensusThreshold,cfg.maximumWheelRate,cfg.maximumSteeringAngle, ...
                cfg.stationaryWheelRate,cfg.stationaryYawRate];
            assert(isequal(size(cfg.effectiveRadius),[1,4]) && all(isfinite(cfg.effectiveRadius)) && all(cfg.effectiveRadius>0) && ...
                isequal(size(cfg.lagCompensation),[1,4]) && all(isfinite(cfg.lagCompensation)) && all(cfg.lagCompensation>=0) && ...
                all(isfinite(settings)) && all(settings>=0) && isscalar(cfg.initialSpeed) && ...
                cfg.wheelbase>0 && cfg.frontTrack>0 && cfg.rearTrack>0 && cfg.rearCgDistance>=0 && ...
                cfg.filterTimeConstant>0 && cfg.maximumWheelAge>0 && cfg.consensusThreshold>0 && ...
                cfg.maximumWheelRate>0 && cfg.maximumSteeringAngle>0 && cfg.maximumSteeringAngle<pi/2 && ...
                ismember(cfg.method,["robust","mean","rear"]), ...
                'VehicleLocalization:InvalidWheelConfig','Invalid wheel observer configuration.');
            pre=find(wt<=t(1),1,'last');
            if isempty(pre) || t(1)-wt(pre)>cfg.maximumWheelAge
                assert(isfinite(cfg.initialSpeed),'VehicleLocalization:WheelInitialization', ...
                    'Supply an explicit initial speed when no recent wheel packet exists.');
                pre=find(wt>t(1),1);if isempty(pre),pre=numel(wt)+1;end
            end
            cuts=unique([t;wt(wt>t(1) & wt<=t(end))]);
            n=numel(t);vout=nan(n,1);valid=false(n,1);counts=zeros(n,1);spread=nan(n,1);
            rejected=false(n,4);age=nan(n,1);raw=nan(n,4);
            y=[cfg.frontTrack,-cfg.frontTrack,cfg.rearTrack,-cfg.rearTrack]/2;
            v=cfg.initialSpeed;lastAccepted=-Inf;lastPacket=-Inf;lastWheel=zeros(1,4);lastCount=0;stationary=false;
            lastReject=false(1,4);lastSpread=NaN;ih=1;iw=pre;io=1;
            for k=1:numel(cuts)
                now=cuts(k);
                while ih<numel(t) && t(ih+1)<=now,ih=ih+1;end
                r=data.yawRate(ih);a=data.longitudinalAcceleration(ih)+cfg.rearCgDistance*r^2;
                delta=data.steeringAngle(ih);
                assert(abs(delta)<=cfg.maximumSteeringAngle, ...
                    'VehicleLocalization:InvalidWheelInput','Steering exceeds the wheel geometry envelope.');
                while iw<=numel(wt) && wt(iw)<=now
                    angle=[atan2(cfg.wheelbase*tan(delta),cfg.wheelbase-y(1:2)*tan(delta)),0,0];
                    available=isfinite(w(iw,:)) & abs(w(iw,:))<=cfg.maximumWheelRate;
                    candidate=w(iw,:).*cfg.effectiveRadius.*cos(angle)+r*y+cfg.lagCompensation*a;
                    stationary=all(available) && all(abs(w(iw,:))<=cfg.stationaryWheelRate) && abs(r)<=cfg.stationaryYawRate;
                    if stationary,candidate(:)=0;end
                    selected=available;
                    if cfg.method=="rear",selected(1:2)=false;end
                    if cfg.method=="robust" && nnz(selected)>=3
                        center=median(candidate(selected));
                        selected=selected & abs(candidate-center)<=cfg.consensusThreshold;
                    end
                    if cfg.method=="robust" && nnz(selected)==2 && ...
                            max(candidate(selected))-min(candidate(selected))>2*cfg.consensusThreshold
                        selected(:)=false;
                    end
                    lastCount=nnz(selected);lastReject=~selected;lastWheel=candidate;
                    if lastCount>=2
                        measurement=mean(candidate(selected));
                        if wt(iw)<now && ~stationary,measurement=measurement+a*(now-wt(iw));end
                        lastSpread=max(candidate(selected))-min(candidate(selected));
                        if ~isfinite(v) || stationary
                            v=measurement;
                        else
                            dt=min(now-lastPacket,cfg.maximumWheelAge);
                            gain=-expm1(-dt/cfg.filterTimeConstant);
                            v=v+gain*(measurement-v);
                        end
                        lastAccepted=wt(iw);
                    end
                    lastPacket=wt(iw);iw=iw+1;
                end
                if io<=n && now==t(io)
                    vout(io)=v;valid(io)=isfinite(v) && now-lastAccepted<=cfg.maximumWheelAge;
                    counts(io)=lastCount;spread(io)=lastSpread;rejected(io,:)=lastReject;
                    age(io)=now-lastAccepted;raw(io,:)=lastWheel;io=io+1;
                end
                if k<numel(cuts) && ~(stationary && now-lastAccepted<=cfg.maximumWheelAge)
                    v=v+a*(cuts(k+1)-now);
                end
            end
            estimate=struct('time',t,'longitudinalSpeed',vout,'valid',valid, ...
                'acceptedWheelCount',counts,'rejectedWheels',rejected,'wheelSpread',spread, ...
                'sourceAge',age,'wheelCandidates',raw,'config',cfg, ...
                'referenceUsedAtRuntime',false,'commonModeSlipObservable',false);
        end

        %% Causal source windows
        function [cloud,history,details,confirmedCurrent]=updateLocalizationSourceWindow(current,timestamp,motionPose,history,cfg,motionSegment)
        % updateLocalizationSourceWindow Confirm and merge recent coarse XY distributions.
        % motionPose is a cumulative planar wheel/gyro pose in any fixed odometry
        % frame. Only relative motion is used. Store Gaussian statistics, not points.
        % Same-class distributions are associated one-to-one per scan, then merged
        % by equal-scan mixture moments. Only repeated tracks enter matching. A track
        % receives one vote per acquisition, never per point or neighboring pillar.
        % Center compatibility and covariance shape are both required. Shape must
        % agree with each original observation, not only a broadened pooled Gaussian.
        % Stability is detection count / configured horizon, with no startup exemption.
        % Covariance is spatial scatter, not covariance of an independent sample mean.
        % The pooled product is XY only: vertical ego motion is not supplied here.
        % Its heightEvidence sidecar contains current-acquisition XYZ statistics only,
        % indexed by confirmed tracks. Old heights are never transported with SE(2).
        % confirmedCurrent contains only this acquisition's confirmed distributions.
        % It lets a pose graph consume each observation once without overlapping stacks.
        % motionSegment optionally identifies uninterrupted valid odometry. Zero
        % withdraws history transport and emits the fresh current cloud unchanged;
        % a new positive segment starts a new history. No repeated-detection weight
        % is invented for a single-scan fallback.
            if nargin<4 || isempty(history),history=struct('clouds',{{}},'time',zeros(0,1),'motion',zeros(0,3));end
            if nargin<5,cfg=localizationSourceWindowConfig();end
            if nargin<6,motionSegment=1;end
            assert(isscalar(motionSegment) && isfinite(motionSegment) && motionSegment>=0 && fix(motionSegment)==motionSegment, ...
                'VehicleLocalization:InvalidMotionSegment','Require a nonnegative integer motion segment.');
            assert(isscalar(timestamp)&&isfinite(timestamp)&&isreal(timestamp), ...
                'VehicleLocalization:InvalidWindowTime','Require a finite acquisition timestamp.');
            assert(motionSegment==0 || (numel(motionPose)==3&&isreal(motionPose)&&all(isfinite(motionPose))), ...
                'VehicleLocalization:InvalidWindowMotion','Require cumulative planar odometry.');
            assert(cfg.minimumDetectionFrames>=2 && cfg.minimumDetectionFrames==fix(cfg.minimumDetectionFrames) && ...
                isfinite(cfg.minimumDetectionFrames) && cfg.maximumFrames>=cfg.minimumDetectionFrames && ...
                cfg.maximumFrames==fix(cfg.maximumFrames) && isfinite(cfg.maximumFrames) && ...
                isfinite(cfg.maximumAgeSeconds) && cfg.maximumAgeSeconds>=0 && ...
                isfinite(cfg.maximumAssociationDistance) && cfg.maximumAssociationDistance>0 && ...
                isfinite(cfg.maximumStandardizedDistance) && cfg.maximumStandardizedDistance>0 && ...
                isfinite(cfg.associationNoiseStandardDeviation) && cfg.associationNoiseStandardDeviation>0 && ...
                isnumeric(cfg.maximumShapeDistance)&&isreal(cfg.maximumShapeDistance)&&isscalar(cfg.maximumShapeDistance)&& ...
                cfg.maximumShapeDistance>0 && ...
                isnumeric(cfg.shapeVarianceFloor)&&isreal(cfg.shapeVarianceFloor)&&isscalar(cfg.shapeVarianceFloor)&& ...
                isfinite(cfg.shapeVarianceFloor)&&cfg.shapeVarianceFloor>0, ...
                'VehicleLocalization:InvalidWindowConfiguration','Invalid source-window bounds.');
            assert(isempty(history.time)||timestamp>history.time(end), ...
                'VehicleLocalization:NonmonotonicWindowTime','Reset the source window before rewinding time.');
            mappingSupport.validateSemanticProbabilityCloud(current);
            assert(~isfield(current.components,'temporalStability'), ...
                'VehicleLocalization:AlreadyStackedSource','Supply a fresh single-scan cloud, not a previous horizon product.');
            if motionSegment==0
                cloud=localizationSupport.projectSemanticProbabilityCloud(current,2);
                confirmedCurrent=cloud;
                confirmedCurrent.acquisitionTime=timestamp;
                confirmedCurrent.observationScope="currentAcquisitionWithoutMotion";
                history=struct('clouds',{{}},'time',zeros(0,1),'motion',zeros(0,3),'segment',0);
                details=struct('frameCount',1,'spanSeconds',0,'componentCount',cloud.components.numComponents, ...
                    'motionSegment',0,'historyTransportApplied',false,'motionSource',"unavailable; current scan only");
                return;
            end
            if isfield(history,'segment') && history.segment~=motionSegment
                history=struct('clouds',{{}},'time',zeros(0,1),'motion',zeros(0,3));
            end
            history.segment=motionSegment;
            keep=history.time>=timestamp-cfg.maximumAgeSeconds;
            history.clouds=history.clouds(keep);history.time=history.time(keep);history.motion=history.motion(keep,:);
            history.clouds{end+1,1}=localizationSupport.projectSemanticProbabilityCloud(current,2);
            history.time(end+1,1)=timestamp;history.motion(end+1,:)=double(motionPose(:).');
            first=max(1,numel(history.time)-cfg.maximumFrames+1);
            history.clouds=history.clouds(first:end);history.time=history.time(first:end);history.motion=history.motion(first:end,:);
            cloud=history.clouds{end};sets=cell(numel(history.time),1);origin=history.motion(end,:);
            rotation=[cos(origin(3)) -sin(origin(3));sin(origin(3)) cos(origin(3))];
            for k=1:numel(sets)
                localizationSupport.validateRegistrationCalibration(cloud,history.clouds{k});
                c=history.clouds{k}.components;delta=history.motion(k,:)-origin;
                r=[cos(delta(3)) -sin(delta(3));sin(delta(3)) cos(delta(3))];
                c.mean=c.mean*r.'+delta(1:2)*rotation;
                c.covariance=pagemtimes(pagemtimes(r,c.covariance),r.');
                c.semanticProbability=windowFieldOrUnit(c,'semanticProbability');
                c.occupancyProbability=windowFieldOrUnit(c,'occupancyProbability');
                sets{k}=struct('mean',c.mean,'covariance',c.covariance,'semanticName',c.semanticName, ...
                    'mixtureWeight',c.mixtureWeight,'semanticProbability',c.semanticProbability, ...
                    'occupancyProbability',c.occupancyProbability);
            end
            [c,statistics,currentIndices,stability,pooledIndices]=windowStableTracks(sets,cfg);
            cloud.components=c;
            heightEvidence=localizationSupport.getHeightEvidence(current);
            cloud.heightEvidence=windowSelectHeightEvidence(heightEvidence,currentIndices,pooledIndices,c.numComponents);
            confirmedCurrent=history.clouds{end};raw=sets{end};
            confirmedCurrent.components=struct('mean',raw.mean(currentIndices,:), ...
                'covariance',raw.covariance(:,:,currentIndices),'semanticName',raw.semanticName(currentIndices), ...
                'mixtureWeight',raw.mixtureWeight(currentIndices),'semanticProbability',raw.semanticProbability(currentIndices), ...
                'occupancyProbability',raw.occupancyProbability(currentIndices),'temporalStability',stability, ...
                'numComponents',numel(currentIndices));
            confirmedCurrent.acquisitionTime=timestamp;
            confirmedCurrent.observationScope="confirmedCurrentAcquisition";
            confirmedCurrent.heightEvidence=windowSelectHeightEvidence(heightEvidence,currentIndices, ...
                (1:numel(currentIndices)).',numel(currentIndices));
            details=struct('frameCount',numel(history.time),'oldestTimestamp',history.time(1), ...
                'newestTimestamp',timestamp,'spanSeconds',timestamp-history.time(1), ...
                'componentCount',c.numComponents,'currentComponentCount',current.components.numComponents, ...
                'unfilteredComponentCount',statistics.inputComponents, ...
                'trackCount',statistics.tracks,'rejectedSingletons',statistics.singletons, ...
                'rejectedTracks',statistics.rejected,'minimumDetectionFrames',cfg.minimumDetectionFrames, ...
                'minimumSupport',statistics.minimumSupport,'meanStability',statistics.meanStability, ...
                'positionCompatiblePairs',statistics.positionPairs,'shapeRejectedPairs',statistics.shapeRejectedPairs, ...
                'maximumShapeDistance',cfg.maximumShapeDistance, ...
                'shapeCompatibility',"covariance Bhattacharyya distance to every original acquisition", ...
                'motionSource',"separately supplied cumulative wheel/gyro odometry", ...
                'informationCalibrated',false,'dimension',2);
            details.motionSegment=motionSegment;details.historyTransportApplied=numel(history.time)>1;
        end

        %% Overlap support geometry
        function model=prepareSemanticOverlapGeometry(fixedCloud,movingCloud,pose,cfg)
        % prepareSemanticOverlapGeometry Prepare support axes for landmark overlap.
        % Means are relative to the current prediction. This prepares mixture geometry;
        % it does not choose correspondences or construct a pose optimization problem.
            arguments
                fixedCloud (1,1) struct
                movingCloud (1,1) struct
                pose (1,3) double {mustBeReal,mustBeFinite}
                cfg (1,1) struct = semanticGeometryConfig()
            end
            [f,m,height]=localizationSupport.prepareSemanticRegistration(fixedCloud,movingCloud,cfg);
            m.quality=overlapGeometryQuality(m);
            f.viewReliability=ones(f.numComponents,1);
            if isfield(fixedCloud.components,'viewReliability')
                reliability=double(fixedCloud.components.viewReliability(:));
                assert(numel(reliability)==f.numComponents && isreal(reliability) && ...
                    all(isfinite(reliability)&reliability>=0&reliability<=1), ...
                    'VehicleLocalization:InvalidViewReliability','Map view reliability must be in [0,1].');
                f.viewReliability=reliability;
            end
            m.temporalStability=ones(m.numComponents,1);
            if isfield(movingCloud.components,'temporalStability')
                m.temporalStability=double(movingCloud.components.temporalStability(:));
                assert(numel(m.temporalStability)==m.numComponents && isreal(m.temporalStability) && ...
                    all(isfinite(m.temporalStability)&m.temporalStability>=0&m.temporalStability<=1), ...
                    'VehicleLocalization:InvalidTemporalStability','Temporal stability must be in [0,1].');
                m.quality(m.temporalStability==0)=0;
            end
            m.planarCovariance=movingCloud.components.covariance(1:2,1:2,:);
            [f,m]=overlapGeometrySupportGeometry(f,m,cfg.support);
            f.planarCovariance=f.covariance(1:2,1:2,:);
            f.mean(:,1:2)=f.mean(:,1:2)-pose(1:2);
            model=struct('fixed',f,'moving',m,'height',height,'origin',pose(1:2));
        end
    end
end

% Alignment and measurement support: local helpers
function [a,b,w,ok]=fullBrackets(source,target,maximum)
    source=source(:);assert(numel(source)>=2 && all(isfinite(source)) && all(diff(source)>0), ...
        'VehicleLocalization:InvalidSyncClock','Need an increasing finite source clock.');
    a=ones(size(target));b=a;w=zeros(size(target));ok=false(size(target));j=1;
    for k=1:numel(target)
        while j<numel(source) && source(j+1)<=target(k),j=j+1;end
        a(k)=j;b(k)=min(j+1,numel(source));
        if source(j)==target(k),b(k)=j;ok(k)=true;
        elseif source(j)<target(k) && b(k)>j && source(b(k))>=target(k) && source(b(k))-source(j)<=maximum
            w(k)=(target(k)-source(j))/(source(b(k))-source(j));ok(k)=true;
        end
    end
end

function result=fullBlend(values,a,b,w)
    result=(1-w).*values(a,:)+w.*values(b,:);
end

% Intrinsic observer and gain synthesis: local helpers
function m=invariantModel(cfg,storedGyro)
    z=cfg.invariantObserver;s=cfg.iss;ell=z.headingLengthM;sigma=z.sigma;n=z.noise;
    r=z.fusion;
    validateattributes(z.nominalGnssWeight,{'double'},{'real','finite','nonnegative','scalar','<=',1});
    validateattributes(r.minimumCourseSpeed,{'double'},{'real','finite','positive','scalar','<=',s.maximumSpeed});
    validateattributes([s.maximumAcceleration,cfg.maximumTrackAngleRate,r.lidarOperatorMismatch], ...
        {'double'},{'real','finite','nonnegative'});
    validateattributes(z.motionResponse.timeConstantS,{'double'},{'real','finite','positive','scalar'});
    validateattributes(z.motionResponse.enabled,{'logical'},{'scalar'});
    assert(z.theta==z.sigma && z.lambda>0 && z.margin>0, ...
        'VehicleLocalization:InvalidFusionDomain','Require theta=sigma and positive LMI margins.');
    assert(min([s.minimumPositionStrength,s.minimumHeadingStrength])> ...
        s.maximumPositionHeadingCoupling+r.lidarOperatorMismatch, ...
        'VehicleLocalization:InvalidFusionDomain','The nominal fusion LiDAR lower bound must be positive.');
    m.motionAvailable=z.velocityAvailable && z.accelerationAvailable;
    if isfield(z,'headingGainLimits')
        validateattributes(z.headingGainLimits,{'double'},{'real','finite','positive','numel',2});
    end
    if isfield(z,'matchedGainLimits')
        validateattributes(z.matchedGainLimits,{'double'},{'real','finite','positive','numel',3});
    end
    assert(z.noiseObjective=="normalizedFullState" && isequal(cfg.lidar.poseScales(:),[1;1;1/ell]), ...
        'VehicleLocalization:InvalidInvariantDesign','Require the declared full-state objective and consistent pose scaling.');
    if nargin<2,storedGyro=invariantGyroDesign(z);end
    ratio=z.yawAccelerationRadps2/z.noise.gyroRadps;g=storedGyro.physicalGain;q=storedGyro.normalizedVarianceProxy;
    check=[q,ratio,g;ratio,2*g,0;g,0,2*g];
    assert(g>0 && min(eig(check))>=-1e-6*max(1,norm(check,2)), ...
        'VehicleLocalization:InvalidInvariantDesign','Stored scalar gyro LMI does not match current noise assumptions.');
    m.gyroDesign=storedGyro;m.gyroGainNormalized=g/z.sigma;
    m.A=zeros(8);m.A(1,2)=1;m.A(2,3)=1;m.A(4,5)=1;m.A(5,6)=1;m.A(7,8)=1;
    m.C=zeros(3,8);m.C(1,1)=1;m.C(2,4)=1;m.C(3,7)=1;m.Cp=m.C(1:2,:);
    m.B=zeros(8,3);m.B(3,1)=1;m.B(6,2)=1;m.B(8,3)=1;m.e7=zeros(8,1);m.e7(7)=1;m.e8=zeros(8,1);m.e8(8)=1;
    m.e1=zeros(8,1);m.e1(1)=1;m.e2=zeros(8,1);m.e2(2)=1;m.e5=zeros(8,1);m.e5(5)=1;
    m.DL=diag([n.lidarPositionM,n.lidarPositionM,ell*n.lidarHeadingRad]);m.DG=n.gnssPositionM*eye(2);
    D=localizationSupport.outputNormalization(cfg,sigma);
    E=diag([ones(1,6),ell,ell]);T=diag(sigma.^[1,2,3,1,2,3,1,2]);
    speedPoints=unique([r.minimumCourseSpeed,(r.minimumCourseSpeed+s.maximumSpeed)/2,s.maximumSpeed]);
    accelerationPoints=s.maximumAcceleration*[0,1,-1,0,0;0,0,0,1,-1];
    m.motion=struct('H',{},'DI',{},'scheduleFactor',{});
    for speed=speedPoints
        for j=1:size(accelerationPoints,2)
            a=accelerationPoints(:,j);nominal=zeros(8,1);nominal(2)=speed;nominal([3,6])=a;
            N=localizationSupport.measurementSecant([speed;0],a,[speed;0],a);
            N=N*diag([n.velocityMps,n.velocityMps,n.accelerationMps2,n.accelerationMps2,n.gyroRadps]);
            % One primitive sample remains one correlated input column.
            N(:,6)=0;N(4,6)=n.velocityMps;
            [~,H]=localizationSupport.outputs(nominal,0,cfg);
            m.motion(end+1)=struct('H',D*H*(E\T),'DI',D*N, ...
                'scheduleFactor',s.maximumSpeed^2/(r.minimumCourseSpeed^2+speed^2));
        end
    end
    m.Dd=diag([n.jerkMps3/sigma^3,n.jerkMps3/sigma^3,ell*z.yawAccelerationRadps2/sigma^2]);
    m.S=diag([s.minimumPositionStrength*[1,1],s.minimumHeadingStrength]);
end

function [cost,constraints]=invariantNoiseEnergy(G,P)
% Sum primitive-column energies without splitting correlated output effects.
    q=sdpvar(size(G,2),1);constraints=[];
    for j=1:size(G,2)
        constraints=[constraints,[q(j),G(:,j).';G(:,j),P]>=0]; %#ok<AGROW>
    end
    cost=sum(q);
end

function d=invariantGyroDesign(z)
% q >= ratio^2/(2*g)+g/2 is minimized analytically at g=q=ratio.
% Eliminates the independent scalar SDP without changing its optimum.
    ratio=z.yawAccelerationRadps2/z.noise.gyroRadps;
    validateattributes(ratio,{'double'},{'real','finite','positive','scalar'});
    gain=max(ratio,z.margin);bound=ratio^2/(2*gain)+gain/2;
    B=[bound,ratio,gain;ratio,2*gain,0;gain,0,2*gain];
    d=struct('physicalGain',gain,'normalizedVarianceProxy',bound, ...
        'minimumLmiEigenvalue',min(eig(B)),'analyticOptimum',ratio,'method',"analytic-scalar-lmi");
end

function v=invariantFeedbackAuthority(d,cfg)
    inverse=d.P\eye(8);z=cfg.invariantObserver;
    v=struct('verified',true,'positionL',d.kappaL*inverse(1,1), ...
        'positionG',d.kappaG*inverse(1,1), ...
        'velocityDirection',d.auxiliaryWeights(3)*inverse(5,5), ...
        'headingL',d.kappaL*inverse(7,7),'headingDirection',d.auxiliaryWeights(3)*inverse(7,7));
    speedFactor=(z.fusion.minimumCourseSpeed/cfg.iss.maximumSpeed)^2;
    % Report individual contributions separately from the coupled response LMI.
    k=d.auxiliaryWeights;
    if isfield(d,'motionGainSchedule'),k=localizationSupport.weights(d,z.fusion.minimumCourseSpeed^2);end
    v.minimumDirectRatesPerSecond=z.theta*speedFactor*k(1:2).'.*[inverse(2,2),inverse(3,3)];
    tolerance=1e-6;
    if isfield(z,'matchedGainLimits')
        v.verified=v.verified && all([v.positionL,v.positionG,v.velocityDirection]<=z.matchedGainLimits*(1+tolerance));
    end
    if isfield(z,'headingGainLimits')
        v.verified=v.verified && all([v.headingL,v.headingDirection]<=z.headingGainLimits*(1+tolerance));
    end
end

% Common sensor certificate: local helpers
function value=sensorWorstEigenvalue(matrices)
    value=-Inf;
    for k=1:numel(matrices)
        M=matrices{k};value=max(value,max(eig((M+M.')/2)));
    end
end

% LiDAR measurement and correction: local helpers
function [r,J]=injectionLineAndPointResiduals(f,pose)
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

function measurement=injectionLandmarkLikelihoodGradient(local,movingCloud,pose,cfg,measurement)
% Per-landmark mixture likelihood at every ladder scale (semanticLandmarkLikelihood).
% Map masses form one density per class and source masses carry 1/C per class;
% Evidence factors multiply after this normalization. Support geometry supplies
% axes, confidences and angular variances for the optional orientation kernel.
    ladder=measurement.scaleLadder;
    geometry=localizationSupport.prepareSemanticOverlapGeometry(local,movingCloud,pose,cfg.geometry);
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
    angularFloor=NaN;if cfg.orientation,angularFloor=cfg.geometry.support.angularFloor;end
    origin=[0,0,pose(3)];gradient=zeros(3,1);information=zeros(3);
    for k=1:numel(ladder)
        fk=f;mk=m;
        if ladder(k)>0
            inflation=ladder(k)^2/2*eye(2);
            fk.covariance=fk.covariance+inflation;mk.covariance=mk.covariance+inflation;
        end
        [~,g,M,explained]=localizationSupport.semanticLandmarkLikelihood(fk,mk,origin,cfg.outlierDensity,angularFloor);
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

% Semantic mixture geometry: local helpers
function available=geometryHasHeight(components)
    available=size(components.mean,2)==3 || ...
        (isfield(components,'heightAvailable') && all(components.heightAvailable));
end

function [energy, gradient] = geometrySpatialGaussianOverlap(fixed, moving, pose)
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

function S=geometryMomentMatch(covariance,means,mu,w)
% Weighted within-plus-between scatter about mu over the leading dimensions.
    d=size(covariance,1);S=zeros(d);
    for k=1:numel(w)
        delta=(means(k,1:d)-mu(1:d)).';S=S+w(k)*(covariance(:,:,k)+delta*delta.');
    end
    S=S/sum(w);S=(S+S.')/2;
end

% Sensor tilt and extrinsic calibration: local helpers
function residual=calibrationHeightResidual(pairs,angle)
    correction=calibrationPitchRotation(angle); residual=zeros(numel(pairs),1);
    for j=1:numel(pairs)
        p=pairs{j};
        difference=p.moving*correction.'*p.movingRotation(3,:).'- ...
            p.reference*correction.'*p.referenceRotation(3,:).'+p.heightDifference;
        residual(j)=median(difference);
    end
end

function rotation=calibrationPitchRotation(angle)
    rotation=[cos(angle) 0 -sin(angle);0 1 0;sin(angle) 0 cos(angle)];
end

function [position,rotation]=calibrationPoseGeometry(row)
    [pose,tilt,z]=localizationSupport.poseRowToPlanarPose(row);
    yaw=[cos(pose(3)) -sin(pose(3)) 0;sin(pose(3)) cos(pose(3)) 0;0 0 1];
    position=[pose(1:2),z]; rotation=yaw*tilt;
end

function R=calibrationRotation(yaw)
    R=[cos(yaw),-sin(yaw);sin(yaw),cos(yaw)];
end

% Common LMI constraint generation: local helpers
function selected=windowSelectHeightEvidence(evidence,source,target,n)
    selected=struct('mean',zeros(n,3),'covariance',zeros(3,3,n), ...
        'available',false(n,1),'scope',"currentAcquisitionOnly");
    selected.mean(target,:)=evidence.mean(source,:);
    selected.covariance(:,:,target)=evidence.covariance(:,:,source);
    selected.available(target)=evidence.available(source);
end

function value=windowFieldOrUnit(c,name)
    value=ones(c.numComponents,1);
    if isfield(c,name),value=c.(name)(:);end
end

function [c,statistics,currentIndices,currentStability,pooledIndices]=windowStableTracks(sets,cfg)
% Associate temporally, with no map, ring, reference pose or raw-point access.
    total=sum(cellfun(@(s)size(s.mean,1),sets));horizon=numel(sets);
    meanXY=zeros(total,2);scatter=zeros(2,2,total);names=strings(total,1);
    count=zeros(total,1);semantic=zeros(total,1);occupancy=zeros(total,1);
    support=false(total,horizon);tracks=0;ownership=cell(horizon,1);
    positionPairs=0;shapeRejectedPairs=0;
    for frame=1:horizon
        observation=sets{frame};n=size(observation.mean,1);assignment=zeros(n,1);
        % Compare one semantic class at a time. A track and an observation
        % can each occur in only one selected pair for this acquisition.
        for name=unique(observation.semanticName).'
            a=find(names(1:tracks)==name);b=find(observation.semanticName==name & observation.mixtureWeight>0);
            if isempty(a)||isempty(b),continue;end
            dx=meanXY(a,1)-observation.mean(b,1).';dy=meanXY(a,2)-observation.mean(b,2).';
            ca=scatter(:,:,a);cb=observation.covariance(:,:,b);
            xx=reshape(ca(1,1,:),[],1)+reshape(cb(1,1,:),1,[])+cfg.associationNoiseStandardDeviation^2;
            xy=reshape(ca(1,2,:),[],1)+reshape(cb(1,2,:),1,[]);
            yy=reshape(ca(2,2,:),[],1)+reshape(cb(2,2,:),1,[])+cfg.associationNoiseStandardDeviation^2;
            distance=(yy.*dx.^2-2*xy.*dx.*dy+xx.*dy.^2)./(xx.*yy-xy.^2);
            positionCompatible=dx.^2+dy.^2<=cfg.maximumAssociationDistance^2 & distance<=cfg.maximumStandardizedDistance^2;
            shape=zeros(size(distance));
            if isfinite(cfg.maximumShapeDistance)
                lookup=zeros(tracks,1);lookup(a)=1:numel(a);
                for acquisition=1:frame-1
                    previous=find(ownership{acquisition}>0);
                    rows=lookup(ownership{acquisition}(previous));previous=previous(rows>0);rows=rows(rows>0);
                    if isempty(rows),continue;end
                    d=windowCovarianceShapeDistance(sets{acquisition}.covariance(:,:,previous),cb,cfg.shapeVarianceFloor);
                    shape(rows,:)=max(shape(rows,:),d);
                end
            end
            positionPairs=positionPairs+nnz(positionCompatible);
            shapeRejectedPairs=shapeRejectedPairs+nnz(positionCompatible & shape>cfg.maximumShapeDistance);
            eligible=find(positionCompatible & shape<=cfg.maximumShapeDistance);
            % Scale the Gaussian shape term to the center-distance score;
            % positional odometry noise remains separate from shape scatter.
            score=distance+4*shape;
            [~,order]=sort(score(eligible));eligible=eligible(order);
            used=false(numel(a),1);
            for candidate=1:numel(eligible)
                index=eligible(candidate);
                [i,j]=ind2sub(size(distance),index);
                if ~used(i)&&assignment(b(j))==0
                    assignment(b(j))=a(i);used(i)=true;
                end
            end
        end
        for j=find(observation.mixtureWeight>0).'
            id=assignment(j);
            if id==0,tracks=tracks+1;id=tracks;names(id)=observation.semanticName(j);end
            previous=count(id);count(id)=previous+1;fraction=1/count(id);
            delta=observation.mean(j,:)-meanXY(id,:);
            % Within-scan scatter plus between-scan disagreement; never divide
            % the scatter by frame count as though scans were independent.
            scatter(:,:,id)=(1-fraction)*scatter(:,:,id)+fraction*observation.covariance(:,:,j) ...
                +fraction*(1-fraction)*(delta.'*delta);
            meanXY(id,:)=meanXY(id,:)+fraction*delta;
            semantic(id)=semantic(id)+fraction*(observation.semanticProbability(j)-semantic(id));
            occupancy(id)=occupancy(id)+fraction*(observation.occupancyProbability(j)-occupancy(id));
            support(id,frame)=true;
            assignment(j)=id;
        end
        ownership{frame}=assignment;
    end
    currentIndices=find(assignment>0);
    currentIndices=currentIndices(count(assignment(currentIndices))>=cfg.minimumDetectionFrames);
    currentStability=count(assignment(currentIndices))/cfg.maximumFrames;
    keep=count>=cfg.minimumDetectionFrames;stability=count(keep)/cfg.maximumFrames;
    lookup=zeros(total,1);lookup(keep)=1:nnz(keep);
    pooledIndices=lookup(assignment(currentIndices));
    mass=semantic(keep).*occupancy(keep).*stability;
    c=struct('mean',meanXY(keep,:),'covariance',scatter(:,:,keep), ...
        'semanticName',names(keep),'mixtureWeight',mass/max(sum(mass),realmin), ...
        'semanticProbability',semantic(keep),'occupancyProbability',occupancy(keep), ...
        'detectionFrameCount',count(keep),'detectionFrameMask',support(keep,:), ...
        'temporalStability',stability,'numComponents',nnz(keep));
    minimumSupport=0;meanStability=0;
    if any(keep),minimumSupport=min(count(keep));meanStability=mean(stability);end
    statistics=struct('inputComponents',total,'tracks',tracks,'singletons',nnz(count==1), ...
        'rejected',tracks-nnz(keep),'minimumSupport',minimumSupport,'meanStability',meanStability);
    statistics.positionPairs=positionPairs;statistics.shapeRejectedPairs=shapeRejectedPairs;
end

function distance=windowCovarianceShapeDistance(a,b,varianceFloor)
% Covariance-only Bhattacharyya distance, invariant to a common XY rotation.
% Add the same small spatial floor; do not normalize away real scale changes.
    ax=reshape(a(1,1,:),[],1)+varianceFloor;ay=reshape(a(2,2,:),[],1)+varianceFloor;
    ac=reshape(a(1,2,:),[],1);
    bx=reshape(b(1,1,:),1,[])+varianceFloor;by=reshape(b(2,2,:),1,[])+varianceFloor;
    bc=reshape(b(1,2,:),1,[]);
    determinant=((ax+bx).*(ay+by)-(ac+bc).^2)/4;
    logRatio=log(determinant)-.5*(log(ax.*ay-ac.^2)+log(bx.*by-bc.^2));
    distance=max(0,.5*logRatio);
end

% Overlap support geometry: local helpers
function q=overlapGeometryQuality(c)
    q=ones(c.numComponents,1);
    if isfield(c,'semanticProbability'), q=double(c.semanticProbability(:)); end
    if isfield(c,'occupancyProbability'), q=q.*double(c.occupancyProbability(:)); end
    if isfield(c,'supportAmplitude')
        q=double(c.supportAmplitude(:));
        assert(numel(q)==c.numComponents && all(isfinite(q)&q>=0), ...
            'VehicleLocalization:InvalidSemanticQuality','Invalid support amplitude.');
        for name=unique(c.semanticName).'
            keep=c.semanticName==name; q(keep)=q(keep)/max(max(q(keep)),eps);
        end
    end
    assert(numel(q)==c.numComponents && all(isfinite(q)&q>=0&q<=1), ...
        'VehicleLocalization:InvalidSemanticQuality','Invalid semantic/occupancy probability.');
    q(c.mixtureWeight<=0)=0;
end

function [f,m]=overlapGeometrySupportGeometry(f,m,cfg)
% supportGeometry Estimate geometric support without class rules.
% Neighborhood moments supply the axes used by the optional orientation kernel.
    values=[cfg.angularFloor,cfg.scatterScale,cfg.neighborhoodRadius, ...
        cfg.minimumSpan,cfg.directionPower,cfg.maximumGap];
    assert(isreal(values)&&all(isfinite(values)&values>0)&&cfg.angularFloor<pi/2 && ...
        isscalar(cfg.directionResolution)&&isreal(cfg.directionResolution)&& ...
        isfinite(cfg.directionResolution)&&cfg.directionResolution>=0, ...
        'VehicleLocalization:InvalidSupportGeometry','Require finite positive support geometry scales.');
    count=f.numComponents;
    if isfield(f,'intrinsicCovariance')
        C=f.intrinsicCovariance;
        assert(isnumeric(C)&&isreal(C)&&size(C,1)==2&&size(C,2)==2&&size(C,3)==count && ...
            all(isfinite(C),'all')&&all(abs(C-permute(C,[2 1 3]))<1e-9,'all'), ...
            'VehicleLocalization:InvalidIntrinsicShape','Require aligned finite symmetric intrinsic scatter.');
    end
    f.orientationNormal=zeros(count,2);f.axisConfidence=zeros(count,1);
    f.angularVariance=zeros(count,1);
    for k=1:count
        C=f.covariance(1:2,1:2,k);
        if isfield(f,'intrinsicCovariance') && any(f.intrinsicCovariance(:,:,k),'all')
            C=f.intrinsicCovariance(:,:,k);
        end
        [~,~,~,intrinsicConfidence]=overlapGeometryGeometry(C);
        orientation=overlapGeometryOrientationMoment(f,k,C,cfg);
        [axis,minor,major,confidence]=overlapGeometryGeometry(orientation);
        f.orientationNormal(k,:)=[-axis(2),axis(1)];
        f.axisConfidence(k)=intrinsicConfidence*confidence^(cfg.directionPower/2)*major/(major+cfg.directionResolution^2);
        f.angularVariance(k)=cfg.scatterScale^2*minor/major;
    end
    count=m.numComponents;m.supportTangent=zeros(count,2);
    m.axisConfidence=zeros(count,1);m.angularVariance=zeros(count,1);
    for k=1:count
        C=m.planarCovariance(:,:,k);
        C=overlapGeometryOrientationMoment(m,k,C,cfg);
        [axis,minor,major,confidence]=overlapGeometryGeometry(C);
        m.supportTangent(k,:)=axis;m.axisConfidence(k)=confidence^(cfg.directionPower/2)*major/(major+cfg.directionResolution^2);
        m.angularVariance(k)=cfg.scatterScale^2*minor/major;
    end
end

function [axis,minor,major,confidence]=overlapGeometryGeometry(C)
    [v,e]=eig((C+C.')/2,'vector');[major,k]=max(e);minor=max(min(e),1e-6);
    assert(min(e)>=-1e-10,'VehicleLocalization:InvalidIntrinsicShape','Scatter must be positive semidefinite.');
    major=max(major,minor);axis=v(:,k).';
    confidence=((major-minor)/(major+minor))^2;
end

function C=overlapGeometryOrientationMoment(c,k,C,cfg)
% Use the same neighborhood rule for map and source component axes.
    if 12*max(eig(C))>=cfg.minimumSpan^2,return;end
    eligible=c.mixtureWeight>0;
    if isfield(c,'quality'),eligible=eligible & c.quality>0;end
    if isfield(c,'viewReliability'),eligible=eligible & c.viewReliability>0;end
    ids=c.semanticName==c.semanticName(k) & eligible & ...
        sum((c.mean(:,1:2)-c.mean(k,1:2)).^2,2)<=cfg.neighborhoodRadius^2+64*eps(max(1,cfg.neighborhoodRadius^2));
    points=unique(c.mean(ids,1:2),'rows');
    delta=permute(points,[1 3 2])-permute(points,[3 1 2]);
    links=sum(delta.^2,3)<=cfg.maximumGap^2+64*eps(max(1,cfg.maximumGap^2));
    [~,anchor]=min(sum((points-c.mean(k,1:2)).^2,2));
    reached=false(size(points,1),1);reached(anchor)=true;
    while true
        expanded=any(links(:,reached),2);
        if isequal(expanded,reached),break;end
        reached=expanded;
    end
    points=points(reached,:);
    if size(points,1)>=3
        centered=points-mean(points,1);neighborhood=centered.'*centered/size(points,1);
        [axis,~,~,~]=overlapGeometryGeometry(neighborhood);span=centered*axis.';
        if max(span)-min(span)>=cfg.minimumSpan-64*eps(max(1,cfg.minimumSpan)),C=neighborhood;end
    end
end
