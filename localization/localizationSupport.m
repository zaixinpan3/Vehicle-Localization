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
            localizationSupport.rejectExternalLidar(data);
            assert(isfield(data,'lidarOverlap'),'VehicleLocalization:MissingOverlapSource', ...
                'Provide the scan clock and source clouds in lidarOverlap.');
            [aligned,lateral,metadata,selected]=observerInternal.alignFrameClock(data,lateralInput,cfg,data.lidarOverlap.time(:));
            aligned.lidarOverlap=data.lidarOverlap;aligned.lidarOverlap.time=aligned.highRate.time;
            aligned.lidarOverlap.sources=data.lidarOverlap.sources(selected);
            if isfield(data.lidarOverlap,'valid'),aligned.lidarOverlap.valid=data.lidarOverlap.valid(selected);end
            for name=["frameReliability","directionReliability"]
                if isfield(data.lidarOverlap,name)
                    aligned.lidarOverlap.(name)=data.lidarOverlap.(name)(selected,:);
                end
            end
        end

        function rejectExternalLidar(data)
            if isfield(data,'lidar')
                assert(~isfield(data,'lidarOverlap'),'VehicleLocalization:AmbiguousLidarInput', ...
                    'Supply only the map and scan clouds in lidarOverlap.');
                assert(~any(isfield(data.lidar,{'residualModels','poseErrorVariance'})), ...
                    'VehicleLocalization:ArchivedRegistrationMeasurement','Frozen registration inputs are retired.');
                error('VehicleLocalization:ExternalLidarPose','Provide a map and scan clouds for LiDAR feedback.');
            end
            if isfield(data,'lidarOverlap')
                assert(~any(isfield(data.lidarOverlap,{'pose','evaluateResidual','gradient','residual'})), ...
                    'VehicleLocalization:ExternalLidarPose','Provide feature clouds for LiDAR feedback.');
            end
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
            kept=4-filter.rank:3;
            strengths=filter.strengths(kept);threshold=1e-10;
            active=strengths>threshold;dimension=nnz(active);
            B=diag(sqrt(strengths(active)))*filter.eigenvectors(:,kept(active)).';
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

        function packet=headingMeasurements(data,t,cfg)
        % Independent receiver heading in radians; no reliability estimate is required.
        % Interpolate the shortest arc; reject invalid endpoints and long gaps.
            t=t(:);n=numel(t);
            packet=struct('time',t,'heading',nan(n,1), ...
                'valid',false(n,1),'delay',0,'waitSeconds',zeros(n,1));
            if isfield(data,'gnss') && isfield(data.gnss,'heading')
                assert(~isfield(data,'gnssHeading'),'VehicleLocalization:AmbiguousHeading','Supply one heading stream.');
                h=data.gnss;
            elseif isfield(data,'gnssHeading')
                h=data.gnssHeading;
            else
                return
            end
            assert(all(isfield(h,{'time','heading','valid','delay'})) ...
                && isequal(h.delay,0),'VehicleLocalization:InvalidHeadingInput', ...
                'Supply independent heading, validity, time and zero delay.');
            time=h.time(:);angle=h.heading(:);valid=h.valid(:);
            assert(~isempty(time) && all(isfinite(time)) && all(diff(time)>0) ...
                && numel(angle)==numel(time) ...
                && numel(valid)==numel(time) && all(ismember(valid,[0,1])), ...
                'VehicleLocalization:InvalidHeadingInput','Invalid independent heading packet dimensions.');
            valid=logical(valid);
            assert(isreal(angle) && all(isfinite(angle(valid))), ...
                'VehicleLocalization:InvalidHeadingInput','Valid heading requires finite radians.');
            maximumBracket=cfg.synchronization.gnssMaximumBracket;
            if isfield(h,'maximumBracketSeconds'),maximumBracket=h.maximumBracketSeconds;end
            validateattributes(maximumBracket,{'numeric'},{'real','finite','positive','scalar'});
            [a,b,w,ok]=observerInternal.frameBrackets(time,t,maximumBracket);
            ok=ok & valid(a) & valid(b);
            for k=find(ok).'
                delta=atan2(sin(angle(b(k))-angle(a(k))),cos(angle(b(k))-angle(a(k))));
                packet.heading(k)=atan2(sin(angle(a(k))+w(k)*delta),cos(angle(a(k))+w(k)*delta));
                packet.waitSeconds(k)=time(b(k))-t(k);
            end
            packet.valid=ok;
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
            if isempty(information),return;end
            covariance=information\eye(2)+R*C*R.'+alignment.headingStdRad^2*(R*J*offset)*(R*J*offset).';
            information=covariance\eye(2);information=(information+information.')/2;
        end

        %% Intrinsic observer and gain synthesis
        function [next,r]=predict(state,dt,~)
        % Body yaw rate is an internal state, constant during the intrinsic step.
            r=state(8);
            A=zeros(8);A(1,2)=1;A(2,3)=1;A(4,5)=1;A(5,6)=1;A(7,8)=1;
            A([3,6],[2,5])=r^2*eye(2);A([3,6],[3,6])=2*r*[0,-1;1,0];
            next=(eye(8)-dt*A)\state;
        end

        function [value,H]=outputs(state,cfg)
            v=state([2,5]);a=state([3,6]);
            cross=v(1)*a(2)-v(2)*a(1);angle=state(7);
            direction=[-sin(angle);cos(angle)];
            % Physical outputs; numerical scales belong to the feedback metric.
            value=[v.'*v;v.'*a;cross;direction.'*v];
            if nargout>1
                [~,H]=localizationSupport.secants(state,state,cfg);
            end
        end

        function D=outputNormalization(cfg,theta)
        % Fixed, invertible numerical scaling; never divides by actual speed.
            if nargin<2,theta=cfg.invariantObserver.theta;end
            V=cfg.iss.maximumSpeed;
            D=diag([1/(2*V),1/V,1/V,1].*theta.^-[2,3,3,2]);
        end

        function packet=measurements(speed,vy,ax,ay,velocityValid,accelerationValid,~)
        % Four additional motion outputs; no independent yaw-rate measurement.
            value=zeros(4,1);valid=[velocityValid;velocityValid&&accelerationValid; ...
                velocityValid&&accelerationValid;velocityValid];
            if valid(1),value(1)=speed^2+vy^2;end
            if valid(2),value(2)=speed*ax+vy*ay;value(3)=speed*ay-vy*ax;end
            if valid(4),value(4)=vy;end
            packet=struct('value',value,'valid',valid);
        end

        function [next,audit]=correct(state,packet,dt,design,cfg)
            [value,H]=localizationSupport.outputs(state,cfg);
            active=logical(packet.valid);
            k=design.auxiliaryWeights;
            speedSquared=0;
            if active(1)
                speedSquared=packet.value(1);
                validateattributes(speedSquared,{'double'},{'real','finite','nonnegative','scalar'});
            end
            k(1:2)=k(1:2)*cfg.iss.maximumSpeed^2/(cfg.invariantObserver.fusion.minimumCourseSpeed^2+speedSquared);
            k=k([1,2,2,3]);D=localizationSupport.outputNormalization(cfg,design.theta);
            C=D(active,active)*H(active,:)*design.toPhysical;
            weighted=C.'.*k(active).';
            residual=D(active,active)*(packet.value(active)-value(active));
            % The same weighted Jacobian acts in the innovation and the implicit metric.
            Q=design.theta*weighted*C;
            delta=(design.P+dt*Q)\(dt*design.theta*weighted*residual);
            next=state+design.toPhysical*delta;
            audit=struct('innovation',packet.value-value,'valid',active, ...
                'allAuxiliaryOutputsAvailable',all(active),'predicted',value,'normalizedCorrection',delta);
        end

        function inputs=invariantInputs(cfg)
        % Version every mathematical design input; reject retired designs.
            z=cfg.invariantObserver;
            assert(~isfield(cfg.lidar,'gainInformationScale') && ~isfield(z,'predictionMinimumSpeedMps'), ...
                'VehicleLocalization:StaleInvariantDesign','Rebuild the geometry-weighted body-rate configuration.');
            assert(~any(isfield(z,{'gnssPositionAvailable','gnssHeadingAvailable','velocityAvailable','accelerationAvailable'})), ...
                'VehicleLocalization:ObsoleteSensorAvailability','Regenerate the config and gains. Set validity on measurement packets.');
            assert(isfield(z,'motionTimeConstantS'),'VehicleLocalization:StaleInvariantDesign', ...
                'Regenerate the observer configuration for the matched motion update.');
            assert(~isfield(cfg,'gainSelection') && ~any(isfield(z, ...
                {'redundancy','regionalRadiusCap','maximumMotionPositionNoiseRatio','motionResponse','matchedGainLimits','headingGainLimits','balancedPoseMetric'})), ...
                'VehicleLocalization:ObsoleteFusionDesign', ...
                'Regenerate the nominal fusion configuration; legacy searches and certificates are removed.');
            assert(~isfield(z,'yawRateGainPerSecond') && isfield(z.noise,'upstreamRateRadps'), ...
                'VehicleLocalization:StaleInvariantDesign','Remove the prescribed yaw-rate gain and regenerate the shared design.');
            if isfield(z,'provenance'),z=rmfield(z,'provenance');end
            inputs=jsondecode(jsonencode(struct('architecture',"unit-lidar-metric-v27", ...
                'physicalOutputs',"squared-dot-cross-body-lateral", ...
                'observer',z,'iss',cfg.iss,'bodyYawRateBound',cfg.maximumBodyYawRate, ...
                'gnssBodyOffset',cfg.gnss.outputPoint.bodyOffset(:).','poseScales',cfg.lidar.poseScales(:).')));
        end

        function design=synthesize(cfg)
        % Solve the pose/rate certificate, then synthesize matched motion feedback in its metric.
            inputs=localizationSupport.invariantInputs(cfg);
            assert(exist('sdpvar','file')==2,'VehicleLocalization:MissingSolver','Add YALMIP and SeDuMi before synthesis.');
            m=invariantModel(cfg);z=cfg.invariantObserver;
            yalmip('clear');
            Pc=sdpvar(3,3);Ph=diag(sdpvar(2,1));P=blkdiag(Pc,Pc,Ph);
            kR=sdpvar(1);certificateFloor=sdpvar(1);
            kp=localizationSupport.gnssPositionGain(cfg);
            % Hypograph of min(1, eigmin(P), kR/margin, vertex decay / required
            % margin). Maximizing this floor per unit noise preserves gain selection.
            constraints=[certificateFloor>=0,certificateFloor<=1,P>=certificateFloor*eye(8),kR>=z.margin*certificateFloor];
            % Preserve the fixed position-chain direction in a common metric.
            positionGain=Pc*kp;
            constraints=[constraints,positionGain(2:3)==0,positionGain(1)>=0];
            candidate=struct('P',P,'kappaR',kR);
            fusion=localizationSupport.fusionBlocks(candidate,cfg,(2*z.lambda+z.margin)*certificateFloor);
            for j=1:numel(fusion)
                constraints=[constraints,fusion{j}<=0]; %#ok<AGROW>
            end
            % Nominal pose-equivalent noise proxy for gain selection. The local ISS
            % bound separately requires a bounded actual filtered gradient bias.
            PG=-m.C.'*m.S*m.DL;
            [cost,blocks]=invariantNoiseEnergy(PG,P);constraints=[constraints,blocks];
            [bound,blocks]=invariantNoiseEnergy(P*m.B*m.Dd,P);
            constraints=[constraints,blocks];cost=cost+bound;
            % Exact squared H2 energy of the scalar rate error in physical
            % time, measured in the same normalized-state metric. Its pole
            % is -g and its disturbance amplitudes are q and g*n. With
            % g=theta*kR/P88, the epigraph is a quadratic-over-linear LMI.
            rateNoise=z.noise.upstreamRateRadps;
            accelerationScale=max(z.yawAccelerationRadps2,z.theta*rateNoise);
            weightedRate=z.theta*kR*rateNoise/accelerationScale;
            processWeight=Ph(2,2)*z.yawAccelerationRadps2/accelerationScale;
            rateSlack=sdpvar(1);
            constraints=[constraints,[rateSlack,processWeight;processWeight,weightedRate]>=0];
            rateEnergy=0.5*(z.headingLengthM/z.theta^2)^2 * ...
                accelerationScale*rateNoise*(rateSlack+weightedRate);
            cost=cost+rateEnergy;
            options=sdpsettings('solver',char(z.solver),'verbose',0);
            if z.solver=="sedumi",options.sedumi.stepdif=2;end
            % Fractional programming: each subproblem is an SDP in the
            % unit-LiDAR metric. At the optimum certificateFloor - ratio*cost is zero.
            ratio=0;
            for iteration=1:30
                result=optimize(constraints,ratio*cost-certificateFloor,options);
                residual=min(check(constraints));
                assert(ismember(result.problem,[0,4]) && isfinite(residual) && residual>=-1e-6, ...
                    'VehicleLocalization:InfeasibleInvariantDesign','Nominal fusion SDP failed: %s (residual %.9g).',result.info,residual);
                energy=value(cost);metricFloor=value(certificateFloor);
                assert(isfinite(energy) && energy>0 && isfinite(metricFloor) && metricFloor>0,'VehicleLocalization:InfeasibleInvariantDesign', ...
                    'The normalized fusion design needs positive noise energy and certificate floor.');
                nextRatio=metricFloor/energy;
                converged=abs(nextRatio-ratio)<=1e-7*nextRatio;
                if converged,break;end
                ratio=nextRatio;
            end
            assert(converged, ...
                'VehicleLocalization:InfeasibleInvariantDesign','Noise per unit certificate floor did not converge.');
            Pv=value(P);Pv=(Pv+Pv.')/2;
            [K,headingBandwidth]=localizationSupport.linearGain(cfg,Pv);
            design=struct('architecture',"unit-lidar-metric-v27",'sigma',z.sigma,'P',Pv, ...
                'K',K,'gnssHeadingBandwidth',headingBandwidth,'gnssNoiseInjectionEnergy',trace(m.DG.'*K.'*Pv*K*m.DG),'kappaR',value(kR),...
                'lambda',z.lambda*metricFloor,'rateNoiseEnergy',value(rateEnergy),'templateNoiseEnergy',energy,'solverStatus',result.problem, ...
                'solverInfo',string(result.info),'inputs',inputs, ...
                'solveAudit',struct('optimizationCalls',iteration+1,'fusionVertices',size(localizationSupport.fusionVertices(cfg),1),'matrixInequalities',numel(fusion), ...
                    'minimumSdpResidual',residual,'referenceUsed',false));
            % Use certificate-floor units for the homogeneous motion SDP.
            design.auxiliaryWeights=metricFloor*localizationSupport.solveMotionWeights(Pv/metricFloor,cfg,options);
            design.verification=localizationSupport.verifyInvariantDesign(design,cfg);
            assert(design.verification.certified,'VehicleLocalization:InfeasibleInvariantDesign', ...
                'Independent matched-motion fusion verification failed (worst fusion %.9g).', ...
                design.verification.worstVertexEigenvalue);
        end

        function v=verifyInvariantDesign(design,cfg)
        % Recompute every vertex numerically; stored solver claims are ignored.
            validateattributes(design.P,{'double'},{'real','finite','size',[8,8]});
            validateattributes(design.auxiliaryWeights,{'double'},{'real','finite','size',[3,1]});
            invariantModel(cfg);z=cfg.invariantObserver;
            P=(design.P+design.P.')/2;
            expected=blkdiag(P(1:3,1:3),P(1:3,1:3),diag(diag(P(7:8,7:8))));
            structure=norm(P-expected,'fro');
            validateattributes(design.kappaR,{'double'},{'real','finite','scalar'});
            blocks=localizationSupport.fusionBlocks(design,cfg,0);rawWorst=sensorWorstEigenvalue(blocks);
            worst=rawWorst+design.lambda;
            floorBound=min([1,min(eig(P)),design.kappaR/z.margin,-rawWorst/(2*z.lambda+z.margin)]);
            floorVerified=design.lambda>0 && design.lambda/z.lambda<=floorBound+1e-7;
            g=z.theta*design.kappaR/P(8,8);
            rateEnergy=P(8,8)*(z.headingLengthM/z.theta^2)^2 * ...
                (z.yawAccelerationRadps2^2+(g*z.noise.upstreamRateRadps)^2)/(2*g);
            rateEnergyVerified=isfinite(rateEnergy) && g>0 && isfinite(design.rateNoiseEnergy) && ...
                design.rateNoiseEnergy>=rateEnergy-1e-6*max(1,abs(rateEnergy));
            [motionBlocks,directionRate]=localizationSupport.motionResponseBlocks(P,design.auxiliaryWeights,cfg);
            responseWorst=sensorWorstEigenvalue(motionBlocks);
            responseVerified=responseWorst<=1e-7*max(1,norm(P,2)) && directionRate>=1/z.motionTimeConstantS*(1-1e-6);
            C=eye(8);C=C([1,4,7],:);PK=P*design.K;
            leverRatio=norm(cfg.gnss.outputPoint.bodyOffset)/z.headingLengthM;
            gp=PK(1,1);gh=PK(7,3);Gamma=diag([gp,gp,gh]);
            % The critical Schur block depends only on the antenna norm.
            poseGram=[2*gp,0,gp*leverRatio;0,2*gp,0;gp*leverRatio,0,2*gh];
            poseDefect=max(0,-min(eig(poseGram)))+2*norm(PK-C.'*Gamma,2)*(1+leverRatio);
            fusionVerified=worst+poseDefect<0;
            valid=fusionVerified && responseVerified && rateEnergyVerified && min(eig(P))>0 ...
                && isequal(design.K,localizationSupport.linearGain(cfg,P)) ...
                && all(design.auxiliaryWeights>0) && structure<=1e-10*max(1,norm(P,'fro')) ...
                && design.sigma==z.sigma && isfinite(design.lambda) && floorVerified ...
                && norm(design.P-design.P.','fro')<=1e-10*max(1,norm(P,'fro'));
            scope="Nominal continuous fusion on the declared positive-speed domain with valid upstream yaw rate and informative LiDAR. Each complete GNSS pose adds dissipation in the common metric. The same four drift inequalities cover every arrival schedule and auxiliary subset. Nonlinear local use assumes a compact invariant operating set and sufficiently small errors/disturbances (Bessafa Remark 3). Invariance, a numerical attraction radius, loss of informative LiDAR, actual overlap maps, the lateral cascade and sampling are not certified.";
            v=struct('certified',valid,'nominalFusionLmiVerified',fusionVerified, ...
                'worstVertexEigenvalue',worst,'gnssDissipationDefect',poseDefect,'vertices',size(localizationSupport.fusionVertices(cfg),1),'matrixInequalities',numel(blocks), ...
                'minimumEigenvalueP',min(eig(P)),'maximumEigenvalueP',max(eig(P)), ...
                'intrinsicRateState',true,'zeroSpeedDivision',false,'zeroSpeedCertified',false, ...
                'actualOverlapCertified',false, ...
                'operatingSet',struct('assumedInvariant',true,'invarianceVerified',false, ...
                    'speedInterval',[z.fusion.minimumCourseSpeed,cfg.iss.maximumSpeed], ...
                    'nonlinearConclusionConditional',true), ...
                'nominalFusion',struct('certified',worst<0,'vertices',size(localizationSupport.fusionVertices(cfg),1),'matrixInequalities',numel(blocks), ...
                    'coversGnssAbsent',true, ...
                    'completeGnssPose',true,'coversZeroLidarInformation',false), ...
                'additionalOutputCount',4,'matchedAuxiliaryGain',true,'motionResponseVerified',responseVerified, ...
                'worstMotionResponseEigenvalue',responseWorst, ...
                'yawRateGainPerSecond',g,'rateNoiseEnergy',rateEnergy,'rateEnergyVerified',rateEnergyVerified, ...
                'independentYawRateCorrection',true,'upstreamYawRateRequired',true,'allAuxiliaryOutputsRequired',false, ...
                'scope',scope);
        end

        function runtime=runtime(cfg)
            h=cfg.invariantDesign;theta=cfg.invariantObserver.theta;ell=cfg.invariantObserver.headingLengthM;
            assert(isfield(h,'inputs') && isequaln(h.inputs,localizationSupport.invariantInputs(cfg)), ...
                'VehicleLocalization:StaleInvariantDesign','Temporary gains use different inputs; solve after configuring this run.');
            assert(theta==h.sigma,'VehicleLocalization:InvalidInvariantDesign','The nominal design requires theta=sigma.');
            v=localizationSupport.verifyInvariantDesign(h,cfg);
            assert(v.certified,'VehicleLocalization:InfeasibleInvariantDesign','Temporary design fails matched-motion fusion verification.');
            toPhysical=diag(theta.^[1,2,3,1,2,3,1,2]./[ones(1,6),ell,ell]);
            matched=struct('P',h.P,'T',toPhysical,'theta',theta,'scalingExponents',[1;2;3;1;2;3;1;2], ...
                'metricSource',"Nominal fusion polytope LMI");
            positionHeadingGain=toPhysical*h.K;
            rateGain=zeros(8,1);rateGain(8)=v.yawRateGainPerSecond;
            runtime=struct('kind',cfg.kind,'P',h.P,'theta',theta,'toPhysical',toPhysical, ...
                'auxiliaryWeights',h.auxiliaryWeights, ...
                'LG',positionHeadingGain, ...
                'linearOutputGain',[positionHeadingGain,rateGain], ...
                'lidarMatched',matched,'continuousCertificate',struct('design',h,'verification',v), ...
                'nominalFusionLmiVerified',v.nominalFusionLmiVerified, ...
                'designRoute',"Constant GNSS gain, direct geometric LiDAR feedback and matched motion correction", ...
                'actualOverlapCertified',false,'sampledSystemCertified',false,'scope',v.scope);
        end

        %% Finite nonlinear increment identities
        function [F,H,Hgnss]=secants(x,xhat,cfg)
        % f(x)-f(xhat)=F*(x-xhat), likewise for all nonlinear outputs.
        % These identities are finite differences, including at equal states.
            arguments
                x (8,1) double {mustBeReal,mustBeFinite}
                xhat (8,1) double {mustBeReal,mustBeFinite}
                cfg (1,1) struct
            end
            e=x-xhat;m=(x+xhat)/2;v=m([2,5]);a=m([3,6]);
            J=[0,-1;1,0];half=e(7)/2;
            r=x(8);rh=xhat(8);rm=(r+rh)/2;
            cardinal=1;if half~=0,cardinal=sin(half)/half;end
            c=cos(half);d=[cos(m(7));sin(m(7))];n=J*d;
            F=zeros(8);F(1,2)=1;F(2,3)=1;F(4,5)=1;F(5,6)=1;F(7,8)=1;
            F([3,6],[2,5])=(r^2+rh^2)/2*eye(2);
            F([3,6],[3,6])=2*rm*J;F([3,6],8)=2*rm*v+2*J*a;
            H=zeros(4,8);H(1,[2,5])=2*v.';
            H(2,[2,5])=a.';H(2,[3,6])=v.';
            H(3,[2,5])=(-J*a).';H(3,[3,6])=(J*v).';
            H(4,[2,5])=c*n.';H(4,7)=-cardinal*d.'*v;
            Hgnss=zeros(3,8);Hgnss(:,[1,4,7])=eye(3);
            Hgnss(1:2,7)=cardinal*[d,n]*J*cfg.gnss.outputPoint.bodyOffset(:);
        end

        function N=measurementSecant(v,a,vm,am)
        % Exact packet increment from ONE primitive [dv(2); da(2)] vector.
        % The lateral velocity error is shared with the quadratic outputs.
            arguments
                v (2,1) double {mustBeReal,mustBeFinite}
                a (2,1) double {mustBeReal,mustBeFinite}
                vm (2,1) double {mustBeReal,mustBeFinite}
                am (2,1) double {mustBeReal,mustBeFinite}
            end
            v=(v+vm)/2;a=(a+am)/2;J=[0,-1;1,0];
            N=zeros(4,4);N(1,1:2)=2*v.';
            N(2,1:2)=a.';N(2,3:4)=v.';
            N(3,1:2)=(-J*a).';N(3,3:4)=(J*v).';
            N(4,2)=1;
        end

        %% Nominal fusion coefficient polytope
        function vertices=fusionVertices(cfg)
        % After orthogonal congruence only |r| and the norm of the rate column
        % matter. The chord and three tangents enclose the lifted parabola.
            R=cfg.maximumBodyYawRate;V=cfg.iss.maximumSpeed;A=cfg.iss.maximumAcceleration;
            validateattributes([R,V,A],{'double'},{'real','finite','nonnegative'});
            rate=R*[0;1/4;3/4;1];squaredRate=R^2*[0;0;1/2;1];
            vertices=unique([rate,squaredRate,2*(A+V*rate),zeros(4,1)],'rows','stable');
        end

        function f=fusionFeatures(v,a,r)
        % Physical body-rate column of the Cartesian drift Jacobian.
            c=2*(r*v(:)+[-a(2);a(1)]);
            f=[r,r^2,c.'];
        end

        function A=fusionMatrices(cfg,f)
            t=cfg.invariantObserver.theta;ell=cfg.invariantObserver.headingLengthM;
            A=zeros(8);A(1,2)=1;A(2,3)=1;A(4,5)=1;A(5,6)=1;A(7,8)=1;
            A([3,6],[2,5])=f(2)/t^2*eye(2);A([3,6],[3,6])=2*f(1)/t*[0,-1;1,0];
            A([3,6],8)=f(3:4).'/(ell*t^2);
        end

        function [blocks,directionRate]=motionResponseBlocks(P,k,cfg)
        % The original longitudinal contraction test in the solved primary metric.
            z=cfg.invariantObserver;t=z.theta;s0=z.fusion.minimumCourseSpeed;V=cfg.iss.maximumSpeed;
            idx=2:3;R=P(idx,idx)-P(idx,1)*P(1,idx)/P(1,1);
            A=[0,t;0,0];decay=R*A+A.'*R+2/z.motionTimeConstantS*R;
            limits=cfg.iss.maximumAcceleration/(2*s0*t);
            % Both diagonal damping terms are weakest at s^2/(s0^2+s^2)=1/2.
            blocks={decay-2*t*[k(1)/2,-limits*k(2);-limits*k(2),k(2)/2], ...
                    decay-2*t*[k(1)/2, limits*k(2); limits*k(2),k(2)/2]};
            directionRate=k(3)*((s0+V)/2)^2/(t*z.headingLengthM^2*P(7,7));
        end

        function weights=solveMotionWeights(P,cfg,options)
        % Minimize the historical motion-noise objective with fixed pose/rate gains.
            z=cfg.invariantObserver;t=z.theta;s0=z.fusion.minimumCourseSpeed;V=cfg.iss.maximumSpeed;
            D=localizationSupport.outputNormalization(cfg,t);
            T=diag(t.^[1,2,3,1,2,3,1,2]./[ones(1,6),z.headingLengthM,z.headingLengthM]);
            L=chol(P,'lower');noise=diag([repmat(z.noise.velocityMps,1,2),repmat(z.noise.accelerationMps2,1,2)]);
            accelerations=cfg.iss.maximumAcceleration*[0,1,-1,0,0;0,0,0,1,-1];
            energy=zeros(3);
            for speed=unique([s0,(s0+V)/2,V])
                factor=V^2/(s0^2+speed^2);
                for a=accelerations
                    x=zeros(8,1);x(2)=speed;x([3,6])=a;
                    [~,H]=localizationSupport.outputs(x,cfg);C=D*H*T;
                    N=D*localizationSupport.measurementSecant([speed;0],a,[speed;0],a)*noise;
                    B=[reshape(L\(factor*C(1,:).'*N(1,:)),[],1), ...
                       reshape(L\(factor*C(2:3,:).'*N(2:3,:)),[],1), ...
                       reshape(L\(C(4,:).'*N(4,:)),[],1)];
                    energy=energy+B.'*B;
                end
            end
            k=sdpvar(3,1);
            [blocks,directionRate]=localizationSupport.motionResponseBlocks(P,k,cfg);
            constraints=[k>=z.margin,directionRate>=1/z.motionTimeConstantS];
            for j=1:numel(blocks)
                constraints=[constraints,blocks{j}<=-z.margin*eye(2)]; %#ok<AGROW>
            end
            solved=optimize(constraints,k.'*(energy/max(norm(energy),eps))*k,options);
            weights=value(k);
            assert(ismember(solved.problem,[0,4]) && all(isfinite(weights)) && min(check(constraints))>=-1e-5, ...
                'VehicleLocalization:InfeasibleInvariantDesign','Matched motion response synthesis failed: %s.',solved.info);
        end

        function kp=gnssPositionGain(cfg)
            g=cfg.invariantObserver.gnssBandwidthRatio;
            validateattributes(g,{'double'},{'real','finite','positive','scalar'});
            kp=[3*g;3*g^2;g^3];
        end

        function [K,bandwidth]=linearGain(cfg,P)
        % Minimize the scalar heading-noise proxy subject to complete-pose dissipation.
            z=cfg.invariantObserver;kp=localizationSupport.gnssPositionGain(cfg);
            leverRatio=norm(cfg.gnss.outputPoint.bodyOffset)/z.headingLengthM;
            gammaP=P(1,1:3)*kp;
            lower=z.theta*gammaP*leverRatio^2/(4*P(7,7));
            unconstrained=z.noise.upstreamRateRadps/z.noise.gnssHeadingRad;
            rate=max(unconstrained,lower);
            K=blkdiag(kp,kp,[rate/z.theta;0]);
            bandwidth=struct('perSecond',rate,'unconstrainedPerSecond',unconstrained, ...
                'dissipationFloorPerSecond',lower);
        end

        function blocks=fusionBlocks(d,cfg,margin)
        % A common dissipation bound for the intrinsic flow and any received poses.
            if nargin<3,margin=d.lambda;end
            z=cfg.invariantObserver;E=eye(8);C=E([1,4,7],:);
            S=diag([cfg.iss.minimumPositionStrength*[1,1],cfg.iss.minimumHeadingStrength]) ...
                -(cfg.iss.maximumPositionHeadingCoupling+z.fusion.lidarOperatorMismatch)*eye(3);
            sensor=-2*C.'*S*C-2*d.kappaR*(E(:,8)*E(:,8).');
            vertices=localizationSupport.fusionVertices(cfg);blocks=cell(1,size(vertices,1));
            for j=1:numel(blocks)
                A=localizationSupport.fusionMatrices(cfg,vertices(j,:));M=d.P*A;
                blocks{j}=M+M.'+sensor+margin*eye(8);
            end
        end

        %% LiDAR measurement and correction
        function filter = filterLidarPoseInformation(information,cfg,frameReliability,directionReliability)
        % filterLidarPoseInformation Direct geometric weighting of supported LiDAR directions.
        % INFORMATION is an unregularized physical [X,Y,psi] information surrogate.
        % Direction reliabilities refer to ascending normalized information eigenvalues.
        % Repeated eigenspaces receive their minimum requested reliability, avoiding
        % dependence on the arbitrary eigenvector basis. No count or eigenvalue floor
        % is added to geometric information. PSD rank loss and complete rejection are
        % valid. Reliability must come from association/geometry checks upstream.
            arguments
                information (3,3) double {mustBeReal,mustBeFinite}
                cfg (1,1) struct
                frameReliability (1,1) double {mustBeReal,mustBeFinite} = 1
                directionReliability (3,1) double {mustBeReal,mustBeFinite} = ones(3,1)
            end
            assert(~isfield(cfg.lidar,'errorCalibration'),'VehicleLocalization:ArchivedRegistrationMeasurement', ...
                'Use the geometric information filter; registration-error calibration is removed.');
            scales=cfg.lidar.poseScales(:);
            validateattributes(scales,{'double'},{'real','finite','positive','numel',3});
            assert(~isfield(cfg.lidar,'gainInformationScale'),'VehicleLocalization:ObsoleteLidarRegularizer', ...
                'Regenerate the LiDAR configuration for direct geometry weighting.');
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
            first=1;
            while first<=3
                last=first;
                while last<3 && values(last+1)-values(first)<=64*eps(values(last+1)),last=last+1;end
                directionReliability(first:last)=min(directionReliability(first:last));
                first=last+1;
            end
            rho=frameReliability*directionReliability;
            f=rho.*values;strengths=f.*values;
            assert(all(isfinite(strengths)),'VehicleLocalization:InvalidLidarInformation', ...
                'Squared normalized geometric strengths exceed the floating-point range.');
            F=U*diag(f)*U.';S=U*diag(strengths)*U.';F=(F+F.')/2;
            informationTrace=sum(values);dimension=0;condition=Inf;
            if values(end)>0
                relative=values/values(end);
                dimension=sum(relative)^2/sum(relative.^2);
                if values(1)>0,condition=values(end)/values(1);end
            end
            filter=struct('F',F,'S',(S+S.')/2,'D',D, ...
                'normalizedInformation',U*diag(values)*U.', ...
                'eigenvectors',U,'eigenvalues',values,'reliability',rho, ...
                'strengths',strengths,'rank',nnz(values>tolerance), ...
                'trace',informationTrace,'minimumEigenvalue',values(1), ...
                'conditionNumber',condition,'effectiveDimension',dimension, ...
                'informationInterpretation',"Weighted geometric surrogate; not calibrated covariance");
        end

        function [correction,audit] = computeLidarMatchedCorrection(measurement,pose,G,design,cfg,options)
        % computeLidarMatchedCorrection Continuous LiDAR injection and its discretization.
        % The scaled pose Jacobian D\(G*T) is theta times the pose selector
        % in N=T*(P\C')*F. The physical-time gradient uses this selector.
        % Supply residual/jacobian/weights/linearizationPose or
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
            assert(isfield(design,'theta') && ~isfield(design,'kappa'), ...
                'VehicleLocalization:StaleInvariantDesign','Regenerate the LiDAR-normalized metric.');
            P=design.P;T=design.T;theta=design.theta;n=size(P,1);
            assert(isreal(P) && isequal(size(P),[n,n]) && all(isfinite(P),'all') && ...
                norm(P-P.','fro')<=1e-12*max(1,norm(P,'fro')), ...
                'VehicleLocalization:InvalidLidarMetric','P must be finite symmetric positive definite.');
            [L,flag]=chol((P+P.')/2,'lower');
            assert(flag==0 && isequal(size(T),[n,n]) && isreal(T) && all(isfinite(T),'all') ...
                && rcond(T)>eps && isequal(size(G),[3,n]), ...
                'VehicleLocalization:InvalidLidarMetric','Require SPD P, nonsingular fixed T and a 3-by-n pose Jacobian.');
            validateattributes(theta,{'numeric'},{'real','finite','positive','scalar'});
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
                    'VehicleLocalization:AmbiguousLidarMeasurement','Supply residual weights and Jacobian without a pose or information field.');
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
                error('VehicleLocalization:InvalidLidarMeasurement', ...
                    'Supply a cost gradient or geometric residual evaluated at the predicted pose.');
            end
            frame=1;direction=ones(3,1);
            if isfield(measurement,'frameReliability'),frame=measurement.frameReliability;end
            if isfield(measurement,'directionReliability'),direction=measurement.directionReliability(:);end
            assert(~isfield(measurement,'poseErrorVariance'),'VehicleLocalization:ArchivedRegistrationMeasurement', ...
                'Pose-error calibration no longer selects LiDAR weights.');
            filter=localizationSupport.filterLidarPoseInformation(information,cfg,frame,direction);
            C=filter.D\(G*T);
            if residualMode
                xi=-filter.F*filter.D.'*gradient;representation="predictedPoseResidual";
            else
                % Like a residual gradient J'*W*r, inject only where the cost has
                % positive curvature; unsupported directions receive no correction.
                normalized=filter.D.'*gradient;
                active=filter.eigenvalues>0;
                U=filter.eigenvectors(:,active);supported=U*(U.'*normalized);
                xi=-filter.F*supported;representation="predictedPoseGradient";
                unsupportedGradient=norm(normalized-supported);
            end
            % In metric coordinates the local curvature is B'*B. A thin SVD
            % avoids forming the ill-conditioned normal equations of the implicit step.
            B=diag(sqrt(filter.reliability/theta).*filter.eigenvalues)*filter.eigenvectors.'*(C/L.');
            [left,singular,right]=svd(B,'econ','vector');
            y=-sqrt(filter.reliability/theta).*(filter.eigenvectors.'*(filter.D.'*gradient));
            if gradientMode,y(~active)=0;end
            whitened=L\((C.'*xi)/theta);
            v=L.'\whitened;Q=(B*L.').'*(B*L.');
            largest=max(singular)^2;step=options.StepSize;
            gainBound=max(filter.reliability.*filter.eigenvalues)/theta;
            contraction=eye(n);
            if options.Discretization=="continuous"
                correction=T*v;
            elseif options.Discretization=="implicit"
                h=sqrt(step)*singular;
                attenuation=(h./hypot(1,h)).^2;
                response=sqrt(step)*(h./hypot(1,h))./hypot(1,h);
                delta=right*(response.*(left.'*y));
                correction=T*(L.'\delta);
                contraction=contraction-right*diag(attenuation)*right.';
            else
                if largest>0,step=min(step,1.8/largest);end
                correction=T*(step*v);
                contraction=contraction-right*diag(step*singular.^2)*right.';
            end
            energyRatio=norm(contraction,2)^2;
            audit=struct('filter',filter,'C',C,'dissipationMatrix',Q, ...
                'continuousCorrection',T*v,'effectiveStrengths',filter.strengths/theta, ...
                'maximumDissipationEigenvalue',largest,'requestedStep',options.StepSize, ...
                'appliedStep',step,'explicitStepProduct',step*largest, ...
                'jumpMaximumEnergyRatio',energyRatio, ...
                'jumpNonexpansive',energyRatio<=1+1e-10, ...
                'jumpCertificateApplies',options.Discretization~="continuous", ...
                'residualNoiseCoefficientBound',gainBound,'representation',representation, ...
                'unsupportedGradientNorm',unsupportedGradient, ...
                'scope',"Fixed-metric locally affine LiDAR channel only; no full-system or outage ISS claim");
        end

        function measurement=evaluateOverlapGradient(fixedCloud,movingCloud,pose,cfg)
        % evaluateOverlapGradient Correspondence-free LiDAR gradient at the observer prediction.
        % Crop the local map, condition its moments on view, and weight map and
        % source masses by available view reliability and temporal stability.
        % Average the exact single-scale negative-log overlap (Cauchy-Schwarz
        % divergence) gradient and Gauss-Newton metric across evaluable shared
        % semantic channels. Similarity is the geometric mean of normalized
        % class overlaps. Optional uniform background reduces the influence of
        % an unsupported class using the spatial density of the map crop.
            arguments
                fixedCloud (1,1) struct
                movingCloud (1,1) struct
                pose (3,1) double {mustBeReal,mustBeFinite}
                cfg (1,1) struct = overlapGradientConfig()
            end
            assert(isfield(cfg,'geometry') && ~any(isfield(cfg, ...
                {'registration','aggregation','orientation','outlierDensity','viewConditioning','evidenceWeights','scaleLadder'})), ...
                'VehicleLocalization:ObsoleteOverlapConfig','Rebuild overlap settings with overlapGradientConfig.');
            timer=tic;linearization=pose;pose=pose(:).';
            assert(isequal(string(cfg.geometry.heightMode),"xy"),'VehicleLocalization:PlanarOverlapRequired', ...
                'Feature-channel feedback requires the XY marginal.');
            measurement=struct('gradient',zeros(3,1),'information',zeros(3),'linearizationPose',linearization, ...
                'available',false,'similarity',0,'minimumInformation',NaN, ...
                'mapComponents',0,'sourceComponents',0, ...
                'sharedClasses',strings(1,0),'informationCalibrated',false,'seconds',0, ...
                'geometrySource',"per-feature-channel exact semantic overlap at the observer prediction");
            measurement.channels=struct('name',{},'similarity',{},'rawSimilarity',{},'supportFraction',{},'gradient',{},'information',{});
            local=localizationSupport.selectLocalProbabilityCloud(fixedCloud,pose,cfg.localMapRadius);
            if local.components.numComponents==0 || movingCloud.components.numComponents==0
                measurement.seconds=toc(timer);return;
            end
            local=localizationSupport.conditionSemanticMapOnView(local,pose);
            [f,m]=localizationSupport.prepareSemanticRegistration(local,movingCloud,cfg.geometry);
            if isfield(f,'viewReliability')
                f.mixtureWeight=f.mixtureWeight.*double(f.viewReliability(:));
            end
            if isfield(m,'temporalStability')
                m.mixtureWeight=m.mixtureWeight.*double(m.temporalStability(:));
            end
            measurement.mapComponents=f.numComponents;measurement.sourceComponents=m.numComponents;
            f.mean(:,1:2)=f.mean(:,1:2)-pose(1:2);
            origin=[0,0,pose(3)];gradient=zeros(3,1);information=zeros(3);
            shared=intersect(f.semanticName(f.mixtureWeight>0),m.semanticName(m.mixtureWeight>0));
            measurement.sharedClasses=reshape(string(shared),1,[]);
            useBackground=isfield(cfg,'uniformBackground') && cfg.uniformBackground;
            if useBackground
                validateattributes(cfg.localMapRadius,{'numeric'},{'real','finite','scalar','positive'});
            end
            logScore=0;count=0;hasSupport=false;
            for c=1:numel(shared)
                measurement.channels(c)=struct('name',shared(c),'similarity',NaN,'rawSimilarity',NaN,'supportFraction',0, ...
                    'gradient',nan(3,1),'information',nan(3,3));
                fixedClass=overlapClassComponents(f,shared(c));
                movingClass=overlapClassComponents(m,shared(c));
                [fixedClass,movingClass]=localizationSupport.balanceSemanticDistributions(fixedClass,movingClass);
                [energy,ascent,metric]=localizationSupport.semanticGaussianOverlap(fixedClass,movingClass,origin);
                % The balanced fields have unit L2 norm. Their L1 masses
                % convert the uniform disk density to the same overlap units.
                background=0;
                if useBackground
                    background=sum(fixedClass.mixtureWeight)*sum(movingClass.mixtureWeight)/(pi*cfg.localMapRadius^2);
                end
                if ~(energy+background>realmin),continue;end
                score=min(energy,1);
                fraction=energy/(energy+background);
                g=-ascent(:)/(energy+background);
                similarity=(score+background)/(1+background);
                measurement.channels(c).rawSimilarity=score;
                measurement.channels(c).similarity=similarity;
                measurement.channels(c).supportFraction=fraction;
                measurement.channels(c).gradient=g;
                measurement.channels(c).information=fraction*metric;
                logScore=logScore+log(similarity);
                gradient=gradient+g;information=information+fraction*metric;
                hasSupport=hasSupport || energy>realmin;
                count=count+1;
            end
            if count>0
                gradient=gradient/count;
                information=(information+information.')/(2*count);
                measurement.gradient=gradient;measurement.information=information;
                measurement.similarity=exp(logScore/count);
                measurement.minimumInformation=min(eig(information));
                measurement.available=hasSupport;
            end
            measurement.seconds=toc(timer);
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

        function [filtered,rate]=filterSignalWithRate(time,signal,timeConstant)
        % Causal first-order state filter; rate comes from tau*xDot=u-x.
        % This is a bandwidth-limited rate estimate with lag, not an exact
        % derivative of the measured signal. No measurement difference is used.
            arguments
                time (:,1) double {mustBeReal,mustBeFinite}
                signal (:,:) double {mustBeReal,mustBeFinite}
                timeConstant (1,1) double {mustBeReal,mustBeFinite,mustBePositive}
            end
            assert(~isempty(time) && size(signal,1)==numel(time) && all(diff(time)>0), ...
                'VehicleLocalization:InvalidRateFilter','Require aligned samples on an increasing clock.');
            filtered=signal;rate=zeros(size(signal));
            for k=2:numel(time)
                blend=-expm1(-(time(k)-time(k-1))/timeConstant);
                filtered(k,:)=filtered(k-1,:)+blend*(signal(k,:)-filtered(k-1,:));
                rate(k,:)=(signal(k,:)-filtered(k,:))/timeConstant;
            end
        end

        function [rotation,rate]=levelingRotation(roll,pitch,bodyRate)
        % Yaw-free leveling rotation and its analytic Euler-kinematic rate.
            arguments
                roll (1,1) double {mustBeReal,mustBeFinite}
                pitch (1,1) double {mustBeReal,mustBeFinite}
                bodyRate (3,1) double {mustBeReal,mustBeFinite}
            end
            cr=cos(roll);sr=sin(roll);cp=cos(pitch);sp=sin(pitch);
            assert(abs(cp)>1e-6,'VehicleLocalization:SingularTilt','Pitch is outside the Euler chart.');
            Rx=[1,0,0;0,cr,-sr;0,sr,cr];Ry=[cp,0,sp;0,1,0;-sp,0,cp];
            dRx=[0,0,0;0,-sr,-cr;0,cr,-sr];dRy=[-sp,0,cp;0,0,0;-cp,0,-sp];
            rollRate=bodyRate(1)+tan(pitch)*(sr*bodyRate(2)+cr*bodyRate(3));
            pitchRate=cr*bodyRate(2)-sr*bodyRate(3);
            rotation=Ry*Rx;rate=dRy*Rx*pitchRate+Ry*dRx*rollRate;
        end

        function acceleration=transportPlanarAcceleration(acceleration,yawRate,yawAcceleration,offset)
        % Transport from a reference point to the body-frame displacement offset.
        % Supply angular acceleration from observer dynamics, not gyro differences.
            arguments
                acceleration (:,2) double {mustBeReal,mustBeFinite}
                yawRate (:,1) double {mustBeReal,mustBeFinite}
                yawAcceleration (:,1) double {mustBeReal,mustBeFinite}
                offset (2,1) double {mustBeReal,mustBeFinite}
            end
            assert(size(acceleration,1)==numel(yawRate) && numel(yawRate)==numel(yawAcceleration), ...
                'VehicleLocalization:InvalidAccelerationTransport','Inputs must share a clock.');
            acceleration=acceleration+[-offset(2)*yawAcceleration-offset(1)*yawRate.^2, ...
                offset(1)*yawAcceleration-offset(2)*yawRate.^2];
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
                state.speed=filtered;
                % Derivative of the low-pass state: tau*vDot = wheelSpeed-v.
                acceleration=(speed-state.speed)/cfg.speedTimeSeconds;
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
                'gravityAccepted',accepted,'gravityNormMps2',norm(gravity),'gyroBias',state.gyroBias, ...
                'longitudinalAcceleration',acceleration,'accelerationSource',"speedFilterDynamics");
        end

        function assertSensorOnlyLidarTilt(tilt)
        % assertSensorOnlyLidarTilt Reject legacy caches carrying reference-derived tilt.
            assert(isstruct(tilt) && all(isfield(tilt, ...
                {'schemaVersion','source','referencePoseUsed','causal','configuration'})) && ...
                tilt.schemaVersion==1 && ~tilt.referencePoseUsed && tilt.causal && ...
                isfield(tilt.configuration,'accelerationSource') && ...
                string(tilt.configuration.accelerationSource)=="speedFilterDynamics" && ...
                ismember(string(tilt.source),["front-ouster-raw-imu","disabled"]), ...
                'VehicleLocalization:ReferenceTiltCache','Regenerate tilt using current sensor-only filter dynamics; legacy caches are not allowed.');
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
                isequal(string(cfg.method),"robust"), ...
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
                    if nnz(selected)>=3
                        center=median(candidate(selected));
                        selected=selected & abs(candidate-center)<=cfg.consensusThreshold;
                    end
                    if nnz(selected)==2 && ...
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

% Intrinsic observer and gain synthesis: local helpers
function m=invariantModel(cfg)
    z=cfg.invariantObserver;s=cfg.iss;ell=z.headingLengthM;sigma=z.sigma;n=z.noise;
    r=z.fusion;
    validateattributes([z.theta,ell,s.maximumSpeed],{'double'},{'real','finite','positive'});
    validateattributes(n.gnssPositionM,{'double'},{'real','finite','nonnegative','scalar'});
    validateattributes(n.gnssHeadingRad,{'double'},{'real','finite','positive','scalar'});
    headingNoise=ell*n.gnssHeadingRad;
    validateattributes(r.minimumCourseSpeed,{'double'},{'real','finite','positive','scalar','<=',s.maximumSpeed});
    validateattributes([s.maximumAcceleration,cfg.maximumBodyYawRate,r.lidarOperatorMismatch], ...
        {'double'},{'real','finite','nonnegative'});
    validateattributes(z.yawAccelerationRadps2,{'double'},{'real','finite','nonnegative','scalar'});
    validateattributes(n.upstreamRateRadps,{'double'},{'real','finite','positive','scalar'});
    assert(z.theta==z.sigma && z.lambda>0 && z.margin>0, ...
        'VehicleLocalization:InvalidFusionDomain','Require theta=sigma and positive LMI margins.');
    assert(min([s.minimumPositionStrength,s.minimumHeadingStrength])> ...
        s.maximumPositionHeadingCoupling+r.lidarOperatorMismatch, ...
        'VehicleLocalization:InvalidFusionDomain','The nominal fusion LiDAR lower bound must be positive.');
    assert(~isfield(z,'angularMetricDiagonal'),'VehicleLocalization:ObsoleteInvariantDesign', ...
        'Rebuild the observer configuration for the direct upstream rate channel.');
    validateattributes([n.velocityMps,n.accelerationMps2],{'double'},{'real','finite','nonnegative'});
    validateattributes(z.motionTimeConstantS,{'double'},{'real','finite','positive','scalar'});
    assert(z.noiseObjective=="normalizedFullState" && isequal(cfg.lidar.poseScales(:),[1;1;1/ell]), ...
        'VehicleLocalization:InvalidInvariantDesign','Require the declared full-state objective and consistent pose scaling.');
    m.C=zeros(3,8);m.C(1,1)=1;m.C(2,4)=1;m.C(3,7)=1;
    m.B=zeros(8,2);m.B(3,1)=1;m.B(6,2)=1;
    m.DL=diag([n.lidarPositionM,n.lidarPositionM,ell*n.lidarHeadingRad]);m.DG=diag([n.gnssPositionM,n.gnssPositionM,headingNoise]);
    % Aggregate residual R(psi)*bodyVelocityDoubleDot + yawAcceleration*J*v.
    % The objective scale is a design weight, not a certified physical bound.
    m.Dd=(n.jerkMps3/sigma^3)*eye(2);
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

% Common sensor certificate: local helpers
function value=sensorWorstEigenvalue(matrices)
    value=-Inf;
    for k=1:numel(matrices)
        M=matrices{k};value=max(value,max(eig((M+M.')/2)));
    end
end

% LiDAR measurement and correction: local helpers
function part=overlapClassComponents(components,name)
% Keep the fields used by the overlap integral for one feature channel.
    keep=components.semanticName==name & components.mixtureWeight>0;
    part=struct('mean',components.mean(keep,:), 'covariance',components.covariance(:,:,keep), ...
        'mixtureWeight',components.mixtureWeight(keep), 'semanticName',components.semanticName(keep));
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
