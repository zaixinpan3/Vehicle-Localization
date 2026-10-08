classdef fullObserverSupport
% fullObserverSupport Continuous gain certificate and inputs of the full observer.
% Static methods design and verify the continuous seven-state ISS gains, align
% measurement sources on the LiDAR frame clock and move GNSS positions to the
% observer output point. runFullLocalizationObserver is the runtime entry.
% Example: design = fullObserverSupport.designFullObserverGains(cfg).

    methods (Static)
        function design=designFullObserverGains(cfg)
        % designFullObserverGains Certify continuous gains before discretization.
        % The continuous seven-state ISS LMIs include position/heading coupling and
        % motion-output feedback. Numerical integration uses these gains unchanged.
            arguments
                cfg (1,1) struct=fullObserverConfig()
            end
            obsolete=intersect(string(fieldnames(cfg.gnss)).',["headingGain","courseWindow", ...
                "maximumCourseGap","minimumCourseDisplacement","minimumSpeed","headingErrorLimit"]);
            assert(isempty(obsolete),'VehicleLocalization:ObsoleteFullConfig', ...
                'GNSS does not correct the heading; remove cfg.gnss fields: %s.',strjoin(obsolete,', '));
            certificate=fullObserverSupport.solveFullObserverIssLmi(cfg);
            assert(certificate.certified,'VehicleLocalization:InfeasibleFullCertificate', ...
                'Candidate gains failed the continuous seven-state ISS LMIs: %s',certificate.solverInfo);
            g=cfg.gains(:);
            matched=struct('P',blkdiag(eye(6),g(1)/g(4)),'T',eye(7),'theta',1, ...
                'scalingExponents',[1;2;3;1;2;3;1],'kappa',g(1), ...
                'baselineFullStateCertified',false, ...
                'metricSource',"Algebraic LiDAR injection metric; the full continuous proof uses continuousCertificate.verification.P");
            design=struct('kind',cfg.kind,'gains',g.','gnssPositionGain',cfg.gnss.positionGain, ...
                'unconditionalIssClaimed',false, ...
                'baselineFullStateCertified',false,'lidarMatched',matched, ...
                'continuousCertificate',certificate,'fullContinuousStateCertified',true, ...
                'designRoute',certificate.designRoute, ...
                'scope',certificate.verification.scope);
        end

        function certificate=solveFullObserverIssLmi(cfg)
        % solveFullObserverIssLmi Test candidate gains in the continuous ISS LMIs.
        % Solve before discretization. Outer parameter selection fixes the gains;
        % this inner SDP finds a common Lyapunov certificate. An infeasible result is
        % not a proof of instability.
        % Requires YALMIP and the configured SDP solver on the MATLAB path.
            arguments
                cfg (1,1) struct
            end
            problem=fullObserverSupport.fullObserverIssProblem(cfg);
            persistent cache
            if isempty(cache),cache=containers.Map('KeyType','char','ValueType','any');end
            key=jsonencode(struct('problem',problem,'solver',cfg.iss.solver));
            if isKey(cache,key),certificate=cache(key);return;end
            assert(exist('sdpvar','file')==2,'VehicleLocalization:MissingSolver', ...
                'Add YALMIP and the SDP solver to the path before continuous gain design.');
            % Homogeneous weights with fixed trace avoid the poorly conditioned
            % large-yaw-weight solutions produced by fixing the velocity weight to 1.
            x=sdpvar(4,1);slack=sdpvar(1);cp=x(1);cy=x(2);cv=x(3);mu=x(4);g=problem.gains;
            W=diag(x);constraints=[x>=1e-9,sum(x)==1];
            for q2=[0,problem.maximumCourseRate^2]
                a=cp*g(1);b=cy*g(4);cross=-(a+b)*problem.maximumPositionHeadingCoupling;
                M=[2*a*problem.minimumPositionStrength,cross,-cp,0; ...
                    cross,2*b*problem.minimumHeadingStrength-cp*problem.gnssLeverPenalty, ...
                    -cv*g(2)*problem.maximumSpeed,-mu*g(3)*problem.maximumAcceleration; ...
                    -cp,-cv*g(2)*problem.maximumSpeed,2*cv*g(2),-(cv+mu*q2); ...
                    0,-mu*g(3)*problem.maximumAcceleration,-(cv+mu*q2),2*mu*g(3)];
                constraints=[constraints,M-problem.decayRate*W>=slack*eye(4)]; %#ok<AGROW>
            end
            result=optimize(constraints,-slack,sdpsettings('solver',char(cfg.iss.solver),'verbose',0));
            recovered=value(x);
            certificate=struct('problem',problem,'weights',recovered([1,2,4])/recovered(3),'solverStatus',result.problem, ...
                'solverInfo',string(result.info),'solverSlack',value(slack),'certified',false, ...
                'designRoute',"Continuous ISS LMI, then discretization with unchanged gains");
            % A solver warning is not a mathematical infeasibility result. Accept a
            % returned witness only through the independent, strictly positive LMI
            % margin, and retain the solver status even when that witness is valid.
            if all(isfinite(certificate.weights)) && all(certificate.weights>0)
                certificate.verification=fullObserverSupport.verifyFullObserverIssCertificate(certificate,cfg);
                certificate.certified=certificate.verification.certified;
            end
            if certificate.certified,cache(key)=certificate;end
        end

        function verification=verifyFullObserverIssCertificate(certificate,cfg)
        % verifyFullObserverIssCertificate Recheck the continuous LMIs without a solver.
        % P certifies the complete continuous observer. It is distinct from the
        % algebraic metric used to implement the LiDAR injection. Physical bounds,
        % information availability and the local heading chart remain assumptions.
            arguments
                certificate (1,1) struct
                cfg (1,1) struct
            end
            p=fullObserverSupport.fullObserverIssProblem(cfg);
            assert(isfield(certificate,'problem') && isequaln(certificate.problem,p), ...
                'VehicleLocalization:StaleFullIssCertificate', ...
                'Re-solve the continuous LMIs after changing gains or design assumptions.');
            w=certificate.weights(:);
            validateattributes(w,{'numeric'},{'real','finite','positive','numel',3});
            cp=w(1);cy=w(2);mu=w(3);g=p.gains;
            W=diag([cp,cy,1,mu]);root=sqrt(diag(W));
            matrices=zeros(4,4,2);margins=zeros(2,1);rates=zeros(2,1);
            for vertex=1:2
                q2=(vertex-1)*p.maximumCourseRate^2;
                a=cp*g(1);b=cy*g(4);cross=-(a+b)*p.maximumPositionHeadingCoupling;
                M=[2*a*p.minimumPositionStrength,cross,-cp,0; ...
                    cross,2*b*p.minimumHeadingStrength-cp*p.gnssLeverPenalty, ...
                    -g(2)*p.maximumSpeed,-mu*g(3)*p.maximumAcceleration; ...
                    -cp,-g(2)*p.maximumSpeed,2*g(2),-(1+mu*q2); ...
                    0,-mu*g(3)*p.maximumAcceleration,-(1+mu*q2),2*mu*g(3)];
                matrices(:,:,vertex)=M;
                margins(vertex)=min(eig((M-p.decayRate*W)./(root*root.')));
                rates(vertex)=min(eig(M./(root*root.')));
            end
            P=diag([cp,1,mu,cp,1,mu,cy]);
            verification=struct('certified',all(margins>p.tolerance), ...
                'weights',w,'P',P,'comparisonMatrices',matrices,'comparisonWeight',W, ...
                'normalizedMargins',margins,'verifiedDecayRate',min(rates), ...
                'requestedDecayRate',p.decayRate,'continuousTime',true, ...
                'sampledSystemCertified',false, ...
                'scope',"Seven-state continuous ISS under the declared maneuver and uniformly sufficient LiDAR information bounds; estimated-heading GNSS lever arm included. No arbitrary-outage or numerical-integration guarantee.");
        end

        function problem=fullObserverIssProblem(cfg)
        % fullObserverIssProblem Declare the continuous seven-state ISS assumptions.
        % A fixed candidate gain makes the two comparison inequalities LMIs in the
        % Lyapunov weights. Timing and integration method do not enter this problem.
            arguments
                cfg (1,1) struct
            end
            assert(~isfield(cfg,'timing') || isequal(string(cfg.timing),"synchronous"), ...
                'VehicleLocalization:ObsoleteFullConfig', ...
                'Only synchronized localization is supported; the historical transported runtime was removed.');
            assert(isfield(cfg,'iss'),'VehicleLocalization:MissingFullIssDomain', ...
                'Declare the continuous ISS domain in cfg.iss before selecting gains.');
            s=cfg.iss;g=cfg.gains(:).';
            validateattributes(g,{'numeric'},{'real','finite','positive','numel',4});
            validateattributes(cfg.maximumTrackAngleRate,{'numeric'},{'real','finite','nonnegative','scalar'});
            validateattributes(cfg.gnss.positionGain,{'numeric'},{'real','finite','nonnegative','scalar'});
            scale=cfg.gnss.gainInformationScale;
            assert(isnumeric(scale) && isscalar(scale) && isreal(scale) && isfinite(scale) && scale>0, ...
                'VehicleLocalization:InvalidGnssInformationScale', ...
                'A positive GNSS regularizer is required for the certified weight bound.');
            validateattributes([s.maximumSpeed,s.maximumAcceleration,s.maximumPositionHeadingCoupling], ...
                {'numeric'},{'real','finite','nonnegative','numel',3});
            validateattributes([s.minimumPositionStrength,s.minimumHeadingStrength], ...
                {'numeric'},{'real','finite','positive','<',1,'numel',2});
            validateattributes([s.decayRate,s.tolerance],{'numeric'},{'real','finite','positive','numel',2});
            assert(isequal(cfg.lidar.poseScales(:),ones(3,1)), ...
                'VehicleLocalization:UnsupportedFullIssScaling','The continuous comparison uses metres and radians without pose rescaling.');
            lidarInjectionSupport.filterLidarPoseInformation(zeros(3),cfg);
            offset=zeros(2,1);
            if isfield(cfg.gnss,'outputPoint'),offset=cfg.gnss.outputPoint.bodyOffset(:);end
            validateattributes(offset,{'numeric'},{'real','finite','numel',2});
            problem=struct('gains',g,'maximumCourseRate',cfg.maximumTrackAngleRate, ...
                'maximumSpeed',s.maximumSpeed,'maximumAcceleration',s.maximumAcceleration, ...
                'minimumPositionStrength',s.minimumPositionStrength, ...
                'minimumHeadingStrength',s.minimumHeadingStrength, ...
                'maximumPositionHeadingCoupling',s.maximumPositionHeadingCoupling, ...
                'decayRate',s.decayRate,'gnssLeverPenalty',.5*cfg.gnss.positionGain*sum(offset.^2), ...
                'gnssPositionGain',cfg.gnss.positionGain,'gnssBodyOffset',offset, ...
                'gnssInformationScale',cfg.gnss.gainInformationScale, ...
                'tolerance',s.tolerance);
            if isfield(cfg.lidar,'errorCalibration')
                problem.lidarErrorCalibration=cfg.lidar.errorCalibration;
            else
                problem.lidarInformationScale=cfg.lidar.gainInformationScale;
            end
        end

        function [aligned,lateral,metadata]=synchronizeLocalizationInputs(data,lateralInput,cfg)
        % synchronizeLocalizationInputs Align existing observations on LiDAR frames.
        % Offline bracket interpolation uses real endpoints only, without motion
        % extrapolation. Invalid endpoints or excessive gaps withdraw that channel.
        % The future-endpoint wait is reported separately from perception delay.
            arguments
                data (1,1) struct
                lateralInput (1,1) struct
                cfg (1,1) struct=fullObserverConfig()
            end
            h=data.highRate;t=data.lidar.time(:);
            assert(all(diff(t)>0) && all(isfinite(t)),'VehicleLocalization:InvalidSyncClock','Invalid LiDAR clock.');
            selected=t>=h.time(1) & t<=h.time(end);t=t(selected);
            assert(numel(t)>=2,'VehicleLocalization:SyncCoverage','Need two covered localization frames.');
            [left,right,fraction,covered]=brackets(h.time,t,cfg.synchronization.motionMaximumBracket);
            assert(all(covered),'VehicleLocalization:SyncCoverage','Motion does not cover the frame clock.');
            aligned=struct('highRate',struct('time',t));
            for name=string(fieldnames(h)).'
                if name=="time",continue;end
                aligned.highRate.(name)=blend(h.(name),left,right,fraction);
            end
            assert(isequal(lateralInput.time,h.time),'VehicleLocalization:SyncCoverage','Lateral and motion clocks differ.');
            lateral=struct('time',t);
            for name=["lateralVelocity","sideSlipAngleRate"]
                lateral.(name)=blend(lateralInput.(name),left,right,fraction);
            end
            aligned.lidar=data.lidar;aligned.lidar.time=t;
            aligned.lidar.pose=data.lidar.pose(selected,:);
            aligned.lidar.valid=data.lidar.valid(selected);
            aligned.lidar.information=data.lidar.information(:,:,selected);
            for name=["frameReliability","directionReliability"]
                if isfield(data.lidar,name),aligned.lidar.(name)=data.lidar.(name)(selected,:);end
            end
            if isfield(data.lidar,'evaluateResidual')
                originalIndex=find(selected);provider=data.lidar.evaluateResidual;
                aligned.lidar.evaluateResidual=@(k,pose)provider(originalIndex(k),pose);
            end
            if isfield(data.lidar,'residualModels'),aligned.lidar.residualModels=data.lidar.residualModels(selected);end
            motionWait=h.time(right)-t;gnssWait=zeros(size(t));gnssBracket=zeros(size(t));
            if isfield(data,'gnss')
                g=data.gnss;
                [a,b,w,ok]=brackets(g.time,t,cfg.synchronization.gnssMaximumBracket);
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
    end
end

function [a,b,w,ok]=brackets(source,target,maximum)
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

function result=blend(values,a,b,w)
    result=(1-w).*values(a,:)+w.*values(b,:);
end
