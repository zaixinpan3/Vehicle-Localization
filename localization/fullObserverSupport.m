classdef fullObserverSupport
% fullObserverSupport Continuous gain certificate and inputs of the full observer.
% Static methods verify the intrinsic eight-state gains, align
% measurement sources on the LiDAR frame clock and move GNSS positions to the
% observer output point. runFullLocalizationObserver is the runtime entry.
% Example: design = fullObserverSupport.designFullObserverGains(cfg).

    methods (Static)
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
            design=invariantObserverSupport.runtime(cfg);
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
            [left,right,fraction,covered]=brackets(h.time,t,cfg.synchronization.motionMaximumBracket);
            assert(all(covered),'VehicleLocalization:SyncCoverage','Motion does not cover the frame clock.');
            aligned=struct('highRate',struct('time',t));
            for name=string(fieldnames(h)).'
                if name=="time",continue;end
                if any(name==["bodyVelocityValid","bodyAccelerationValid","yawRateValid"])
                    aligned.highRate.(name)=logical(h.(name)(left)) & logical(h.(name)(right));
                else
                    aligned.highRate.(name)=blend(h.(name),left,right,fraction);
                end
            end
            assert(isequal(lateralInput.time,h.time),'VehicleLocalization:SyncCoverage','Lateral and motion clocks differ.');
            lateral=struct('time',t);
            assert(all(isfield(lateralInput,{'lateralVelocity','sideSlipAngle','sideSlipAngleRate'})), ...
                'VehicleLocalization:AlignedLateralRequired','Supply the upstream lateral angle/rate pair before synchronization.');
            for name=["lateralVelocity","sideSlipAngle","sideSlipAngleRate"]
                lateral.(name)=blend(lateralInput.(name),left,right,fraction);
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
