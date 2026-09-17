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
