function [aligned,lateral,metadata,selected]=alignFrameClock(data,lateralInput,cfg,t)
% Align motion and GNSS on covered LiDAR timestamps.
    h=data.highRate;
    if isfield(lateralInput,'minimumDesignSpeedMps')
        lateralInput=globalObserverMotionMeasurements(lateralInput,lateralInput.measurements.longitudinalSpeed,lateralInput.measurements.yawRate);
        low=lateralInput.lowSpeedMeasurement;
        h.longitudinalSpeed(low)=lateralInput.longitudinalSpeed(low);
    end
    assert(all(diff(t)>0) && all(isfinite(t)),'VehicleLocalization:InvalidSyncClock','Invalid LiDAR clock.');
    selected=t>=h.time(1) & t<=h.time(end);t=t(selected);
    assert(numel(t)>=2,'VehicleLocalization:SyncCoverage','Need two covered localization frames.');
    [left,right,fraction,covered]=observerInternal.frameBrackets(h.time,t,cfg.synchronization.motionMaximumBracket);
    assert(all(covered),'VehicleLocalization:SyncCoverage','Motion does not cover the frame clock.');
    aligned=struct('highRate',struct('time',t));
    for name=string(fieldnames(h)).'
        if name=="time",continue;end
        if any(name==["bodyVelocityValid","bodyAccelerationValid","yawRateValid"])
            mask=h.(name);
            assert(isvector(mask) && numel(mask)==numel(h.time) && all(ismember(mask(:),[0,1])), ...
                'VehicleLocalization:InvalidMotionValidity','Invalid native motion validity mask.');
            mask=logical(mask(:));aligned.highRate.(name)=mask(left) & mask(right);
        else
            aligned.highRate.(name)=fullBlend(h.(name),left,right,fraction);
        end
    end
    assert(isequal(lateralInput.time,h.time),'VehicleLocalization:SyncCoverage','Lateral and motion clocks differ.');
    lateral=struct('time',t);
    assert(all(isfield(lateralInput,{'lateralVelocity'})), ...
        'VehicleLocalization:AlignedLateralRequired','Supply upstream lateral velocity before synchronization.');
    for name="lateralVelocity"
        lateral.(name)=fullBlend(lateralInput.(name),left,right,fraction);
    end
    if isfield(lateralInput,'yawRate')
        [rate,valid]=observerInternal.upstreamYawRate(lateralInput);
        lateral.yawRate=fullBlend(rate,left,right,fraction);
        lateral.yawRateValid=valid(left) & valid(right);
    end
    if isfield(lateralInput,'velocityValid')
        valid=lateralInput.velocityValid(:);
        assert(numel(valid)==numel(h.time) && all(ismember(valid,[0,1])), ...
            'VehicleLocalization:InvalidMotionValidity','Invalid lateral velocity validity mask.');
        lateral.velocityValid=logical(valid(left)) & logical(valid(right));
    end
    motionWait=h.time(right)-t;gnssWait=zeros(size(t));gnssBracket=zeros(size(t));
    if isfield(data,'gnss')
        g=data.gnss;
        [a,b,w,ok]=observerInternal.frameBrackets(g.time,t,cfg.synchronization.gnssMaximumBracket);
        ok=ok & g.valid(a) & g.valid(b);
        position=nan(numel(t),2);information=nan(2,2,numel(t));
        for k=find(ok).'
            position(k,:)=(1-w(k))*g.position(a(k),:)+w(k)*g.position(b(k),:);
            if isfield(g,'information')
                information(:,:,k)=(1-w(k))*g.information(:,:,a(k))+w(k)*g.information(:,:,b(k));
            end
        end
        gnssWait(ok)=g.time(b(ok))-t(ok);gnssBracket(ok)=g.time(b(ok))-g.time(a(ok));
        aligned.gnss=struct('time',t,'position',position,'information',information,'valid',ok,'delay',0, ...
            'sourceLeftTime',g.time(a),'sourceRightTime',g.time(b));
        if ~isfield(g,'information'),aligned.gnss=rmfield(aligned.gnss,'information');end
    end
    if isfield(data,'gnssHeading') || (isfield(data,'gnss') && isfield(data.gnss,'heading'))
        heading=localizationSupport.headingMeasurements(data,t,cfg);
        if isfield(data,'gnssHeading')
            aligned.gnssHeading=rmfield(heading,{'waitSeconds'});
        else
            aligned.gnss.heading=heading.heading;
            aligned.gnss.valid=aligned.gnss.valid & heading.valid;
        end
        gnssWait=max(gnssWait,heading.waitSeconds);
    end
    metadata=struct('clock',"Native LiDAR frames; no synthetic inter-frame localization outputs", ...
        'nominalRateHz',1/median(diff(t)),'frames',numel(t), ...
        'motionModelMeasurementExtrapolation',false,'offlineBracketInterpolation',true, ...
        'maximumGnssBracketSeconds',max(gnssBracket),'maximumGnssWaitSeconds',max(gnssWait), ...
        'maximumMotionWaitSeconds',max(motionWait),'frameWaitSeconds',max(motionWait,gnssWait), ...
        'inputScope',"Wheel/lateral estimates retain their native preprocessing; only aligned frame values enter localization");
end

function result=fullBlend(values,a,b,w)
    result=(1-w).*values(a,:)+w.*values(b,:);
end
