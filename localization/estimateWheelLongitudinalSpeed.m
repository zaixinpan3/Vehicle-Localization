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
