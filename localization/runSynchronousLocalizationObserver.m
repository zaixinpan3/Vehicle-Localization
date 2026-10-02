function estimate=runSynchronousLocalizationObserver(data,cfg,lateral)
% runSynchronousLocalizationObserver One backward-Euler update per frame.
% Inputs and real measurement reconstructions must share the same frame clock.
% No pose anchor is propagated, no inter-frame pose is generated, and absent
% measurements withdraw their own correction. The continuous gain certificate
% is retained as design context, not a proof of this sampled nonlinear system.
% Optional data.lidarMatcher(k,seed,positionAid) runs before the current update.
% It replaces data.lidar, receives the previous fused-state prediction and
% current point-corrected GNSS, and returns a registration result. Candidate
% selection may depend on GNSS; its information must exclude GNSS curvature.
% Wheel and lateral-observer velocities are used without LiDAR-derived bias.
% GNSS corrects position only; the heading is the integrated gyro, corrected
% only by qualified LiDAR yaw (diagnostics.headingMode 2, otherwise 0).
    h=data.highRate;t=h.time(:);n=numel(t);
    assert(n>=2 && all(isfinite(t)) && all(diff(t)>0), ...
        'VehicleLocalization:InvalidSyncClock','Require an increasing frame clock.');
    assert(isfield(lateral,'time') && isequal(lateral.time(:),t), ...
        'VehicleLocalization:AlignedLateralRequired','Supply lateral estimates on the frame clock.');
    fields=["longitudinalSpeed","longitudinalAcceleration","lateralAcceleration","yawRate"];
    for name=fields
        assert(numel(h.(name))==n && all(isfinite(h.(name))), ...
            'VehicleLocalization:InvalidFullInput','Invalid aligned motion.');
    end
    for name=["lateralVelocity","sideSlipAngleRate"]
        assert(numel(lateral.(name))==n && all(isfinite(lateral.(name))), ...
            'VehicleLocalization:InvalidFullInput','Invalid aligned lateral output.');
    end
    G=source(data,'gnss',t,2);L=source(data,'lidar',t,3);
    onlineMatching=isfield(data,'lidarMatcher');
    if onlineMatching
        assert(isa(data.lidarMatcher,'function_handle') && ~isfield(data,'lidar'), ...
            'VehicleLocalization:AmbiguousLidarInput','Supply a matcher or recorded LiDAR measurements, not both.');
    end
    matchingResults=cell(0,1);matchingSeeds=zeros(0,3);
    if onlineMatching,matchingResults=cell(n,1);matchingSeeds=zeros(n,3);end
    alignment=struct('bodyOffset',[0;0],'bodyCovariance',zeros(2),'headingStdRad',0);
    if isfield(cfg.gnss,'outputPoint'),alignment=cfg.gnss.outputPoint;end
    continuous=designFullObserverGains(cfg);
    design=struct('kind',"synchronous-backward-euler",'continuousDesign',continuous, ...
        'sampledSystemCertified',false,'gainsRetuned',isfield(cfg.gnss,'positionGainDesign'));
    x=cfg.initialState;
    assert(numel(x)==7 && all(isfinite(x)),'VehicleLocalization:FullInitializationRequired', ...
        'Supply an explicit common initial state.');x=x(:);
    z=zeros(n,7);mode=zeros(n,1);headingMode=zeros(n,1);correction=zeros(n,4);
    yawCorrection=zeros(n,1);gnssPosition=nan(n,2);gnssInformation=nan(2,2,n);
    margins=nan(n,1);weights=nan(n,2);maximumRate=0;
    for k=1:n
        dt=0;if k>1,dt=t(k)-t(k-1);end
        speed=h.longitudinalSpeed(k);vy=lateral.lateralVelocity(k);
        rate=h.yawRate(k);
        if k>1,rate=(h.yawRate(k-1)+rate)/2;end
        predYaw=x(7)+rate*dt;
        if onlineMatching
            midYaw=x(7)+rate*dt/2;
            Rmid=[cos(midYaw),-sin(midYaw);sin(midYaw),cos(midYaw)];
            seed=[(x([1,4])+dt*Rmid*[speed;vy]).',predYaw];
            aid=struct('valid',G.valid(k),'timestamp',t(k));
            if aid.valid
                [aid.position,I]=correctGnssOutputPoint(G.values(k,:),G.information(:,:,k),predYaw,alignment);
                aid.covariance=I\eye(2);
            end
            matched=data.lidarMatcher(k,seed,aid);
            matchingResults{k}=matched;matchingSeeds(k,:)=seed;
            L.values(k,:)=matched.poseXYTheta;L.information(:,:,k)=matched.information;
            % The existing observer requires full pose. Partial geometry is
            % retained in diagnostics rather than filled with a GNSS prior.
            L.valid(k)=matched.accepted;
            assert(~L.valid(k) || (all(isfinite(L.values(k,:))) && ...
                all(isfinite(L.information(:,:,k)),'all') && min(eig(L.information(:,:,k)))>0), ...
                'VehicleLocalization:InvalidOnlineLidar','Accepted matcher output needs finite full-pose geometry.');
        end
        q=h.yawRate(k)+lateral.sideSlipAngleRate(k);maximumRate=max(maximumRate,abs(q));
        Wl=weight(L,k,cfg.lidar.gainInformationScale,3);
        yaw=predYaw;
        if L.valid(k) && Wl(3,3)>=cfg.lidar.minimumPoseWeight
            gain=cfg.gains(4)*Wl(3,3);yaw=predYaw+dt*gain/(1+dt*gain)*wrap(L.values(k,3)-predYaw);
            headingMode(k)=2;yawCorrection(k)=gain*wrap(L.values(k,3)-yaw);
        end
        Wg=zeros(2);
        if G.valid(k)
            [gnssPosition(k,:),gnssInformation(:,:,k)]=correctGnssOutputPoint(G.values(k,:),G.information(:,:,k),yaw,alignment);
            I=gnssInformation(:,:,k);Wg=I/(I+cfg.gnss.gainInformationScale*eye(2));Wg=(Wg+Wg.')/2;
        end
        K=cfg.gnss.positionGain*Wg+cfg.gains(1)*Wl(1:2,1:2);
        R=[cos(yaw),-sin(yaw);sin(yaw),cos(yaw)];J=[0,-1;1,0];
        velocity=R*[speed;vy];acceleration=R*[h.longitudinalAcceleration(k);h.lateralAcceleration(k)];
        A=[(1+dt*cfg.gains(2))*eye(2),-dt*eye(2); ...
            -dt*q^2*eye(2),(1+dt*cfg.gains(3))*eye(2)-2*dt*q*J];
        va=A\[x([2,5])+dt*cfg.gains(2)*velocity;x([3,6])+dt*cfg.gains(3)*acceleration];
        b=zeros(2,1);
        if G.valid(k),b=b+cfg.gnss.positionGain*Wg*gnssPosition(k,:).';end
        if L.valid(k),b=b+cfg.gains(1)*Wl(1:2,1:2)*L.values(k,1:2).';end
        p=(eye(2)+dt*K)\(x([1,4])+dt*va(1:2)+dt*b);
        x=[p(1);va(1);va(3);p(2);va(2);va(4);yaw];
        assert(all(isfinite(x)),'VehicleLocalization:NonfiniteObserver','Synchronous update became nonfinite.');
        if G.valid(k),correction(k,1:2)=(cfg.gnss.positionGain*Wg*(gnssPosition(k,:).'-p)).';weights(k,1)=min(eig(Wg));end
        if L.valid(k),correction(k,3:4)=(cfg.gains(1)*Wl(1:2,1:2)*(L.values(k,1:2).'-p)).';weights(k,2)=min(eig(Wl));end
        alpha=min(eig(K));q2=cfg.maximumTrackAngleRate^2;
        margins(k)=min(eig([2*alpha,-1,0;-1,2*cfg.gains(2),-(1+q2);0,-(1+q2),2*cfg.gains(3)]));
        z(k,:)=x.';mode(k)=double(G.valid(k))+2*double(L.valid(k));
    end
    estimate=struct('time',t,'z',z,'onlineZ',z,'pose',[z(:,[1,4]),wrap(z(:,7))], ...
        'position',z(:,[1,4]),'velocity',z(:,[2,5]),'acceleration',z(:,[3,6]), ...
        'heading',wrap(z(:,7)),'headingUnwrapped',z(:,7),'lateral',lateral,'observer',design);
    estimate.diagnostics=struct('mode',mode,'headingMode',headingMode,'positionCorrection',correction, ...
        'gnssPositionAtObserverPoint',gnssPosition,'gnssInformationAtObserverPoint',gnssInformation, ...
        'yawCorrection',yawCorrection, ...
        'minimumWeights',weights,'translationDissipationMargin',margins, ...
        'maximumTrackAngleRate',maximumRate, ...
        'rateEnvelopeSatisfied',maximumRate<=cfg.maximumTrackAngleRate,'stateResets',0, ...
        'packetCounts',struct('gnssValid',nnz(G.valid),'gnssInvalid',nnz(~G.valid), ...
        'lidarValid',nnz(L.valid),'lidarInvalid',nnz(~L.valid)), ...
        'localizationUpdates',n-1,'virtualPoseUpdates',0,'integrationSubsteps',0, ...
        'nominalOutputRateHz',1/median(diff(t)),'referenceUsed',false, ...
        'allTheoremHypothesesVerified',false,'sampledSystemCertified',false, ...
        'scope',"One implicit update per synchronized frame; no measurement transport. Input alignment is a separate offline operation.");
    if onlineMatching
        estimate.matchingResults=matchingResults;estimate.matchingSeeds=matchingSeeds;
        estimate.diagnostics.matchingFeedback=true;
        estimate.diagnostics.matchingSource="Per-frame callback with fused seed and optional current position aid";
    end
end

function s=source(data,name,t,width)
    n=numel(t);s=struct('valid',false(n,1),'values',nan(n,width),'information',nan(width,width,n));
    if ~isfield(data,name),return;end
    a=data.(name);field='pose';if width==2,field='position';end
    assert(isfield(a,'delay') && isequal(a.delay,0),'VehicleLocalization:FullZeroDelayRequired','Declare zero perception delay.');
    assert(isequal(a.time(:),t),'VehicleLocalization:SynchronousInputsRequired','Source clocks must equal the localization clock.');
    assert(isequal(size(a.(field)),[n,width]) && numel(a.valid)==n && ...
        all(ismember(a.valid,[0,1])) && size(a.information,1)==width && size(a.information,2)==width && size(a.information,3)==n, ...
        'VehicleLocalization:InvalidFullSource','Invalid synchronized source dimensions.');
    s=struct('valid',logical(a.valid(:)),'values',a.(field),'information',a.information);
    for k=find(s.valid).'
        I=s.information(:,:,k);
        assert(all(isfinite(s.values(k,:))) && all(isfinite(I),'all') && ...
            norm(I-I.','fro')<=1e-9*max(1,norm(I,'fro')) && min(eig((I+I.')/2))>0, ...
            'VehicleLocalization:InvalidFullSource','Valid frames require finite positive information.');
    end
end

function W=weight(s,k,scale,width)
    W=zeros(width);if ~s.valid(k),return;end
    I=s.information(:,:,k);I=(I+I.')/2;W=I/(I+scale*eye(width));W=(W+W.')/2;
end

function x=wrap(x)
    x=atan2(sin(x),cos(x));
end
