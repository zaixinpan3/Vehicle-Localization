function estimate=runSynchronousLocalizationObserver(data,cfg,lateral)
% runSynchronousLocalizationObserver One backward-Euler update per frame.
% Inputs and real measurement reconstructions must share the same frame clock.
% No pose anchor is propagated, no inter-frame pose is generated, and absent
% measurements withdraw their own correction. The continuous gain certificate
% is retained as design context, not a proof of this sampled nonlinear system.
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
    alignment=struct('bodyOffset',[0;0],'bodyCovariance',zeros(2),'headingStdRad',0);
    if isfield(cfg.gnss,'outputPoint'),alignment=cfg.gnss.outputPoint;end
    continuous=designFullObserverGains(cfg);
    design=struct('kind',"synchronous-backward-euler",'continuousDesign',continuous, ...
        'sampledSystemCertified',false,'gainsRetuned',isfield(cfg.gnss,'positionGainDesign'));
    x=cfg.initialState;
    assert(numel(x)==7 && all(isfinite(x)),'VehicleLocalization:FullInitializationRequired', ...
        'Supply an explicit common initial state.');x=x(:);
    z=zeros(n,7);mode=zeros(n,1);headingMode=zeros(n,1);correction=zeros(n,4);
    yawCorrection=zeros(n,2);biasTrace=zeros(n,2);courseTrace=nan(n,1);
    gnssPosition=nan(n,2);gnssInformation=nan(2,2,n);
    margins=nan(n,1);weights=nan(n,2);gyro=0;rawIntegral=0;bodyIntegral=0;rotationIntegral=0;
    gh=history();lh=history();bias=0;targetBias=0;courseUpdates=0;biasUpdates=0;maximumRate=0;
    for k=1:n
        dt=0;if k>1,dt=t(k)-t(k-1);end
        speed=h.longitudinalSpeed(k);rawVy=lateral.lateralVelocity(k);
        part=participation(speed,cfg.bias.minimumSpeed);
        rate=h.yawRate(k);
        if k>1
            rate=(h.yawRate(k-1)+rate)/2;
            meanSpeed=(h.longitudinalSpeed(k-1)+speed)/2;
            meanVy=(lateral.lateralVelocity(k-1)+rawVy)/2;
            delta=rotationStep(gyro,rate,dt);
            rawIntegral=rawIntegral+delta*complex(meanSpeed,meanVy);
            bodyIntegral=bodyIntegral+delta*complex(meanSpeed+part*real(bias),meanVy+part*imag(bias));
            rotationIntegral=rotationIntegral+delta;gyro=gyro+rate*dt;
        end
        predYaw=x(7)+rate*dt;course=NaN;
        if G.valid(k)
            % Course history uses only the current predicted attitude. No
            % reference attitude or future observer state enters alignment.
            coursePosition=correctGnssOutputPoint(G.values(k,:),G.information(:,:,k),predYaw,alignment);
            gh=append(gh,t(k),[coursePosition,0],gyro,bodyIntegral,rotationIntegral,speed);
            [gh,first]=window(gh,t(k),cfg.gnss.courseWindow,cfg.gnss.maximumCourseGap,cfg.gnss.minimumSpeed);
            if ~isempty(first)
                displacement=complex(coursePosition(1)-gh.pose(first,1),coursePosition(2)-gh.pose(first,2));
                motion=exp(-1i*gyro)*(bodyIntegral-gh.integral(first));
                if min(abs([displacement,motion]))>=cfg.gnss.minimumCourseDisplacement
                    course=angle(displacement)-angle(motion);courseUpdates=courseUpdates+1;
                end
            end
        else
            gh=history();
        end
        if L.valid(k)
            lh=append(lh,t(k),L.values(k,:),gyro,rawIntegral,rotationIntegral,speed);
            [lh,first]=window(lh,t(k),cfg.bias.window,cfg.bias.maximumGap,cfg.bias.minimumSpeed);
            if cfg.bias.enabled && ~isempty(first)
                rotation=exp(1i*(lh.pose(first,3)-lh.gyro(first)));
                B=rotation*(rotationIntegral-lh.rotation(first));
                displacement=complex(L.values(k,1)-lh.pose(first,1),L.values(k,2)-lh.pose(first,2));
                if abs(B)>.7*(t(k)-lh.time(first))
                    candidate=(displacement-rotation*(rawIntegral-lh.integral(first)))/B;
                    if max(abs([real(candidate),imag(candidate)]))<=cfg.bias.maximumMagnitude
                        % The same displacement identifies both body-velocity
                        % discrepancies. Retain the longitudinal component
                        % instead of discarding wheel-speed bias evidence.
                        targetBias=candidate;biasUpdates=biasUpdates+1;
                    end
                end
            end
        else
            lh=history();
            % No new displacement evidence: hold the learned velocity bias.
            % Continuing toward the last noisy target during a long outage
            % would change the motion model without a new observation.
            targetBias=bias;
        end
        oldBias=bias;bias=targetBias+(bias-targetBias)*exp(-dt/cfg.bias.timeConstant);
        vx=speed+part*real(bias);vy=rawVy+part*imag(bias);betaRate=0;
        if dt>0
            betaRate=part/dt*(imag(bias-oldBias)*vx-real(bias-oldBias)*vy)/max(vx^2+vy^2,1);
        end
        q=h.yawRate(k)+lateral.sideSlipAngleRate(k)+betaRate;maximumRate=max(maximumRate,abs(q));
        Wl=weight(L,k,cfg.lidar.gainInformationScale,3);
        yaw=predYaw;
        if L.valid(k) && Wl(3,3)>=cfg.lidar.minimumPoseWeight
            gain=cfg.gains(4)*Wl(3,3);yaw=predYaw+dt*gain/(1+dt*gain)*wrap(L.values(k,3)-predYaw);
            headingMode(k)=2;yawCorrection(k,2)=gain*wrap(L.values(k,3)-yaw);
        elseif G.valid(k) && isfinite(course) && speed>=cfg.gnss.minimumSpeed
            for iteration=1:8
                yaw=yaw-(yaw-predYaw-dt*cfg.gnss.headingGain*sin(course-yaw))/ ...
                    (1+dt*cfg.gnss.headingGain*cos(course-yaw));
            end
            headingMode(k)=1;yawCorrection(k,1)=cfg.gnss.headingGain*sin(course-yaw);
        end
        Wg=zeros(2);
        if G.valid(k)
            [gnssPosition(k,:),gnssInformation(:,:,k)]=correctGnssOutputPoint(G.values(k,:),G.information(:,:,k),yaw,alignment);
            I=gnssInformation(:,:,k);Wg=I/(I+cfg.gnss.gainInformationScale*eye(2));Wg=(Wg+Wg.')/2;
        end
        K=cfg.gnss.positionGain*Wg+cfg.gains(1)*Wl(1:2,1:2);
        R=[cos(yaw),-sin(yaw);sin(yaw),cos(yaw)];J=[0,-1;1,0];
        velocity=R*[vx;vy];acceleration=R*[h.longitudinalAcceleration(k);h.lateralAcceleration(k)];
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
        biasTrace(k,:)=part*[real(bias),imag(bias)];courseTrace(k)=course;
    end
    estimate=struct('time',t,'z',z,'onlineZ',z,'pose',[z(:,[1,4]),wrap(z(:,7))], ...
        'position',z(:,[1,4]),'velocity',z(:,[2,5]),'acceleration',z(:,[3,6]), ...
        'heading',wrap(z(:,7)),'headingUnwrapped',z(:,7),'lateral',lateral,'observer',design);
    estimate.diagnostics=struct('mode',mode,'headingMode',headingMode,'positionCorrection',correction, ...
        'gnssPositionAtObserverPoint',gnssPosition,'gnssInformationAtObserverPoint',gnssInformation, ...
        'yawCorrection',yawCorrection,'lidarVelocityBias',biasTrace(:,2), ...
        'lidarLongitudinalVelocityBias',biasTrace(:,1),'gnssDerivedHeading',courseTrace, ...
        'motionBiasSource',"Past accepted LiDAR displacement versus wheel/gyro/lateral integration; held without LiDAR", ...
        'minimumWeights',weights,'translationDissipationMargin',margins,'gnssCourseUpdates',courseUpdates, ...
        'lidarBiasUpdates',biasUpdates,'maximumTrackAngleRate',maximumRate, ...
        'rateEnvelopeSatisfied',maximumRate<=cfg.maximumTrackAngleRate,'stateResets',0, ...
        'packetCounts',struct('gnssValid',nnz(G.valid),'gnssInvalid',nnz(~G.valid), ...
        'lidarValid',nnz(L.valid),'lidarInvalid',nnz(~L.valid)), ...
        'localizationUpdates',n-1,'virtualPoseUpdates',0,'integrationSubsteps',0, ...
        'nominalOutputRateHz',1/median(diff(t)),'referenceUsed',false, ...
        'allTheoremHypothesesVerified',false,'sampledSystemCertified',false, ...
        'scope',"One implicit update per synchronized frame; no measurement transport. Input alignment is a separate offline operation.");
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

function h=history()
    h=struct('time',zeros(0,1),'pose',zeros(0,3),'gyro',zeros(0,1), ...
        'integral',complex(zeros(0,1)),'rotation',complex(zeros(0,1)),'speed',zeros(0,1));
end

function h=append(h,time,pose,gyro,integral,rotation,speed)
    h.time(end+1,1)=time;h.pose(end+1,:)=pose;h.gyro(end+1,1)=gyro;
    h.integral(end+1,1)=integral;h.rotation(end+1,1)=rotation;h.speed(end+1,1)=speed;
end

function [h,first]=window(h,time,duration,gap,speed)
    keep=h.time>=time-duration-gap;
    for name=string(fieldnames(h)).',h.(name)=h.(name)(keep,:);end
    first=find(h.time<=time-duration,1,'last');
    if ~isempty(first) && (time-h.time(first)>duration+gap || any(diff(h.time(first:end))>gap) || min(h.speed(first:end))<speed)
        first=[];
    end
end

function p=participation(speed,minimum)
    a=min(1,max(0,(abs(speed)-1)/(minimum-1)));p=a^3*(10-15*a+6*a^2);
end

function value=rotationStep(yaw,rate,dt)
    half=rate*dt/2;factor=1;if abs(half)>1e-10,factor=sin(half)/half;end
    value=dt*factor*exp(1i*(yaw+half));
end

function x=wrap(x)
    x=atan2(sin(x),cos(x));
end
