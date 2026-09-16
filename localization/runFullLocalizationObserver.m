function estimate=runFullLocalizationObserver(data,lateralDesign,cfg,options)
% runFullLocalizationObserver Inject available GNSS and LiDAR simultaneously.
% highRate and optional LateralInputs are sampled, left-held input streams.
% data.gnss: time, position (N-by-2), information (2-by-2-by-N), valid, delay=0.
% data.lidar: time, pose (N-by-3), information (3-by-3-by-N), valid, delay=0.
% Missing source, invalid packet, and expired packet cause zero injection.
% Invalid payloads may be NaN; invalidation immediately withdraws older data.
% Pose anchors propagate using past/current body motion and gyro only. There
% is no future-frame interpolation, pose reset, or reference-state input.
% GNSS heading is reconstructed from past position displacement and integrated
% body motion, and used only without qualified LiDAR yaw. At insufficient
% motion it is disabled, rather than claiming stationary yaw observability.
    arguments
        data (1,1) struct
        lateralDesign (1,1) struct
        cfg (1,1) struct=fullObserverConfig()
        options.LateralInputs (1,1) struct=struct()
    end
    design=designFullObserverGains(cfg);validateConfig(cfg);
    h=data.highRate;t=h.time(:);n=numel(t);
    assert(n>=2 && all(isfinite(t)) && all(diff(t)>0), ...
        'VehicleLocalization:InvalidFullInput','Require increasing finite motion times.');
    names=["longitudinalSpeed","longitudinalAcceleration","lateralAcceleration","yawRate"];
    for name=names
        assert(isfield(h,name) && numel(h.(name))==n && all(isfinite(h.(name))), ...
            'VehicleLocalization:InvalidFullInput','Require finite aligned motion.');
    end
    lateral=options.LateralInputs;
    if isempty(fieldnames(lateral))
        lateral=runLateralVelocityObserver(h,lateralDesign,lateralDesign.cfg);
    end
    assert(isequal(lateral.time(:),t),'VehicleLocalization:InvalidFullInput','Lateral time must match motion time.');
    for name=["lateralVelocity","sideSlipAngleRate"]
        assert(numel(lateral.(name))==n && all(isfinite(lateral.(name))), ...
            'VehicleLocalization:InvalidFullInput','Require finite aligned lateral outputs.');
    end
    G=normalize(data,'gnss',2);L=normalize(data,'lidar',3);
    % Every event/expiry is an integration boundary; output times stay fixed.
    cuts=unique([t;G.time;L.time;G.time+cfg.gnss.maximumAge;L.time+cfg.lidar.maximumAge]);
    cuts=cuts(cuts>=t(1) & cuts<=t(end));
    assert(all(G.time>=t(1)) && all(L.time>=t(1)), ...
        'VehicleLocalization:InvalidFullInput','Trim pre-start packets explicitly.');
    x=cfg.initialState;
    if isempty(x)
        [p,yaw]=initialPose(G,L,t(1),cfg.initialHeading);
        v=rotate(yaw,[h.longitudinalSpeed(1);lateral.lateralVelocity(1)]);
        a=rotate(yaw,[h.longitudinalAcceleration(1);h.lateralAcceleration(1)]);
        x=[p(1);v(1);a(1);p(2);v(2);a(2);yaw];
    end
    assert(isnumeric(x) && numel(x)==7 && all(isfinite(x)), ...
        'VehicleLocalization:InvalidFullInitialState','Supply a finite seven-state initial condition.');
    x=x(:);ig=1;il=1;ih=1;io=1;g=emptyAnchor(2);l=emptyAnchor(3);
    gyro=0;bodyIntegral=0;rawIntegral=0;rotationIntegral=0;
    gh=emptyHistory();lh=emptyHistory();targetBias=0;bias=0;
    z=zeros(n,7);mode=zeros(n,1);headingMode=zeros(n,1);ages=nan(n,2);
    positionCorrection=zeros(n,4);yawCorrection=zeros(n,2);biasTrace=zeros(n,1);
    stateBiasUpdates=0;courseUpdates=0;steps=0;maxRate=0;
    counts=struct('gnssValid',0,'gnssInvalid',0,'lidarValid',0,'lidarInvalid',0);
    minWeight=nan(n,2);courseTrace=nan(n,1);translationMargin=nan(n,1);
    for k=1:numel(cuts)
        now=cuts(k);
        while ih<n && t(ih+1)<=now,ih=ih+1;end
        rawVy=lateral.lateralVelocity(ih);
        speed=h.longitudinalSpeed(ih);
        participation=smoothParticipation(speed,cfg.bias.minimumSpeed);
        vy=rawVy+participation*bias;
        while ig<=numel(G.time) && G.time(ig)<=now
            if G.valid(ig)
                counts.gnssValid=counts.gnssValid+1;
                g=anchor(G,ig,x(7),cfg.gnss.gainInformationScale);
                gh=appendHistory(gh,now,G.values(ig,:),gyro,bodyIntegral,rotationIntegral,speed);
                gh=trimHistory(gh,now,cfg.gnss.courseWindow,cfg.gnss.maximumCourseGap);
                [ok,first]=coveredWindow(gh,cfg.gnss.courseWindow,cfg.gnss.maximumCourseGap,cfg.gnss.minimumSpeed);
                if ok
                    displacement=complex(g.pose(1)-gh.position(first,1),g.pose(2)-gh.position(first,2));
                    bodyDisplacement=exp(-1i*gyro)*(bodyIntegral-gh.integral(first));
                    if min(abs([displacement,bodyDisplacement]))>=cfg.gnss.minimumCourseDisplacement
                        g.courseYaw=angle(displacement)-angle(bodyDisplacement);
                        g.courseValid=true;courseUpdates=courseUpdates+1;
                    end
                end
            else
                g=emptyAnchor(2);gh=emptyHistory();counts.gnssInvalid=counts.gnssInvalid+1;
            end
            ig=ig+1;
        end
        while il<=numel(L.time) && L.time(il)<=now
            if L.valid(il)
                counts.lidarValid=counts.lidarValid+1;
                l=anchor(L,il,L.values(il,3),cfg.lidar.gainInformationScale);
                lh=appendHistory(lh,now,L.values(il,:),gyro,rawIntegral,rotationIntegral,speed);
                lh=trimHistory(lh,now,cfg.bias.window,cfg.bias.maximumGap);
                [ok,first]=coveredWindow(lh,cfg.bias.window,cfg.bias.maximumGap,cfg.bias.minimumSpeed);
                if ok && cfg.bias.enabled
                    rotation=exp(1i*(lh.position(first,3)-lh.gyro(first)));
                    B=rotation*(rotationIntegral-lh.rotation(first));
                    displacement=complex(l.pose(1)-lh.position(first,1),l.pose(2)-lh.position(first,2));
                    if abs(B)>.7*(now-lh.time(first))
                        candidate=(displacement-rotation*(rawIntegral-lh.integral(first)))/B;
                        if max(abs([real(candidate),imag(candidate)]))<=cfg.bias.maximumMagnitude
                            targetBias=imag(candidate);stateBiasUpdates=stateBiasUpdates+1;
                        end
                    end
                end
            else
                l=emptyAnchor(3);lh=emptyHistory();counts.lidarInvalid=counts.lidarInvalid+1;
            end
            il=il+1;
        end
        if now>=g.time+cfg.gnss.maximumAge,g.active=false;g.courseValid=false;end
        if now>=l.time+cfg.lidar.maximumAge,l.active=false;end
        u=[speed,vy,h.longitudinalAcceleration(ih),h.lateralAcceleration(ih),h.yawRate(ih),lateral.sideSlipAngleRate(ih)];
        if io<=n && now==t(io)
            [~,details]=rhs(x,u,g,l,cfg);
            z(io,:)=x.';mode(io)=double(g.active)+2*double(l.active);
            headingMode(io)=details.headingMode;positionCorrection(io,:)=details.position;
            yawCorrection(io,:)=details.yaw;ages(io,:)=[age(g,now),age(l,now)];
            biasTrace(io)=participation*bias;minWeight(io,:)=[weightMinimum(g),weightMinimum(l)];
            gain=zeros(2);if g.active,gain=gain+cfg.gnss.positionGain*g.weight;end
            if l.active,gain=gain+cfg.gains(1)*l.weight(1:2,1:2);end
            alpha=min(eig(gain));q2=cfg.maximumTrackAngleRate^2;
            translationMargin(io)=min(eig([2*alpha,-1,0;-1,2*cfg.gains(2),-(1+q2);0,-(1+q2),2*cfg.gains(3)]));
            if g.courseValid,courseTrace(io)=g.courseYaw;end
            io=io+1;
        end
        if k==numel(cuts),break;end
        span=cuts(k+1)-now;pieces=max(1,ceil(span/cfg.maximumIntegrationStep-1e-10));dt=span/pieces;
        for j=1:pieces
            oldBias=bias;bias=targetBias+(bias-targetBias)*exp(-dt/cfg.bias.timeConstant);
            meanBias=(oldBias+bias)/2;vy=rawVy+participation*meanBias;
            betaRate=participation*(bias-oldBias)/dt*speed/max(speed^2+vy^2,1);
            u=[speed,vy,h.longitudinalAcceleration(ih),h.lateralAcceleration(ih),h.yawRate(ih),lateral.sideSlipAngleRate(ih)+betaRate];
            gm=propagate(g,u,dt/2);lm=propagate(l,u,dt/2);
            ge=propagate(g,u,dt);le=propagate(l,u,dt);
            d1=rhs(x,u,g,l,cfg);d2=rhs(x+dt*d1/2,u,gm,lm,cfg);
            d3=rhs(x+dt*d2/2,u,gm,lm,cfg);d4=rhs(x+dt*d3,u,ge,le,cfg);
            x=x+dt*(d1+2*d2+2*d3+d4)/6;g=ge;l=le;
            delta=rotationStep(gyro,u(5),dt);
            bodyIntegral=bodyIntegral+delta*complex(speed,vy);
            rawIntegral=rawIntegral+delta*complex(speed,rawVy);
            rotationIntegral=rotationIntegral+delta;gyro=gyro+dt*u(5);
            maxRate=max(maxRate,abs(u(5)+u(6)));steps=steps+1;
        end
        assert(all(isfinite(x)),'VehicleLocalization:NonfiniteObserver','Full observer became nonfinite.');
    end
    heading=atan2(sin(z(:,7)),cos(z(:,7)));
    estimate=struct('time',t,'z',z,'onlineZ',z,'pose',[z(:,[1,4]),heading], ...
        'position',z(:,[1,4]),'velocity',z(:,[2,5]),'acceleration',z(:,[3,6]), ...
        'heading',heading,'headingUnwrapped',z(:,7),'lateral',lateral,'observer',design);
    estimate.diagnostics=struct('mode',mode,'headingMode',headingMode,'sourceAge',ages, ...
        'positionCorrection',positionCorrection,'yawCorrection',yawCorrection, ...
        'minimumWeights',minWeight,'gnssDerivedHeading',courseTrace,'lidarVelocityBias',biasTrace, ...
        'translationDissipationMargin',translationMargin, ...
        'gnssCourseUpdates',courseUpdates,'lidarBiasUpdates',stateBiasUpdates,'packetCounts',counts, ...
        'integrationSteps',steps,'maximumTrackAngleRate',maxRate, ...
        'rateEnvelopeSatisfied',maxRate<=cfg.maximumTrackAngleRate,'stateResets',0, ...
        'referenceUsed',false,'futureMeasurementInterpolation',false, ...
        'unconditionalIssClaimed',false,'allTheoremHypothesesVerified',false, ...
        'scope',"Causal sampled runtime from supplied motion/lateral streams; upstream preprocessing, map and matching may still use offline reference information.");
end

function [d,audit]=rhs(x,u,g,l,cfg)
    v=rotate(x(7),u(1:2).');a=rotate(x(7),u(3:4).');q=u(5)+u(6);
    cg=zeros(2,1);cl=cg;yg=0;yl=0;headingMode=0;
    if g.active,cg=cfg.gnss.positionGain*g.weight*(g.pose(1:2)-x([1,4]));end
    lidarYaw=l.active && l.weight(3,3)>=cfg.lidar.minimumPoseWeight;
    if l.active
        cl=cfg.gains(1)*l.weight(1:2,1:2)*(l.pose(1:2)-x([1,4]));
        if lidarYaw,yl=cfg.gains(4)*l.weight(3,3)*wrap(l.pose(3)-x(7));headingMode=2;end
    end
    if g.active && g.courseValid && ~lidarYaw && u(1)>=cfg.gnss.minimumSpeed
        yg=cfg.gnss.headingGain*sin(g.courseYaw-x(7));headingMode=1;
    end
    d=zeros(7,1);d([1,4])=x([2,5])+cg+cl;
    d([2,5])=x([3,6])+cfg.gains(2)*(v-x([2,5]));
    d([3,6])=q^2*x([2,5])+2*q*[-x(6);x(3)]+cfg.gains(3)*(a-x([3,6]));
    d(7)=u(5)+yl+yg;
    if nargout>1,audit=struct('position',[cg.',cl.'],'yaw',[yg,yl],'headingMode',headingMode);end
end

function source=normalize(data,name,width)
    source=struct('time',zeros(0,1),'values',zeros(0,width),'information',zeros(width,width,0),'valid',false(0,1));
    if ~isfield(data,name) || isempty(fieldnames(data.(name))),return;end
    s=data.(name);field='pose';if width==2,field='position';end
    assert(isfield(s,'delay') && isequal(s.delay,0),'VehicleLocalization:FullZeroDelayRequired','Declare zero processing delay.');
    assert(isfield(s,'time') && isfield(s,field) && isfield(s,'valid') && isfield(s,'information'), ...
        'VehicleLocalization:InvalidFullSource','Missing source fields.');
    time=s.time(:);n=numel(time);valid=s.valid(:);
    assert(all(isfinite(time)) && all(diff(time)>0) && isequal(size(s.(field)),[n,width]) ...
        && numel(valid)==n && all(ismember(valid,[0,1])) && size(s.information,1)==width ...
        && size(s.information,2)==width && size(s.information,3)==n, ...
        'VehicleLocalization:InvalidFullSource','Malformed source dimensions/timestamps.');
    source=struct('time',time,'values',s.(field),'information',s.information,'valid',logical(valid));
    for k=find(valid).'
        I=s.information(:,:,k);
        assert(all(isfinite(source.values(k,:))) && all(isfinite(I),'all') ...
            && norm(I-I.','fro')<=1e-9*max(1,norm(I,'fro')) && min(eig((I+I.')/2))>0, ...
            'VehicleLocalization:InvalidFullSource','Valid packets require finite full-information measurements.');
    end
end

function a=anchor(source,k,yaw,scale)
    width=size(source.values,2);a=emptyAnchor(width);a.active=true;a.time=source.time(k);
    a.pose=[source.values(k,1:2).';yaw];I=source.information(:,:,k);I=(I+I.')/2;
    a.weight=I/(I+scale*eye(width));a.weight=(a.weight+a.weight.')/2;
end

function a=emptyAnchor(width)
    a=struct('active',false,'time',-Inf,'pose',zeros(3,1),'weight',zeros(width), ...
        'courseValid',false,'courseYaw',0);
end

function a=propagate(a,u,dt)
    if ~a.active,return;end
    displacement=rotationStep(a.pose(3),u(5),dt)*complex(u(1),u(2));
    a.pose(1:2)=a.pose(1:2)+[real(displacement);imag(displacement)];
    a.pose(3)=a.pose(3)+dt*u(5);a.courseYaw=a.courseYaw+dt*u(5);
end

function value=rotationStep(yaw,rate,dt)
    half=rate*dt/2;factor=1;if abs(half)>1e-10,factor=sin(half)/half;end
    value=dt*factor*exp(1i*(yaw+half));
end

function h=emptyHistory()
    h=struct('time',zeros(0,1),'position',zeros(0,3),'gyro',zeros(0,1), ...
        'integral',complex(zeros(0,1)),'rotation',complex(zeros(0,1)),'speed',zeros(0,1));
end

function h=appendHistory(h,time,position,gyro,integral,rotation,speed)
    h.time(end+1,1)=time;h.position(end+1,:)=zeros(1,3);h.position(end,1:numel(position))=position;
    h.gyro(end+1,1)=gyro;h.integral(end+1,1)=integral;h.rotation(end+1,1)=rotation;h.speed(end+1,1)=speed;
end

function h=trimHistory(h,time,window,gap)
    keep=h.time>=time-window-gap;
    for name=string(fieldnames(h)).',h.(name)=h.(name)(keep,:);end
end

function [ok,first]=coveredWindow(h,window,gap,speed)
    first=find(h.time<=h.time(end)-window,1,'last');ok=false;
    if isempty(first),return;end
    ok=h.time(end)-h.time(first)<=window+gap && all(diff(h.time(first:end))<=gap) ...
        && min(h.speed(first:end))>=speed;
end

function [p,yaw]=initialPose(G,L,time,yaw)
    g=find(G.time==time & G.valid,1);l=find(L.time==time & L.valid,1);
    assert(~isempty(g) || ~isempty(l),'VehicleLocalization:FullInitializationRequired', ...
        'Supply an initial state when no valid position is available at start.');
    if ~isempty(l),p=L.values(l,1:2);yaw=L.values(l,3);else,p=G.values(g,:);end
end

function p=rotate(yaw,v)
    p=[cos(yaw),-sin(yaw);sin(yaw),cos(yaw)]*v;
end

function x=wrap(x)
    x=atan2(sin(x),cos(x));
end

function x=age(a,time)
    x=NaN;if a.active,x=time-a.time;end
end

function x=weightMinimum(a)
    x=NaN;if a.active,x=min(eig(a.weight));end
end

function p=smoothParticipation(speed,minimum)
    x=min(1,max(0,(abs(speed)-1)/(minimum-1)));p=x^3*(10-15*x+6*x^2);
end

function validateConfig(cfg)
    values=[cfg.maximumIntegrationStep,cfg.gnss.gainInformationScale,cfg.gnss.maximumAge, ...
        cfg.gnss.courseWindow,cfg.gnss.maximumCourseGap,cfg.gnss.minimumCourseDisplacement, ...
        cfg.gnss.minimumSpeed,cfg.lidar.maximumAge,cfg.bias.window,cfg.bias.maximumGap, ...
        cfg.bias.timeConstant,cfg.bias.maximumMagnitude];
    assert(all(isfinite(values)) && all(values>0) && isfinite(cfg.initialHeading) ...
        && isscalar(cfg.bias.enabled) && cfg.bias.minimumSpeed>1, ...
        'VehicleLocalization:InvalidFullConfig','Invalid full-observer settings.');
end
