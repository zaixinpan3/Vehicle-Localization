function estimate=runMotionAidedVehicleObserver(data,lateral,cfg)
% runMotionAidedVehicleObserver Fuse continuous LiDAR and body motion outputs.
% State order is [X,Vx,Ax,Y,Vy,Ay,psi]. LiDAR position corrects position;
% measured body velocity and acceleration correct their corresponding states.
% The seven-state q^2*v+2*q*J*a prediction is retained, q=r+betaDot.
% Only declared aligned piecewise-linear, zero-delay inputs are supported.
% Full geometric information determines W. Its XY principal block and yaw
% diagonal act in separate cascade stages; XY-yaw feedback terms are omitted.
% This weighting is not a calibrated covariance or an independence claim.
% No reference pose, output blending, state reset or delayed history is used.
    arguments
        data (1,1) struct
        lateral (1,1) struct
        cfg (1,1) struct=motionAidedObserverConfig()
    end
    design=designMotionAidedObserverGains(cfg);
    assert(isscalar(cfg.maximumIntegrationStep) && isfinite(cfg.maximumIntegrationStep) ...
        && cfg.maximumIntegrationStep>0 && isfinite(cfg.lidar.gainInformationScale) ...
        && cfg.lidar.gainInformationScale>0, ...
        'VehicleLocalization:InvalidMotionConfiguration','Invalid integration step or information scale.');
    [t,u,p,weights]=inputs(data,lateral,cfg);n=numel(t);
    initial=cfg.initialState;
    if isempty(initial)
        R=[cos(p(1,3)),-sin(p(1,3));sin(p(1,3)),cos(p(1,3))];
        v=R*u(1,1:2).';a=R*u(1,3:4).';
        initial=[p(1,1);v(1);a(1);p(1,2);v(2);a(2);p(1,3)];
    end
    assert(isnumeric(initial) && isreal(initial) && numel(initial)==7 && all(isfinite(initial)), ...
        'VehicleLocalization:InvalidMotionInitialState','Initial state must contain seven finite values on the supplied yaw lift.');
    state=initial(:);z=zeros(n,7);z(1,:)=state.';steps=0;
    for k=2:n
        span=t(k)-t(k-1);pieces=max(1,ceil(span/cfg.maximumIntegrationStep-1e-10));dt=span/pieces;
        for j=1:pieces
            a=(j-1)/pieces;b=j/pieces;c=(a+b)/2;
            u1=(1-a)*u(k-1,:)+a*u(k,:);u2=(1-c)*u(k-1,:)+c*u(k,:);u3=(1-b)*u(k-1,:)+b*u(k,:);
            p1=(1-a)*p(k-1,:)+a*p(k,:);p2=(1-c)*p(k-1,:)+c*p(k,:);p3=(1-b)*p(k-1,:)+b*p(k,:);
            w1=(1-a)*weights(:,:,k-1)+a*weights(:,:,k);
            w2=(1-c)*weights(:,:,k-1)+c*weights(:,:,k);
            w3=(1-b)*weights(:,:,k-1)+b*weights(:,:,k);
            d1=rhs(state,u1,p1,w1,cfg.gains);
            d2=rhs(state+dt*d1/2,u2,p2,w2,cfg.gains);
            d3=rhs(state+dt*d2/2,u2,p2,w2,cfg.gains);
            d4=rhs(state+dt*d3,u3,p3,w3,cfg.gains);
            state=state+dt*(d1+2*d2+2*d3+d4)/6;steps=steps+1;
        end
        assert(all(isfinite(state)),'VehicleLocalization:NonfiniteObserver','Motion-aided integration became nonfinite.');
        z(k,:)=state.';
    end
    q=u(:,5)+u(:,6);heading=atan2(sin(z(:,7)),cos(z(:,7)));
    estimate=struct('time',t,'z',z,'pose',[z(:,[1,4]),heading], ...
        'position',z(:,[1,4]),'velocity',z(:,[2,5]),'acceleration',z(:,[3,6]), ...
        'heading',heading,'headingUnwrapped',z(:,7),'trackAngleRate',q,'lateral',lateral, ...
        'observer',design,'diagnostics',struct('integrationStepCount',steps, ...
        'maximumTrackAngleRate',max(abs(q)), ...
        'rateEnvelopeSatisfied',all(abs(q)<=cfg.maximumTrackAngleRate+1e-12), ...
        'delaySeconds',0,'measurementRepresentation',"piecewiseLinear", ...
        'weightReconstruction',"Linear interpolation of qualified knot weights", ...
        'referenceUsed',false,'stateResets',0,'unconditionalStabilityClaimed',false));
end

function d=rhs(x,u,p,W,g)
    c=cos(x(7));s=sin(x(7));v=[c*u(1)-s*u(2);s*u(1)+c*u(2)];
    a=[c*u(3)-s*u(4);s*u(3)+c*u(4)];q=u(5)+u(6);
    d=zeros(7,1);d([1,4])=x([2,5])+g(1)*W(1:2,1:2)*(p(1:2).'-x([1,4]));
    d([2,5])=x([3,6])+g(2)*(v-x([2,5]));
    d([3,6])=q^2*x([2,5])+2*q*[-x(6);x(3)]+g(3)*(a-x([3,6]));
    d(7)=u(5)+g(4)*W(3,3)*(p(3)-x(7));
end

function [t,u,p,W]=inputs(data,lateral,cfg)
    assert(isfield(data,'highRate') && isfield(data,'lidar'), ...
        'VehicleLocalization:InvalidMotionInputs','Motion and LiDAR inputs are required.');
    h=data.highRate;source=data.lidar;
    assert(isfield(h,'time') && isnumeric(h.time) && isreal(h.time) && isvector(h.time) ...
        && numel(h.time)>=2 && all(isfinite(h.time)) && all(diff(h.time)>0), ...
        'VehicleLocalization:InvalidMotionInputs','Input times must be finite and increasing.');
    t=h.time(:);n=numel(t);
    assert(isfield(lateral,'time') && isequal(lateral.time(:),t), ...
        'VehicleLocalization:InvalidLateralInputs','Lateral samples must be aligned.');
    names=["longitudinalSpeed","longitudinalAcceleration","lateralAcceleration","yawRate"];
    for name=names
        assert(isfield(h,name) && isnumeric(h.(name)) && isreal(h.(name)) && isvector(h.(name)) ...
            && numel(h.(name))==n && all(isfinite(h.(name))), ...
            'VehicleLocalization:InvalidMotionInputs','Motion measurements must be finite aligned vectors.');
    end
    for name=["lateralVelocity","sideSlipAngleRate"]
        assert(isfield(lateral,name) && isnumeric(lateral.(name)) && isreal(lateral.(name)) ...
            && isvector(lateral.(name)) && numel(lateral.(name))==n && all(isfinite(lateral.(name))), ...
            'VehicleLocalization:InvalidLateralInputs','Lateral measurements must be finite aligned vectors.');
    end
    assert(isfield(source,'delay') && isequal(source.delay,0), ...
        'VehicleLocalization:ZeroDelayRequired','This design requires declared zero LiDAR delay.');
    assert(isfield(source,'representation') && string(source.representation)=="piecewiseLinear" ...
        && isfield(source,'headingConvention') && string(source.headingConvention)=="unwrapped" ...
        && isfield(source,'time') && isequal(source.time(:),t) ...
        && isfield(source,'pose') && isnumeric(source.pose) && isreal(source.pose) ...
        && isequal(size(source.pose),[n,3]) && all(isfinite(source.pose),'all') ...
        && isfield(source,'information') && isequal(size(source.information),[3,3,n]), ...
        'VehicleLocalization:InvalidContinuousSignal','Declare aligned finite pose, information and unwrapped heading.');
    u=[h.longitudinalSpeed(:),lateral.lateralVelocity(:),h.longitudinalAcceleration(:), ...
        h.lateralAcceleration(:),h.yawRate(:),lateral.sideSlipAngleRate(:)];p=source.pose;
    W=zeros(3,3,n);
    for k=1:n
        [~,~,audit,W(:,:,k)]=computeLidarInformationWeights(source.information(:,:,k),cfg);
        assert(audit.qualified,'VehicleLocalization:InsufficientLidarInformation', ...
            'Every continuous reconstruction knot must satisfy the declared information sector.');
    end
end
