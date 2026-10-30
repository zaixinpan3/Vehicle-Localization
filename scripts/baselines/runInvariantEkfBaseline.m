function estimate=runInvariantEkfBaseline(data,lateral,cfg,options)
% Same intrinsic state/predictor and five outputs, with covariance feedback.
% LiDAR is a local quadratic overlap likelihood, not a Gaussian pose sensor.
% Its temperature is a tunable empirical likelihood scale. Shared primitive
% noise columns preserve squared-speed/dot/cross correlations.
    arguments
        data struct
        lateral struct
        cfg struct
        options.LidarTemperature double=1
        options.ProcessScale double=1
        options.Iterations double=1
        options.PoseProvider=[]
    end
    t=data.highRate.time;n=numel(t);x=cfg.initialState(:);if numel(x)==7,x(8)=0;end
    P=diag([.5,.5,1,.5,.5,1,deg2rad(2),.02].^2);Z=nan(n,8);seconds=zeros(n,1);
    G=zeros(3,8);G(:,[1,4,7])=eye(3);noise=cfg.invariantObserver.noise;
    for k=1:n
        timer=tic;dt=0;if k>1,dt=t(k)-t(k-1);end
        valid=lateral.velocityValid(k);br=0;if valid,br=lateral.sideSlipAngleRate(k);end
        old=x;x=invariantObserverSupport.predict(old,dt,br);
        F=zeros(8);for j=1:8,d=1e-5*max(1,abs(old(j)));a=old;a(j)=a(j)+d;b=old;b(j)=b(j)-d;F(:,j)=(invariantObserverSupport.predict(a,dt,br)-invariantObserverSupport.predict(b,dt,br))/(2*d);end
        Q=diag([1e-6,1e-4,noise.jerkMps3^2,1e-6,1e-4,noise.jerkMps3^2,1e-7,.03^2]);
        P=F*P*F.'+options.ProcessScale*dt*Q;
        if isfield(data,'gnss') && data.gnss.valid(k)
            [z,I]=fullObserverSupport.correctGnssOutputPoint(data.gnss.position(k,:),data.gnss.information(:,:,k),x(7),cfg.gnss.outputPoint);
            H=zeros(2,8);H(:,[1,4])=eye(2);[x,P]=update(x,P,z.'-H*x,H,I\eye(2));
        end
        h=data.highRate;beta=0;vy=0;if valid,beta=lateral.sideSlipAngle(k);vy=lateral.lateralVelocity(k);end
        av=true;if isfield(h,'bodyAccelerationValid'),av=h.bodyAccelerationValid(k);end
        packet=invariantObserverSupport.measurements(h.longitudinalSpeed(k),vy,h.longitudinalAcceleration(k),h.lateralAcceleration(k),h.yawRate(k),beta,valid,av,true,cfg);
        [v,H]=invariantObserverSupport.outputs(x,beta,cfg);
        B=invariantIssSupport.measurementSecant([h.longitudinalSpeed(k);vy],[h.longitudinalAcceleration(k);h.lateralAcceleration(k)],[h.longitudinalSpeed(k);vy],[h.longitudinalAcceleration(k);h.lateralAcceleration(k)]);
        R=B*diag([noise.velocityMps^2*[1,1],noise.accelerationMps2^2*[1,1],noise.gyroRadps^2])*B.';
        R(4,4)=noise.velocityMps^2;R=R+1e-8*eye(5);ids=packet.valid;
        [x,P]=update(x,P,packet.value(ids)-v(ids),H(ids,:),R(ids,ids));
        if ~isempty(options.PoseProvider) && dt>0
            measurement=options.PoseProvider(k,x([1,4,7]));
            if ~isempty(measurement)
                U=measurement.basis;residual=measurement.pose-x([1,4,7]);residual(3)=atan2(sin(residual(3)),cos(residual(3)));
                [x,P]=update(x,P,U.'*residual,U.'*G,U.'*measurement.covariance*U);
            end
        elseif isfield(data,'lidarOverlap') && data.lidarOverlap.valid(k) && dt>0
            prior=x;Pi=P\eye(8);current=x;measurement=[];
            for it=1:options.Iterations
                measurement=lidarInjectionSupport.evaluateOverlapGradient(data.lidarOverlap.map,data.lidarOverlap.sources{k},current([1,4,7]),data.lidarOverlap.config);
                if ~measurement.available,break;end
                Hq=measurement.information/options.LidarTemperature;g=measurement.gradient/options.LidarTemperature;
                next=current-(Pi+G.'*Hq*G)\(Pi*(current-prior)+G.'*g);
                % Bound iteration size, preserving local tracking initialization.
                delta=next-current;ratio=max([1,norm(delta([1,4]))/1,abs(delta(7))/.1]);next=current+delta/ratio;
                if norm(next-current)<1e-6,current=next;break;end
                current=next;
            end
            if ~isempty(measurement) && measurement.available,x=current;P=(Pi+G.'*Hq*G)\eye(8);end
        end
        P=(P+P.')/2;Z(k,:)=x.';seconds(k)=toc(timer);
        if any(~isfinite(x)) || norm(x([2,5]))>100,break;end
    end
    estimate=struct('pose',Z(:,[1,4,7]),'z',Z,'seconds',seconds);
end
function [x,P]=update(x,P,r,H,R)
    K=(P*H.')/(H*P*H.'+R);x=x+K*r;A=eye(8)-K*H;P=A*P*A.'+K*R*K.';
end
