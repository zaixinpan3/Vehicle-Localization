function result = simulateImprovedObserverScenario(observerDesign,lateralDesign,cfg)
% simulateImprovedObserverScenario Validate one continuous measurement mode.
% Constant-speed circular/straight Cartesian truth is analytic, including the
% prehistory. Smooth bounded sinusoidal measurement errors remain continuous.
% A nonempty lateralDesign runs the real upstream observer; struct() supplies
% exact lateral interfaces to isolate the seven-state global observer.
    arguments
        observerDesign (1,1) struct
        lateralDesign (1,1) struct = struct()
        cfg (1,1) struct = improvedObserverConfig()
    end
    t=(0:cfg.simulation.sampleTime:cfg.simulation.finalTime).';
    n=numel(t);v=cfg.simulation.speed;q=cfg.simulation.courseRate;
    z=zeros(n,7);
    for k=1:n,z(k,:)=trajectory(t(k),v,q,cfg.simulation.initialHeading).';end
    high=struct('time',t,'steeringAngle',zeros(n,1),'longitudinalSpeed',v*ones(n,1), ...
        'longitudinalAcceleration',zeros(n,1),'lateralAcceleration',v*q*ones(n,1), ...
        'yawRate',q*ones(n,1));
    if ~isempty(fieldnames(lateralDesign))
        wheelbase=lateralDesign.cfg.vehicle.lf+lateralDesign.cfg.vehicle.lr;
        high.steeringAngle(:)=atan2(q*wheelbase,max(v,eps));
    end
    data=struct('highRate',high);delay=cfg.measurement.fixedLidarDelay;
    if cfg.mode=="gnss"
        data.gnss=struct('evaluate',@(time) positionOutput(time,cfg));
    else
        data.lidar=struct('delay',delay,'headingConvention',"unwrapped", ...
            'evaluate',@(time) lidarOutput(time,cfg));
    end
    error=cfg.simulation.initialError;
    runCfg=cfg;runCfg.observer.initialState=z(1,:).'+error;
    initial=@(time) trajectory(time,v,q,cfg.simulation.initialHeading)+error;
    if isempty(fieldnames(lateralDesign))
        lateral=struct('time',t,'lateralVelocity',zeros(n,1), ...
            'sideSlipAngle',zeros(n,1),'sideSlipAngleRate',zeros(n,1));
        estimate=runImprovedVehicleObserver(data,struct(),observerDesign,runCfg, ...
            LateralInputs=lateral,InitialHistory=initial);
    else
        estimate=runImprovedVehicleObserver(data,lateralDesign,observerDesign,runCfg,InitialHistory=initial);
    end
    errorState=estimate.z-z;settled=t>=.5*cfg.simulation.finalTime;
    metrics=struct('positionRmse',sqrt(mean(sum(errorState(settled,[1,4]).^2,2))), ...
        'velocityRmse',sqrt(mean(sum(errorState(settled,[2,5]).^2,2))), ...
        'accelerationRmse',sqrt(mean(sum(errorState(settled,[3,6]).^2,2))), ...
        'headingRmse',sqrt(mean(errorState(settled,7).^2)), ...
        'maximumPositionError',max(vecnorm(errorState(:,[1,4]),2,2)), ...
        'maximumVelocityError',max(vecnorm(errorState(:,[2,5]),2,2)), ...
        'maximumAccelerationError',max(vecnorm(errorState(:,[3,6]),2,2)), ...
        'maximumHeadingError',max(abs(errorState(:,7))), ...
        'finalErrorNorm',norm(errorState(end,:)), ...
        'integrationStepCount',estimate.diagnostics.integrationStepCount);
    result=struct('truth',struct('time',t,'z',z,'position',z(:,[1,4]), ...
        'velocity',z(:,[2,5]),'acceleration',z(:,[3,6]),'heading',z(:,7)), ...
        'sensorData',data,'estimate',estimate,'metrics',metrics,'cfg',runCfg);
end

function z=trajectory(t,v,q,initialHeading)
    yaw=initialHeading+q*t;
    if q==0
        position=v*t*[cos(initialHeading);sin(initialHeading)];
    else
        position=(v/q)*[sin(yaw)-sin(initialHeading);cos(initialHeading)-cos(yaw)];
    end
    velocity=v*[cos(yaw);sin(yaw)];acceleration=v*q*[-sin(yaw);cos(yaw)];
    z=[position(1);velocity(1);acceleration(1);position(2);velocity(2);acceleration(2);yaw];
end

function y=positionOutput(t,cfg)
    z=trajectory(t,cfg.simulation.speed,cfg.simulation.courseRate,cfg.simulation.initialHeading);
    y=z([1,4])+cfg.simulation.positionNoiseAmplitude*[sin(1.3*t);cos(.9*t)];
end

function y=lidarOutput(t,cfg)
    z=trajectory(t-cfg.measurement.fixedLidarDelay,cfg.simulation.speed, ...
        cfg.simulation.courseRate,cfg.simulation.initialHeading);
    noise=[cfg.simulation.positionNoiseAmplitude*sin(1.3*t); ...
        cfg.simulation.positionNoiseAmplitude*cos(.9*t);cfg.simulation.headingNoiseAmplitude*sin(.7*t)];
    y=struct('pose',z([1,4,7])+noise,'information',1e6*eye(3));
end
