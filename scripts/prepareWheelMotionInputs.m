function [high,wheel,metadata]=prepareWheelMotionInputs(sensorFolder,parameters,clock,startTime,endTime,options)
% prepareWheelMotionInputs Build motion inputs using four-wheel speed only.
% clock contains increasing rosTime and receiverTime columns in seconds.
% startTime/endTime use that receiver clock. Wheel fields FL/FR/RL/RR are
% rad/s. Steering and corrected IMU are linearly reconstructed offline;
% wheel packets enter the causal wheel estimator at their native times.
% Missing or expired wheel data cause an error, never a speed-source fallback.
    arguments
        sensorFolder (1,1) string
        parameters (1,1) struct
        clock (1,1) struct
        startTime (1,1) double {mustBeFinite}
        endTime (1,1) double {mustBeFinite}
        options.InitialSpeed (1,1) double = NaN
    end
    path=fullfile(sensorFolder,'wheel_speed_report.csv');
    assert(isfile(path),'VehicleLocalization:MissingWheelSpeed', ...
        'Four-wheel wheel_speed_report.csv is required; export /vehicle/wheel_speed_report.');
    wh=readtable(path);imu=readtable(fullfile(sensorFolder,'imu.csv'));
    steering=readtable(fullfile(sensorFolder,'steering.csv'));
    assert(numel(clock.rosTime)>=2 && numel(clock.rosTime)==numel(clock.receiverTime) && ...
        all(isfinite(clock.rosTime)) && all(isfinite(clock.receiverTime)) && ...
        all(diff(clock.rosTime)>0) && all(diff(clock.receiverTime)>0) && endTime>startTime, ...
        'VehicleLocalization:InvalidMotionClock','Require increasing finite clocks and interval.');
    fields={'front_left','front_right','rear_left','rear_right'};
    assert(all(ismember([{'stamp_sec'},fields],wh.Properties.VariableNames)) && height(wh)>=1, ...
        'VehicleLocalization:InvalidWheelInput','Require timestamps and all four wheel columns.');
    origin=clock.rosTime(1);
    bridge=@(stamp) interp1(clock.rosTime-origin,clock.receiverTime,stamp-origin,'linear','extrap')-startTime;
    wt=bridge(wh.stamp_sec);it=bridge(imu.stamp_sec);st=bridge(steering.stamp_sec);
    assert(all(diff(it)>0) && all(diff(st)>0) && max(it(1),st(1))<=0, ...
        'VehicleLocalization:MotionCoverage','IMU and steering must cover the start.');
    last=min([endTime-startTime,it(end),st(end)]);t=(0:.01:last).';
    assert(numel(t)>=2,'VehicleLocalization:MotionCoverage','Motion interval is too short.');
    high=struct('time',t);offset=0;
    if isfield(parameters,'steeringWheelOffsetRad'),offset=parameters.steeringWheelOffsetRad;end
    high.steeringAngle=(interp1(st,steering.steering_wheel_angle_rad,t,'linear')-offset)/parameters.steeringRatio;
    raw=["acceleration_x_mps2","acceleration_y_mps2","angular_z_radps"];
    names=["longitudinalAcceleration","lateralAcceleration","yawRate"];
    for k=1:3
        correction=parameters.input_correction.(names(k));
        high.(names(k))=correction.sign*interp1(it,imu.(raw(k)),t,'linear')+correction.offset;
    end
    input=high;input.wheels=struct('time',wt,'angularVelocity',wh{:,fields});
    cfg=wheelSpeedObserverConfig();cfg.initialSpeed=options.InitialSpeed;
    wheel=estimateWheelLongitudinalSpeed(input,cfg);
    initializing=isfinite(options.InitialSpeed) & t<wt(1) & t<=cfg.maximumWheelAge;
    assert(all(wheel.valid | initializing),'VehicleLocalization:WheelSpeedCoverage', ...
        'Wheel aiding is unavailable beyond the declared startup/maximum-age limit.');
    assert(all(isfinite(wheel.longitudinalSpeed)),'VehicleLocalization:WheelSpeedCoverage','Nonfinite wheel velocity.');
    high.longitudinalSpeed=max(0,wheel.longitudinalSpeed);
    metadata=struct('source',"/vehicle/wheel_speed_report: four-wheel fusion with steering/IMU", ...
        'wheelFile',path,'wheelPackets',numel(wt),'configuration',cfg, ...
        'initialSpeed',options.InitialSpeed,'initialPredictionOnlySamples',nnz(initializing & ~wheel.valid), ...
        'maximumAcceptedWheelAge',max(wheel.sourceAge(wheel.valid)), ...
        'forwardOnly',true,'referenceVelocityUsed',false,'alternativeSpeedFallback',false, ...
        'upstreamReconstruction',"Offline linear IMU/steering reconstruction; native causal wheel packets");
end
