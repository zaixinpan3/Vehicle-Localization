function inputs=prepareDowntownMotionInputs(datasetFolder,lateralDesign)
% prepareDowntownMotionInputs Receiver-free measured wheel/DBW-IMU adapter.
% Native sensor timing was associated through raw Ouster packets in Python.
% Current MnCAV vehicle, steering, wheel and fixed IMU corrections are loaded.
% No mapping/reference trajectory or GNSS file is opened.
    parameters=mncavReplayConfig();folder=fullfile(datasetFolder,'sensors');
    imu=readtable(fullfile(folder,'imu.csv'));steering=readtable(fullfile(folder,'steering.csv'));wh=readtable(fullfile(folder,'wheel_speed_report.csv'));
    begin=max([imu.native_time_sec(1),steering.native_time_sec(1),wh.native_time_sec(1)]);
    finish=min([imu.native_time_sec(end),steering.native_time_sec(end),wh.native_time_sec(end)]);
    assert(finish>begin && all(diff(imu.native_time_sec)>0) && all(diff(steering.native_time_sec)>0));
    protocol=downtownReferenceReplayConfig();step=protocol.motionStepSeconds;
    t=(0:step:finish-begin).';h=struct('time',t);
    h.steeringAngle=(interp1(steering.native_time_sec-begin,steering.steering_wheel_angle_rad,t)-parameters.steeringWheelOffsetRad)/parameters.steeringRatio;
    for pair=["longitudinalAcceleration","lateralAcceleration","yawRate";"acceleration_x_mps2","acceleration_y_mps2","angular_z_radps"]
        correction=parameters.input_correction.(pair(1));h.(pair(1))=correction.sign*interp1(imu.native_time_sec-begin,imu.(pair(2)),t)+correction.offset;
    end
    input=h;input.wheels=struct('time',wh.native_time_sec-begin,'angularVelocity',wh{:,{'front_left','front_right','rear_left','rear_right'}});
    wheel=estimateWheelLongitudinalSpeed(input,wheelSpeedObserverConfig());
    assert(all(wheel.valid) && all(isfinite(wheel.longitudinalSpeed)),'VehicleLocalization:WheelSpeedCoverage','No speed fallback is allowed.');
    h.longitudinalSpeed=max(0,wheel.longitudinalSpeed);
    cfg=lateralObserverConfig();lateralObserverSupport.assertLateralVehicleMatches(lateralDesign,cfg);
    root=fileparts(fileparts(mfilename('fullpath')));alignment=jsondecode(fileread(fullfile(root,'config','offlineReferenceFrontLidarMotion.json')));
    lever=alignment.leverLidarToInsMeters(:);r=h.yawRate;rdot=gradient(r,t);
    velocityMap=repmat([1,0,0,1,0,0],numel(t),1);
    velocityMap(:,5:6)=[lever(2)*r,-lever(1)*r];
    lateral=runLateralVelocityObserver(h,lateralDesign,cfg,velocityMap);
    % Existing calibrated output point is the INS point. LiDAR lies at -lever.
    h.longitudinalSpeed=lateral.longitudinalSpeed;
    vy=lateral.lateralVelocity;
    displacement=[-cfg.outputPoint.forwardOffsetM-lever(1);-lever(2)];
    h.longitudinalAcceleration=h.longitudinalAcceleration-rdot*displacement(2)-r.^2*displacement(1);
    h.lateralAcceleration=h.lateralAcceleration+rdot*displacement(1)-r.^2*displacement(2);
    [motion,motionSegment]=integrateRecordedPlanarMotion(t,[h.longitudinalSpeed,vy,r],t,lateral.velocityValid);
    inputs=struct('highRate',h,'lateral',lateral,'nativeOriginSeconds',begin,'motion',motion,'motionSegment',motionSegment, ...
        'wheel',wheel,'parameters',parameters,'pointAlignment',alignment,'lateralOriginal',lateral, ...
        'referenceUsed',false,'gnssUsed',false,'tiltMode',"disabled-moving-start-no-stationary-alignment", ...
        'reconstruction',"Offline linear DBW interpolation; LPV output-point transport precedes the existing angle/rate filter; not a real-time latency claim");
end
