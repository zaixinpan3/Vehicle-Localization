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
    wheel=localizationSupport.estimateWheelLongitudinalSpeed(input,wheelSpeedObserverConfig());
    assert(all(wheel.valid) && all(isfinite(wheel.longitudinalSpeed)),'VehicleLocalization:WheelSpeedCoverage','No speed fallback is allowed.');
    h.longitudinalSpeed=max(0,wheel.longitudinalSpeed);
    cfg=lateralObserverConfig();lateralObserverSupport.assertLateralVehicleMatches(lateralDesign,cfg);
    root=fileparts(fileparts(mfilename('fullpath')));alignment=jsondecode(fileread(fullfile(root,'config','offlineReferenceFrontLidarMotion.json')));
    lever=alignment.leverLidarToInsMeters(:);
    % Rigid point transport uses the observer state and its ODE derivative.
    map=@(state,rate) receiverVelocityMap(state,rate,lever);
    lateral=runLateralVelocityObserver(h,lateralDesign,cfg,map);
    h.longitudinalSpeed=lateral.longitudinalSpeed;
    vy=lateral.lateralVelocity;
    displacement=[-cfg.outputPoint.forwardOffsetM-lever(1);-lever(2)];
    acceleration=localizationSupport.transportPlanarAcceleration( ...
        [h.longitudinalAcceleration,h.lateralAcceleration], ...
        lateral.yawRate,lateral.yawRateRate,displacement);
    h.longitudinalAcceleration=acceleration(:,1);
    h.lateralAcceleration=acceleration(:,2);
    r=h.yawRate;
    [motion,motionSegment]=integrateRecordedPlanarMotion(t,[h.longitudinalSpeed,vy,r],t,lateral.velocityValid);
    inputs=struct('highRate',h,'lateral',lateral,'nativeOriginSeconds',begin,'motion',motion,'motionSegment',motionSegment, ...
        'wheel',wheel,'parameters',parameters,'pointAlignment',alignment,'lateralOriginal',lateral, ...
        'referenceUsed',false,'gnssUsed',false,'tiltMode',"disabled-moving-start-no-stationary-alignment", ...
        'reconstruction',"Offline linear DBW interpolation; LPV output-point transport precedes the algebraic angle/rate interface; not a real-time latency claim");
end

function map=receiverVelocityMap(state,rate,lever)
    n=size(state,1);value=repmat([1,0,0,1,0,0],n,1);derivative=zeros(n,6);
    value(:,5:6)=[lever(2)*state(:,2),-lever(1)*state(:,2)];
    derivative(:,5:6)=[lever(2)*rate(:,2),-lever(1)*rate(:,2)];
    map=struct('value',value,'rate',derivative);
end
