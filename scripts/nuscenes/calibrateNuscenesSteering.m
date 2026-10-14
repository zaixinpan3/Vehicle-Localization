function calibration=calibrateNuscenesSteering(datasetFolder,calibrationScenes)
% calibrateNuscenesSteering Approximate steering ratio from wheel speed/gyro.
% Uses delta = atan(wheelbase*r/vx) on moving, moderate-turn CAN samples.
% This is a sensor-only kinematic approximation, not identified tire dynamics.
% Call with scenes disjoint from evaluation; pose and pose.vel are never read.
    arguments
        datasetFolder (1,1) string
        calibrationScenes (1,:) string
    end
    ratios=[];counts=zeros(numel(calibrationScenes),1);
    trial=nuscenesTrialConfig(1);
    for j=1:numel(calibrationScenes)
        prefix=fullfile(datasetFolder,'can_bus',calibrationScenes(j));
        imu=jsondecode(fileread(prefix+"_ms_imu.json"));
        wheels=jsondecode(fileread(prefix+"_zoe_veh_info.json"));
        steering=jsondecode(fileread(prefix+"_steeranglefeedback.json"));
        m=prepareNuscenesMotion(imu,wheels,steering,trial);
        valid=m.longitudinalSpeed>2 & abs(m.yawRate)>.025 & abs(m.yawRate)<.35 & abs(m.steeringAngle)>.2;
        value=m.steeringAngle(valid)./atan((trial.vehicle.lf+trial.vehicle.lr)*m.yawRate(valid)./m.longitudinalSpeed(valid));
        value=value(value>0 & isfinite(value));counts(j)=numel(value);ratios=[ratios;value]; %#ok<AGROW>
    end
    assert(numel(ratios)>=100,'VehicleLocalization:InsufficientSteeringCalibration','Need at least 100 informative CAN samples.');
    calibration=struct('steeringRatio',median(ratios),'scenes',calibrationScenes, ...
        'samplesByScene',counts,'samples',numel(ratios),'ratioInterquartileRange',prctile(ratios,[25,75]), ...
        'method',"Median steering feedback / atan(L*gyro/wheelSpeed), no navigation/reference data", ...
        'referenceUsed',false,'identifiedDynamicModel',false);
end
