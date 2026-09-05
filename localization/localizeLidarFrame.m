function [measurement, result] = localizeLidarFrame(frame, localMapCloud, initialPose, timestamp, cfg)
% localizeLidarFrame: Online pillar perception -> local D2D -> observer event.
% localMapCloud is one selected/cached window from temporalMapToProbabilityCloud.
% initialPose is [mapX mapY yaw] in meters/radians, timestamp is acquisition
% time in seconds. The caller supplies a vehicle-aligned frame and sets known
% IMU tilt in cfg.perception.coarseProbabilityCloud.projectionRotation when
% required by the map convention. poseRowToPlanarPose supplies this rotation
% for recorded mapping poses. This routine does not estimate extrinsics/tilt.
% Set cfg.registration.heightTranslation to the sensor/vehicle origin in map
% Z (third output of poseRowToPlanarPose for recorded data). Auto mode uses
% XYZ only with a known vertical reference and height in both clouds;
% result.height describes the chosen mode. The event pose remains [x y yaw].
% An accepted event has timestamp, arrivalTime, pose fields consumed by
% runImprovedVehicleObserver. The caller sets arrivalTime when it is delivered.
% Empty measurement means rejection. Curvature is deliberately not exported
% as sensor information: that conversion requires empirical calibration.
    if nargin < 5 || isempty(cfg)
        cfg = struct('perception',perceptionConfig(),'registration',distributionRegistrationConfig());
    end
    assert(isscalar(timestamp) && isfinite(timestamp), 'Expected finite acquisition time.');
    startTime = tic;
    cloud = perceiveCoarseProbabilityCloud(frame,cfg.perception);
    perceptionSeconds = toc(startTime);
    registrationStart = tic;
    result = registerSemanticProbabilityCloud(localMapCloud,cloud,initialPose,cfg.registration);
    result.perceptionSeconds = perceptionSeconds;
    result.registrationSeconds = toc(registrationStart);
    result.probabilityCloud = cloud;
    measurement = [];
    if result.accepted
        measurement = struct('timestamp',double(timestamp),'arrivalTime',double(timestamp), ...
            'pose',result.poseXYTheta);
    end
end
