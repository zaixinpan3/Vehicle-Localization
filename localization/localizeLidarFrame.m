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
% result.height describes the chosen mode. The state and event pose are ALWAYS
% [X Y psi]; Z/roll/pitch are not optimized. Geometric D2D uses height only to
% condition correspondence compatibility. Validated rank-deficient geometry
% emits a directionalPose event; result.accepted still denotes full pose only.
% Set cfg.perception.frameCalibration consistently with the
% offline map; its default is identity and it never changes point selection.
% The returned timestamped record is a registration product, not the input
% contract of the continuous observer. An offline reconstruction must explicitly
% provide continuous, uniformly informative pose output before using it there.
% Empty measurement means rejection. Geometric D2D returns robust Gaussian
% model information; the opt-in weightedNdt candidate returns overlap-objective
% curvature. Both use physical map-frame [X,Y,psi] coordinates, and neither
% is an empirically calibrated inverse pose-error covariance.
    if nargin < 5 || isempty(cfg)
        cfg = struct('perception',perceptionConfig(),'registration',distributionRegistrationConfig());
    end
    assert(isscalar(timestamp) && isfinite(timestamp), 'Expected finite acquisition time.');
    assert(isfield(cfg.registration,'method') && ismember(string(cfg.registration.method),["geometricD2D","weightedNdt"]), ...
        'VehicleLocalization:RegistrationInformationUnavailable', ...
        'Online pose events require a registration method with explicit pose information.');
    startTime = tic;
    cloud = perceiveCoarseProbabilityCloud(frame,cfg.perception);
    perceptionSeconds = toc(startTime);
    registrationStart = tic;
    result = registerSemanticProbabilityCloud(localMapCloud,cloud,initialPose,cfg.registration);
    result.perceptionSeconds = perceptionSeconds;
    result.registrationSeconds = toc(registrationStart);
    result.probabilityCloud = cloud;
    measurement = registrationSupport.registrationPoseMeasurement(result,timestamp);
end
