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
% An accepted event has timestamp, arrivalTime, pose, information fields consumed by
% runImprovedVehicleObserver. The caller sets arrivalTime when it is delivered;
% the acquisition-time default is explicitly marked arrivalTimeIsPlaceholder.
% Empty measurement means rejection. Information is the final robust Gaussian
% model information in physical map-frame [X,Y,psi] coordinates, not an
% inverse empirically calibrated pose covariance or a density-score Hessian.
    if nargin < 5 || isempty(cfg)
        cfg = struct('perception',perceptionConfig(),'registration',distributionRegistrationConfig());
    end
    assert(isscalar(timestamp) && isfinite(timestamp), 'Expected finite acquisition time.');
    assert(isfield(cfg.registration,'method') && string(cfg.registration.method)=="geometricD2D", ...
        'VehicleLocalization:RegistrationInformationUnavailable', ...
        'Online pose events require geometricD2D with Gaussian pose information.');
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
