function [measurement, result, history] = localizeLidarFrame(frame, localMapCloud, initialPose, timestamp, cfg, history, motionPose)
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
% offline map; the dataset profile supplies it without changing point selection.
% The returned timestamped record is a registration product, not the input
% contract of the continuous observer. An offline reconstruction must explicitly
% provide continuous, uniformly informative pose output before using it there.
% Optional history and cumulative wheel/gyro motionPose enable a causal
% three-scan XY source window. Pass returned history into the next call. Never
% use previous registration poses as this odometry input. Without motionPose
% the call uses the current scan. The window introduces no look-ahead delay.
% Empty measurement means rejection. Exported robust Gaussian information
% uses physical map-frame [X,Y,psi] coordinates and is not empirically calibrated.
    if nargin < 5 || isempty(cfg)
        cfg = struct('perception',perceptionConfig(),'registration',distributionRegistrationConfig());
    end
    assert(isscalar(timestamp) && isfinite(timestamp), 'Expected finite acquisition time.');
    if nargin<6,history=[];end
    if nargin<7,motionPose=[];end
    assert(isfield(cfg.registration,'method') && string(cfg.registration.method)=="geometricD2D", ...
        'VehicleLocalization:RegistrationInformationUnavailable', ...
        'Online pose events require a registration method with explicit pose information.');
    startTime = tic;
    cloud = perceiveCoarseProbabilityCloud(frame,cfg.perception);
    perceptionSeconds = toc(startTime);
    registrationStart = tic;
    matchingCloud=cloud;window=struct('frameCount',1,'spanSeconds',0,'dimension',2);
    if ~isempty(motionPose)
        assert(string(cfg.registration.heightMode)=="xy", ...
            'VehicleLocalization:WindowRequiresXY','The source window requires XY registration.');
        windowCfg=localizationSourceWindowConfig();
        if isfield(cfg,'sourceWindow'),windowCfg=cfg.sourceWindow;end
        [matchingCloud,history,window]=updateLocalizationSourceWindow(cloud,timestamp,motionPose,history,windowCfg);
    else
        history=[];
    end
    result = registerSemanticProbabilityCloud(localMapCloud,matchingCloud,initialPose,cfg.registration);
    result.perceptionSeconds = perceptionSeconds;
    result.registrationSeconds = toc(registrationStart);
    result.probabilityCloud = cloud;
    result.sourceWindow=window;
    measurement = registrationSupport.registrationPoseMeasurement(result,timestamp);
end
