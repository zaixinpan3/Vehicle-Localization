function [measurement, result, history] = localizeLidarFrame(frame, localMapCloud, initialPose, timestamp, cfg, history, motionPose, positionAid)
% localizeLidarFrame: Online pillar perception -> local D2D -> observer event.
% localMapCloud is one selected/cached window from temporalMapToProbabilityCloud.
% initialPose is [mapX mapY yaw] in meters/radians, timestamp is acquisition
% time in seconds. The caller supplies a vehicle-aligned frame and may set
% causal sensor-derived tilt in cfg.perception.coarseProbabilityCloud.projectionRotation.
% Reference-pose roll/pitch must not be used here. lidarCalibrationSupport.updateLidarImuTilt provides
% sensor-only compensation; this routine does not estimate extrinsics/tilt.
% The temporal source uses XY because its motion input is planar. The state
% and event pose are [X Y psi]; Z/roll/pitch are not optimized.
% Validated rank-deficient geometry
% emits a directionalPose event; result.accepted still denotes full pose only.
% Set cfg.perception.frameCalibration consistently with the
% offline map; the dataset profile supplies it without changing point selection.
% The returned timestamped record is a registration product, not the input
% contract of the continuous observer. An offline reconstruction must explicitly
% provide continuous, uniformly informative pose output before using it there.
% History and cumulative wheel/gyro motionPose maintain a causal stable-source
% horizon. Pass returned history into the next call; do not feed registration
% corrections into this odometry. A fresh call without motion can collect its
% first scan, but emits no measurement until a later scan confirms support.
% Subsequent calls require motionPose. There is no look-ahead delay.
% Empty measurement means rejection. Exported robust Gaussian information
% uses physical map-frame [X,Y,psi] coordinates and is not empirically calibrated.
% Accepted events also carry a serializable lidarResidualModel. The current
% synchronous observer reevaluates it at its prediction for the Route A gain;
% the frozen model retains associations, robust influence and degeneracy gates.
% Optional positionAid provides current map-frame position and covariance at
% the observer point for hypothesis selection only; GNSS is not added to H.
    if nargin < 5 || isempty(cfg)
        cfg = struct('perception',perceptionConfig(),'registration',distributionRegistrationConfig());
    end
    assert(isscalar(timestamp) && isfinite(timestamp), 'Expected finite acquisition time.');
    if nargin<6,history=[];end
    if nargin<7,motionPose=[];end
    if nargin<8,positionAid=[];end
    if ~isempty(positionAid) && isfield(positionAid,'timestamp')
        assert(abs(positionAid.timestamp-timestamp)<1e-6, ...
            'VehicleLocalization:PositionAidTimeMismatch','Align position aid to acquisition time.');
    end
    assert(isfield(cfg.registration,'method') && any(string(cfg.registration.method)==["geometricD2D","anisotropicD2D","supportD2D"]), ...
        'VehicleLocalization:RegistrationInformationUnavailable', ...
        'Online pose events require a registration method with explicit pose information.');
    startTime = tic;
    cloud = perceiveCoarseProbabilityCloud(frame,cfg.perception);
    perceptionSeconds = toc(startTime);
    registrationStart = tic;
    if isempty(motionPose)
        assert(isempty(history),'VehicleLocalization:WindowMotionRequired', ...
            'Supply cumulative odometry when continuing a perception horizon.');
        motionPose=[0 0 0];
    end
    assert(string(cfg.registration.heightMode)=="xy", ...
        'VehicleLocalization:WindowRequiresXY','The source window requires XY registration.');
    windowCfg=localizationSourceWindowConfig();
    if isfield(cfg,'sourceWindow'),windowCfg=cfg.sourceWindow;end
    [matchingCloud,history,window]=updateLocalizationSourceWindow(cloud,timestamp,motionPose,history,windowCfg);
    windowSeconds=toc(registrationStart);
    result = registerSemanticProbabilityCloud(localMapCloud,matchingCloud,initialPose,cfg.registration,positionAid);
    result.perceptionSeconds = perceptionSeconds;
    result.registrationSeconds = toc(registrationStart);
    result.probabilityCloud = matchingCloud;
    result.currentProbabilityCloud = cloud;
    result.sourceWindowSeconds = windowSeconds;
    result.sourceWindow=window;
    measurement = registrationSupport.registrationPoseMeasurement(result,timestamp);
end
