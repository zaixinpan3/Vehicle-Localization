function estimate=runFullLocalizationObserver(data,cfg,options)
% runFullLocalizationObserver Run the current intrinsic global observer.
% Supply a feature map and scan clouds in data.lidarOverlap for LiDAR feedback.
% Prediction uses an internal rate state; heading innovations also correct that state.
% All calls use the intrinsic eight-state design.
% The common-clock runtime uses the nominal fusion LMI gains.
% Pass this run's solved configuration to share gains within one experiment;
% if absent, the entry solves them before the first observer update.
% Supply aligned lateral estimates through LateralInputs; upstream synthesis is separate.
% Missing/invalid measurements withdraw their own correction for that frame.
% Optional highRate.bodyVelocityValid and bodyAccelerationValid masks also
% withdraw unavailable motion-output feedback. Absent masks mean all valid.
% LateralInputs supplies lateralVelocity on the frame clock. The predictor
% uses only its own state; no sideslip-angle or rate input is required.
% Yaw rate is corrected through the coupled motion/pose gains, not a separate input.
% Optional data.gnssHeading supplies independent heading (rad), time, valid
% and delay=0; it is never inferred from a reference pose. Information metadata
% is not used to weight position or heading corrections.
% Optional LateralInputs.velocityValid withdraws all upstream signals,
% and their velocity-dependent innovations.
% LiDAR remains available without velocity aiding, including at standstill.
% diagnostics.lidarPoseSupport reports local geometry, not alignment correctness.
    arguments
        data (1,1) struct
        cfg (1,1) struct=mncavFullObserverConfig()
        options.LateralInputs (1,1) struct=struct()
    end
    assert(~isfield(data,'lidarMatcher'),'VehicleLocalization:ArchivedRegistration', ...
        'Pose-registration callbacks are removed. Supply overlap-gradient sources.');
    assert(~isfield(cfg.lidar,'errorCalibration'),'VehicleLocalization:ArchivedRegistrationMeasurement', ...
        'Use the geometric information filter; registration-error calibration is removed.');
    localizationSupport.rejectExternalLidar(data);
    valid=false(numel(data.highRate.time),1);
    lidar=struct('valid',valid,'name',"none");
    if isfield(data,'lidarOverlap')
        lidar=overlapSource(data.lidarOverlap,data.highRate.time(:));
        lidar.name="overlapGradient";
        map=lidar.map;sources=lidar.sources;settings=lidar.config;
        lidar.evaluate=@(k,pose)localizationSupport.evaluateOverlapGradient(map,sources{k},pose,settings);
    end
    estimate=observerInternal.integrate(data,cfg,options.LateralInputs,lidar);
end

function o=overlapSource(o,t)
    n=numel(t);
    assert(isstruct(o) && isscalar(o) && all(isfield(o,{'map','sources'})) && iscell(o.sources) && ...
        numel(o.sources)==n && isfield(o.map,'components'),'VehicleLocalization:InvalidOverlapSource', ...
        'lidarOverlap requires a map cloud and one source-cloud cell per frame.');
    o.sources=o.sources(:);
    if ~isfield(o,'valid'),o.valid=~cellfun(@isempty,o.sources);end
    assert(numel(o.valid)==n && all(ismember(o.valid,[0,1])),'VehicleLocalization:InvalidOverlapSource', ...
        'lidarOverlap.valid must be one logical value per frame.');
    o.valid=logical(o.valid(:)) & ~cellfun(@isempty,o.sources);
    if isfield(o,'time')
        assert(isequal(o.time(:),t),'VehicleLocalization:SynchronousInputsRequired', ...
            'Overlap source clocks must equal the localization clock.');
    end
    if isfield(o,'delay')
        assert(isequal(o.delay,0),'VehicleLocalization:FullZeroDelayRequired', ...
            'Overlap sources require zero perception delay.');
    end
    defaults=struct('frameReliability',ones(n,1),'directionReliability',ones(n,3));
    for name=["frameReliability","directionReliability"]
        if ~isfield(o,name),o.(name)=defaults.(name);end
        value=o.(name);
        assert(isequal(size(value),size(defaults.(name))) && isreal(value) && ...
            all(isfinite(value),'all') && all(value>=0 & value<=1,'all'), ...
            'VehicleLocalization:InvalidLidarReliability','Invalid aligned reliability array.');
    end
    if ~isfield(o,'config'),o.config=overlapGradientConfig();end
end
