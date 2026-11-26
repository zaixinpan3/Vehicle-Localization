function estimate=runFullLocalizationObserver(data,cfg,options)
% runFullLocalizationObserver Run the current intrinsic global observer.
% Supply a feature map and scan clouds in data.lidarOverlap for LiDAR feedback.
% Heading prediction integrates the internal body yaw-rate state.
% All calls use the intrinsic eight-state design.
% The common-clock runtime uses the nominal fusion LMI gains.
% Pass this run's solved configuration to share gains within one experiment;
% if absent, the entry solves them before the first observer update.
% Supply aligned lateral estimates through LateralInputs; upstream synthesis is separate.
% Missing/invalid measurements withdraw their own correction for that frame.
% highRate.bodyVelocityValid selects signed body velocity (m/s).
% bodyAccelerationValid independently selects body-axis inertial acceleration
% (m/s^2) at the same output point. Absent masks mean all valid.
% LateralInputs.yawRate is the upstream yaw-rate observation in rad/s. Its
% difference from the internal rate enters the first (linear) feedback term.
% A finite value is active when yawRateValid is true (defaults to true).
% During upstream-rate gaps, heading propagates with the internal rate.
% The common angular metric lets other available innovations update that rate.
% gnss.valid enables position feedback. Optional gnss.heading is in radians
% on that packet's clock; NaN means heading is unavailable. A separate
% data.gnssHeading stream can instead supply its own time, valid and delay=0.
% During receiver-heading gaps, signed wheel/lateral motion and yaw rate
% align a resolved antenna displacement between two available position fixes.
% The recovered angle uses the same angular gain and rotates the antenna
% offset. Until such a chord is available, position uses predicted yaw.
% cfg.gnss.positionStdM is the receiver position error scale in meters.
% diagnostics.recoveredHeading is NaN on frames without a derived angle.
% Optional LateralInputs.velocityValid gates the velocity-dependent innovations.
% Low-speed wheel/gyro packets enter through the same motion innovations.
% LiDAR remains available without velocity aiding, including at standstill.
% The LiDAR term is 2*L*S*((G+S)\g), where g is the log-overlap ascent
% gradient and S is the configured pose-strength floor. The raw field Gram
% G is diagnostics.poseGram(:,:,k)/diagnostics.poseGramDenominator(k).
% diagnostics.lidarMatched{k}.feedbackGram is the effective Gram of the step.
% diagnostics.lidarIncrement is the physical state increment attributable to
% LiDAR at the joint solution, with one row per epoch.
    arguments
        data (1,1) struct
        cfg (1,1) struct=mncavFullObserverConfig()
        options.LateralInputs (1,1) struct=struct()
    end
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
    if ~isfield(o,'config'),o.config=overlapGradientConfig();end
end
