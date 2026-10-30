function cfg=mncavSmallOffsetObserverConfig(options)
% Temporary [1,30] m/s global design with an explicitly assumed GNSS offset.
% This is synthetic geometry, not a replacement for the measured BESTPOS
% calibration. GNSS noise and alignment uncertainty retain the base priors.
% The 1 m heading length also defines this scenario's LiDAR information
% normalization. The L-position feedback cap is 16; other caps are retained.
% Engineering objective selection leaves the original certificate feasible set unchanged.
% Solve gains after configuration with synthesizeMncavInvariantDesign(cfg).
    arguments
        options.GnssOffsetNormM (1,1) double {mustBeReal,mustBeFinite,mustBeNonnegative}=.01
    end
    cfg=mncavFullObserverConfig();
    measured=cfg.gnss.outputPoint;
    direction=measured.bodyOffset(:)/norm(measured.bodyOffset);
    cfg.iss.maximumSpeed=30;
    cfg.invariantObserver.redundancy.minimumCourseSpeed=1;
    cfg.invariantObserver.headingLengthM=1;
    cfg.lidar.poseScales=[1;1;1];
    cfg.invariantObserver.matchedGainLimits=[16,8,2];
    % Candidate objective weights are solver preferences, outside the theorem
    % inputs. All returned gains must pass the unchanged certificate verifier.
    cfg.gainSelection=struct('enabled',true,'penaltyWeights',[0,.001,.01,.1,1]);
    cfg.gnss.outputPoint=struct( ...
        'identifier',"mncav-hypothetical-small-gnss-offset-v1", ...
        'bodyOffset',options.GnssOffsetNormM*direction, ...
        'bodyCovariance',measured.bodyCovariance, ...
        'headingStdRad',measured.headingStdRad, ...
        'convention',measured.convention, ...
        'interpretation',"Temporary assumed measurement-point geometry; not an empirical calibration", ...
        'assumedGeometry',true, ...
        'sourceCalibrationIdentifier',measured.identifier);
    cfg.invariantObserver.provenance.gnssGeometry= ...
        "Explicit small-offset design scenario; measured offset direction retained, magnitude overridden; noise priors unchanged";
    cfg.invariantObserver.provenance.headingLengthM= ...
        "Temporary 1 m heading length; information floors and coupling bounds are interpreted in this pose normalization";
    cfg.invariantObserver.provenance.feedback= ...
        "Small-offset design: L-position cap 16, G-position cap 8, velocity-direction cap 2; heading L/direction caps remain 8; no runtime gain clipping";
    cfg.invariantObserver.provenance.gainSelection= ...
        "Engineering multi-objective candidate search over the original LMI feasible set; final primitive-noise proxy ranks independently verified candidates. No hard position-coupling bound, reference trajectory or persisted gains";
end
