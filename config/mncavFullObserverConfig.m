function cfg=mncavFullObserverConfig(options)
% Current MnCAV intrinsic predictor; IMU is confined to invariant innovations.
% The eighth, internal yaw-rate state removes speed-denominator singularities.
% Gains are solved from the new drift LMI and the invariant-noise objective.
    arguments
        options.LoadDesign (1,1) logical=true
        options.DesignOverride (1,1) struct=struct()
        options.Theta (1,1) double {mustBeReal,mustBeFinite}=4
    end
    cfg=measuredCourseObserverConfig(LoadDesign=false);
    cfg.kind="mncav-intrinsic-invariant-v1";
    z=cfg.thetaFramework;cfg=rmfield(cfg,'thetaFramework');
    z.sigma=4;z.theta=options.Theta;z.yawAccelerationRadps2=cfg.maximumTrackAngleRate/2;z.sideslipRateNoiseRadps=.01;
    z.balancedPoseMetric=true;
    z.angularMetricDiagonal=true;
    z.noiseObjective="normalizedFullState";
    z.velocityAvailable=true;z.accelerationAvailable=true;
    z.operatingPoints=["lidar","lidarGnss","weakLidarGnss","gnssOnly"];
    z=rmfield(z,'deadReckoningHorizonS');
    z.provenance.architecture="State [X,VX,AX,Y,VY,AY,psi,r]; predict uses internal r and estimated lateral sideslip rate; IMU only in invariant innovations";
    z.provenance.angularModel="Yaw-rate process scale maximumTrackAngleRate / 2 s and lateral sideslip-rate error 0.01 rad/s; revised after the slow-rate prototype, with no navigation input to runtime";
    z.provenance.operatingPoints="Noise objective at informative LiDAR, LiDAR plus GNSS, weak LiDAR plus GNSS and moving GNSS only; no absolute-pose steady-state objective without a pose anchor";
    z.provenance.metric="Angular two-state Lyapunov block bounded by translational position/velocity principal block in normalized units; introduced after diagnosing weak heading authority of the initial prototype";
    z.provenance.theta="theta = sigma = 4 fixed for the intrinsic design; runtime scale inherited from the preceding route, no new theta sweep";
    z.provenance.noiseObjective="All eight normalized errors at pose-anchored nominal points; diagonal noise proxy is a synthesis objective, not an empirical covariance guarantee";
    for name=string(fieldnames(options.DesignOverride)).',z.(name)=options.DesignOverride.(name);end
    cfg.lidar.poseScales=[1;1;1/z.headingLengthM];
    cfg.invariantObserver=z;cfg.gainSource="unsolved-invariant";
    if options.LoadDesign
        path=fullfile(fileparts(mfilename('fullpath')),'mncavInvariantDesign.json');
        assert(isfile(path),'VehicleLocalization:MissingInvariantDesign','Run synthesizeMncavInvariantDesign first.');
        stored=jsondecode(fileread(path));
        assert(isequaln(invariantObserverSupport.inputs(cfg),stored.inputs), ...
            'VehicleLocalization:StaleInvariantDesign','Stored invariant gains use different declared inputs; re-synthesize.');
        cfg.invariantDesign=stored.design;cfg.gainSource="config/mncavInvariantDesign.json";
    end
end
