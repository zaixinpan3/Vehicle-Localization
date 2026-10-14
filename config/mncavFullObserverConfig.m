function cfg=mncavFullObserverConfig(options)
% Current MnCAV intrinsic predictor; IMU is confined to invariant innovations.
% The eighth, internal yaw-rate state removes speed-denominator singularities.
% Gains satisfy the common sensor LMIs and the invariant-noise objective.
    arguments
        options.LoadDesign (1,1) logical=true
        options.DesignOverride (1,1) struct=struct()
        options.Theta (1,1) double {mustBeReal,mustBeFinite}=4
    end
    here=fileparts(mfilename('fullpath'));
    cfg=struct('kind',"mncav-intrinsic-redundancy-v2",'maximumTrackAngleRate',.4,'initialState',[]);
    cfg.timing="synchronous";
    cfg.synchronization=struct('gnssMaximumBracket',.15,'motionMaximumBracket',.025);
    cfg.gnss=struct('gainInformationScale',16,'outputPoint', ...
        jsondecode(fileread(fullfile(here,'mncavBestposOutputPoint.json'))));
    cfg.lidar=struct('poseScales',[1;1;.1],'gainInformationScale',64);
    cfg.iss=struct('maximumSpeed',15,'maximumAcceleration',2, ...
        'minimumPositionStrength',.25,'minimumHeadingStrength',.97, ...
        'maximumPositionHeadingCoupling',.06);
    z=struct('sigma',4,'headingLengthM',10, ...
        'noise',struct('lidarPositionM',.0279,'lidarHeadingRad',.00211,'gnssPositionM',.23, ...
            'velocityMps',.256,'accelerationMps2',.233,'gyroRadps',.0018,'jerkMps3',.05), ...
        'weakPositionStrength',.05,'minimumGnssWeight',.5,'nominalGnssWeight',.5, ...
        'gnssCouplingSplit',.5,'operatingPoints',["lidar","lidarGnss","weakLidarGnss","gnssOnly"], ...
        'lambda',1,'margin',1e-7,'solver',"sedumi");
    z.provenance=struct( ...
        'noise',"Inherited declared sensor priors: conditional registration moments on the first 60 s of the 2026-10-02 study; BESTPOS reported sigma on 12-09-31; adopted lateral identification and independently corrected IMU on 12-11-24; these are design priors, not an overlap covariance certificate", ...
        'headingLengthM',"Heading error weighted by a 10 m landmark range", ...
        'normalization',"lambda = 1 and P >= I");
    z.theta=options.Theta;z.yawAccelerationRadps2=cfg.maximumTrackAngleRate/2;z.sideslipRateNoiseRadps=.01;
    z.balancedPoseMetric=false;
    z.angularMetricDiagonal=true;
    z.noiseObjective="normalizedFullState";
    z.velocityAvailable=true;z.accelerationAvailable=true;
    z.redundancy=struct('minimumGnssSpeed',10,'informativeLidarMismatch',.001, ...
        'degradedLidarMismatch',.001,'speedPartitions',10, ...
        'accelerationSplit',.5,'gnssSplit',.5,'lambda',1,'kinematicGnssEnclosure',true, ...
        'exactGnssLeverEnclosure',true);
    z.headingGainLimits=[8,8]; % kappaL/P77 and directionGain/P77.
    z.matchedGainLimits=[8,8,2]; % Position L/G and velocity-direction self-feedback.
    z.provenance.architecture="State [X,VX,AX,Y,VY,AY,psi,r]; predict uses internal r and estimated lateral sideslip rate; IMU only in invariant innovations";
    z.provenance.angularModel="Yaw-rate process scale maximumTrackAngleRate / 2 s and lateral sideslip-rate error 0.01 rad/s; revised after the slow-rate prototype, with no navigation input to runtime";
    z.provenance.operatingPoints="Noise objective at informative LiDAR, LiDAR plus GNSS, weak LiDAR plus GNSS and moving GNSS only; no absolute-pose steady-state objective without a pose anchor";
    z.provenance.metric="Rotation-invariant translational blocks and a diagonal angular block; angular/translation balance is not a theorem condition and is withdrawn to permit fixed feedback-authority limits";
    z.provenance.theta="theta = sigma = 4 fixed for the intrinsic design; runtime scale inherited from the preceding route, no new theta sweep";
    z.provenance.noiseObjective="All eight normalized errors at pose-anchored nominal points; diagonal noise proxy is a synthesis objective, not an empirical covariance guarantee";
    z.provenance.redundancy="Common fixed metric/gains for LiDAR-sufficient/GNSS-absent and moving-GNSS-sufficient/rank-zero-LiDAR sets; actual overlap mismatch <=0.001 is a geometric contract, not a measured map certificate; G is withdrawn when velocity/acceleration channels are unavailable";
    z.provenance.feedback="Normalized matched self-feedback limits L-position 8, G-position 8, velocity-direction 2 and heading L/direction 8; constrained inside the solve, without runtime clipping; exact G kinematics and lever coupling reduce sufficient-condition conservatism";
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
