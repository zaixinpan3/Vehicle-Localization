function cfg=mncavFullObserverConfig(options)
% mncavFullObserverConfig MnCAV full observer in the theta framework of the source paper.
% The observer is the multi-output high-gain observer of Bessafa et al.
% (Automatica 188, 2026) with this project's model and outputs: the
% main-output gain is L = T(theta) K, the extra-output gain is
% M = T(theta) N / theta^3, and K and N are solved from the paper's LMI (55)
% (fullObserverSupport.synthesizeThetaFrameworkDesign) for the declared inputs
% below by scripts/synthesizeMncavThetaDesign.m, which writes
% config/mncavThetaDesign.json. This function loads that file and refuses it
% when any declared input has changed (user decisions of 2026-10-11: gains
% are solved, not set a priori, inside the paper's framework). theta is the
% single tuning parameter. It was selected on the tuning segment of the
% 2024-06-07 recording with the position and heading objective of 2026-10-10
% extended to every production scenario, GNSS only and the sensor outages
% included (user decision of 2026-10-11; research/theta_framework_20261011).
% The LiDAR and GNSS information scales
% are measurement-weighting parameters (damping of the Gauss-Newton step),
% not observer gains; LiDAR information is filtered in the normalized pose
% units [m m ell*rad] of the design.
    arguments
        options.LoadDesign (1,1) logical=true
        options.DesignOverride (1,1) struct=struct()
        options.Theta (1,1) double {mustBeReal,mustBeFinite}=4
    end
    here=fileparts(mfilename('fullpath'));
    cfg=fullObserverConfig();cfg.kind="mncav-theta-framework-v1";
    cfg.gnss.outputPoint=jsondecode(fileread(fullfile(here,'mncavBestposOutputPoint.json')));
    cfg.lidar.gainInformationScale=64;   % 1/m^2 in normalized pose units: prior 0.125 m (0.0125 rad)
    cfg.gnss.gainInformationScale=16;    % 1/m^2: prior 0.25 m
    cfg.thetaFramework=struct('sigma',1,'headingLengthM',10, ...
        'noise',struct('lidarPositionM',.0279,'lidarHeadingRad',.00211,'gnssPositionM',.23, ...
            'velocityMps',.256,'accelerationMps2',.233,'gyroRadps',.0018,'jerkMps3',.05), ...
        'weakPositionStrength',.05,'minimumGnssWeight',.5,'nominalGnssWeight',.5,'gnssCouplingSplit',.5, ...
        'operatingPoints',["lidar","lidarGnss","weakLidarGnss","deadReckoning"],'deadReckoningHorizonS',10, ...
        'lambda',1,'margin',1e-7,'solver',"sedumi");
    cfg.thetaFramework.provenance=struct( ...
        'noise',"lidar: conditional registration error second moments of the first 60 s (2026-10-02 calibration study); gnss: BESTPOS reported sigma median on the 12-09-31 drive; velocity: lateral-velocity RMSE of the adopted MnCAV identification (mncavVehicleParameters.json); acceleration and gyro: validation RMSE of the fixed IMU corrections on the 12-11-24 drive (mncavInputCorrections.json); jerk: declared bound above the 0.014 m/s^3 residual of the accelerating-turn maneuver", ...
        'headingLengthM',"heading error weighted by a 10 m landmark range (error scale of the 2026-10-11 synthesis)", ...
        'operatingPoints',"mean-square objective at LiDAR without GNSS, LiDAR with GNSS (weight 0.5), LiDAR translation strength 0.05 with GNSS, and dead reckoning without LiDAR and GNSS over a 10 s horizon (the outage length of the synthetic cascade validation)", ...
        'normalization',"lambda = 1 and P >= I, as in the 2026-10-03 reproduction of the paper's design", ...
        'theta',"runtime theta 4 selected on frames 1-700 of the 12-09-31 drive from {1,1.5,2,3,4,5,6} with the position and heading objective of 2026-10-10 over every production scenario (GNSS + LiDAR, LiDAR only, GNSS only, GNSS, LiDAR and both outages, alternating; user decision 2026-10-11), checked on frames 701-1169 (research/theta_framework_20261011)");
    for name=string(fieldnames(options.DesignOverride)).'
        cfg.thetaFramework.(name)=options.DesignOverride.(name);
    end
    cfg.lidar.poseScales=[1;1;1/cfg.thetaFramework.headingLengthM];
    cfg.gains=[];cfg.gnss.positionGain=[];cfg.gainSource="unsolved";
    if options.LoadDesign
        path=fullfile(here,'mncavThetaDesign.json');
        assert(isfile(path),'VehicleLocalization:MissingThetaDesign', ...
            'config/mncavThetaDesign.json is missing; run synthesizeMncavThetaDesign.');
        stored=jsondecode(fileread(path));
        assert(isequaln(fullObserverSupport.thetaFrameworkInputs(cfg),stored.inputs),'VehicleLocalization:StaleThetaDesign', ...
            'config/mncavThetaDesign.json was solved for different declared inputs; rerun synthesizeMncavThetaDesign.');
        cfg.hgo=struct('theta',options.Theta,'sigma',stored.sigma,'ell',cfg.thetaFramework.headingLengthM, ...
            'P',stored.P,'kappaL',stored.kappaL,'kappaG',stored.kappaG,'Z',stored.Z,'lambda',stored.lambda);
        cfg.gainSource="config/mncavThetaDesign.json";
    end
end
