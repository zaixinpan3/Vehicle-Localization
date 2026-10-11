function cfg=mncavFullObserverConfig(options)
% mncavFullObserverConfig MnCAV full-observer configuration with LMI-solved gains.
% The observer gains [kp,kv,ka,kpsi] and the GNSS position gain are not set
% by hand. They are solved from the continuous seven-state ISS LMI
% (fullObserverSupport.synthesizeFullObserverGains) for the declared ISS
% domain, noise scales, error scales and decay rate below by
% scripts/synthesizeMncavObserverGains.m, which writes
% config/mncavSynthesizedGains.json. This function loads that file and
% refuses it when any declared input has changed (user decision of
% 2026-10-11: no a-priori gains). The LiDAR and GNSS information scales are
% declared measurement-weighting parameters, the damping of the Gauss-Newton
% step, not observer gains. The decay rate is the single design parameter;
% it was selected on the tuning segment of the 2024-06-07 recording
% (research/gain_synthesis_20261011), as theta is selected in the source
% paper's design.
    arguments
        options.LoadSynthesizedGains (1,1) logical=true
        options.SynthesisOverride (1,1) struct=struct()
    end
    here=fileparts(mfilename('fullpath'));
    cfg=fullObserverConfig();cfg.kind="mncav-continuous-iss-v1";
    cfg.gnss.outputPoint=jsondecode(fileread(fullfile(here,'mncavBestposOutputPoint.json')));
    cfg.lidar.gainInformationScale=64;   % 1/m^2: prior pose uncertainty 0.125 m in the damped Gauss-Newton step
    cfg.gnss.gainInformationScale=16;    % 1/m^2: prior 0.25 m
    cfg.synthesis=struct('decayRate',3,'designMargin',.01, ...
        'noise',struct('lidarPositionM',.0279,'lidarHeadingRad',.00211,'gnssPositionM',.23, ...
            'velocityMps',.256,'accelerationMps2',.233,'gyroRadps',.0018,'jerkMps3',.05), ...
        'errorScales',struct('positionM',1,'headingLengthM',10,'velocitySeconds',1,'accelerationSeconds2',1), ...
        'weakPositionStrength',.05,'minimumGnssWeight',.5,'solver',"sedumi");
    cfg.synthesis.provenance=struct( ...
        'noise',"lidar: conditional registration error second moments of the first 60 s (2026-10-02 calibration study, reference variance 7.81e-4 m^2 and 4.47e-6 rad^2); gnss: BESTPOS reported sigma median on the 12-09-31 drive; velocity: lateral-velocity RMSE of the adopted MnCAV identification on the evaluation drive (mncavVehicleParameters.json); acceleration and gyro: validation RMSE of the fixed IMU corrections on the 12-11-24 drive (mncavInputCorrections.json); jerk: declared bound above the 0.014 m/s^3 residual of the accelerating-turn maneuver", ...
        'errorScales',"position in metres; heading weighted by a 10 m landmark range; velocity and acceleration by one second of integration", ...
        'scenarios',"LiDAR sufficient without GNSS, and LiDAR translational strength 0.05 with GNSS weight at least 0.5 (fixes the GNSS gain)", ...
        'decayRate',"single design parameter, selected on frames 1-700 of the 12-09-31 drive and checked on frames 701-1169 (research/gain_synthesis_20261011)");
    for name=string(fieldnames(options.SynthesisOverride)).'
        cfg.synthesis.(name)=options.SynthesisOverride.(name);
    end
    cfg.iss.decayRate=cfg.synthesis.decayRate;
    cfg.gains=[];cfg.gnss.positionGain=[];cfg.gainSource="unsolved";
    if options.LoadSynthesizedGains
        path=fullfile(here,'mncavSynthesizedGains.json');
        assert(isfile(path),'VehicleLocalization:MissingSynthesizedGains', ...
            'config/mncavSynthesizedGains.json is missing; run synthesizeMncavObserverGains.');
        stored=jsondecode(fileread(path));
        assert(isequaln(jsondecode(jsonencode(cfg.synthesis)),stored.inputs),'VehicleLocalization:StaleSynthesizedGains', ...
            'config/mncavSynthesizedGains.json was solved for different synthesis inputs; rerun synthesizeMncavObserverGains.');
        cfg.gains=stored.gains(:).';cfg.gnss.positionGain=stored.gnssPositionGain;
        cfg.gainSource="config/mncavSynthesizedGains.json";
        cfg.gnss.positionGainDesign="Solved with the LiDAR gains from the ISS LMI (weak-translation scenario with GNSS)";
    end
end
