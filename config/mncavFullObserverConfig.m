function cfg=mncavFullObserverConfig(options)
% mncavFullObserverConfig Apply independent-drive receiver point calibration.
% Keep nominal correction bandwidth, with soft information-dependent gains.
% Geometric LiDAR information is not a calibrated inverse pose-error covariance.
% GainProfile="mississippi-20240607-fixed-scale" explicitly selects the best
% tested gains on that recording at information scale 16. This is a dataset
% tuning result, not a universal optimum. The default remains nominal.
% All profiles use the same continuous ISS-LMI design and numerical runtime.
% GainProfile="mississippi-20240607-calibrated" selects the experimental
% conditional error calibration, without a LiDAR information-scale parameter.
% Both dataset-tuned profiles were developed using historical reference tilt.
% They remain explicit transfer-comparison profiles after sensor-only tilt was
% introduced; their old accuracy/optimality claims do not describe the new inputs.
    arguments
        options.GainProfile (1,1) string {mustBeMember(options.GainProfile, ...
            ["nominal","mississippi-20240607-fixed-scale","mississippi-20240607-calibrated"])}="nominal"
    end
    cfg=fullObserverConfig();
    cfg.kind="mncav-continuous-iss-v1";
    cfg.gnss.outputPoint=jsondecode(fileread(fullfile(fileparts(mfilename('fullpath')), ...
        'mncavBestposOutputPoint.json')));
    cfg.gnss.positionGain=cfg.gains(1);
    cfg.lidar.gainInformationScale=cfg.gnss.gainInformationScale;
    cfg.gnss.positionGainDesign="Equal nominal position gains and information saturation scales; geometric LiDAR information remains uncalibrated";
    if options.GainProfile=="mississippi-20240607-fixed-scale"
        tuned=jsondecode(fileread(fullfile(fileparts(mfilename('fullpath')), ...
            'mncavMississippiTunedGains.json')));
        cfg.gains([1,4])=[tuned.positionGain,tuned.headingGain];
        cfg.lidar.gainInformationScale=tuned.lidarInformationScale;
        cfg.gnss.positionGainDesign="Nominal GNSS bandwidth retained; LiDAR position/heading gains tuned on the declared dataset";
        cfg.tuning=tuned;
    elseif options.GainProfile=="mississippi-20240607-calibrated"
        tuned=jsondecode(fileread(fullfile(fileparts(mfilename('fullpath')), ...
            'mncavMississippiCalibratedGains.json')));
        cfg.gains([1,4])=[tuned.positionGain,tuned.headingGain];
        cfg.lidar=rmfield(cfg.lidar,'gainInformationScale');
        cfg.lidar.errorCalibration=tuned.calibration;
        cfg.iss.minimumPositionStrength=tuned.minimumPositionStrength;
        cfg.iss.minimumHeadingStrength=tuned.minimumHeadingStrength;
        cfg.iss.maximumPositionHeadingCoupling=tuned.maximumPositionHeadingCoupling;
        cfg.gnss.positionGainDesign="Nominal GNSS bandwidth retained; LiDAR confidence uses measured conditional pose error";
        cfg.tuning=rmfield(tuned,'calibration');
    end
end
