function cfg=mncavFullObserverConfig(options)
% mncavFullObserverConfig Apply independent-drive receiver point calibration.
% The default gains are the single tuned MnCAV set of
% config/mncavTunedGains.json: LiDAR position gain 80/s, heading gain 1/s and
% information scale 64, with the nominal GNSS position gain 4/s and GNSS
% information scale 16. They were selected on 2026-10-10 on the
% 2024-06-07 recording for both LiDAR channels, on a tuning segment and a
% held-out segment of the same drive, from candidates that pass the continuous
% seven-state ISS LMIs. This is a dataset tuning result, not a universal
% optimum; the runtime re-verifies the LMIs at every design.
% Geometric LiDAR information is not a calibrated inverse pose-error covariance.
% GainProfile="mississippi-20240607-calibrated" selects the experimental
% conditional error calibration, without a LiDAR information-scale parameter.
% It was developed using historical reference tilt and remains an explicit
% transfer-comparison profile after sensor-only tilt was introduced.
    arguments
        options.GainProfile (1,1) string {mustBeMember(options.GainProfile, ...
            ["mncav-20261010-tuned","mississippi-20240607-calibrated"])}="mncav-20261010-tuned"
    end
    cfg=fullObserverConfig();
    cfg.kind="mncav-continuous-iss-v1";
    cfg.gnss.outputPoint=jsondecode(fileread(fullfile(fileparts(mfilename('fullpath')), ...
        'mncavBestposOutputPoint.json')));
    cfg.gnss.positionGain=cfg.gains(1);
    if options.GainProfile=="mncav-20261010-tuned"
        tuned=jsondecode(fileread(fullfile(fileparts(mfilename('fullpath')),'mncavTunedGains.json')));
        cfg.gains([1,4])=[tuned.positionGain,tuned.headingGain];
        cfg.gnss.positionGain=tuned.gnssPositionGain;cfg.gnss.gainInformationScale=tuned.gnssInformationScale;
        cfg.lidar.gainInformationScale=tuned.lidarInformationScale;
        cfg.gnss.positionGainDesign="Nominal GNSS bandwidth retained; LiDAR position/heading gains and information scale tuned on the declared dataset";
        cfg.tuning=tuned;
    else
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
