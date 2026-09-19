function cfg=mncavFullObserverConfig()
% mncavFullObserverConfig Apply independent-drive receiver point calibration.
% Keep nominal correction bandwidth, with soft information-dependent gains.
% Geometric LiDAR information is not a calibrated inverse pose-error covariance.
    cfg=fullObserverConfig();
    cfg.kind="mncav-synchronous-aligned-observer-v4";
    cfg.gnss.outputPoint=jsondecode(fileread(fullfile(fileparts(mfilename('fullpath')), ...
        'mncavBestposOutputPoint.json')));
    cfg.gnss.positionGain=cfg.gains(1);
    cfg.lidar.gainInformationScale=cfg.gnss.gainInformationScale;
    cfg.gnss.positionGainDesign="Equal nominal position gains and information saturation scales; geometric LiDAR information remains uncalibrated";
end
