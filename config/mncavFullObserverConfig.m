function cfg=mncavFullObserverConfig()
% mncavFullObserverConfig Apply independent-drive receiver point calibration.
% Give either position channel the same nominal 0.25 s correction time scale.
% The choice is structural, not a sweep against evaluation-reference error.
    cfg=fullObserverConfig();
    cfg.kind="mncav-synchronous-aligned-observer-v3";
    cfg.gnss.outputPoint=jsondecode(fileread(fullfile(fileparts(mfilename('fullpath')), ...
        'mncavBestposOutputPoint.json')));
    cfg.gnss.positionGain=cfg.gains(1);
    cfg.gnss.positionGainDesign="Match the LiDAR position gain; preserve correction bandwidth when either source is missing";
end
