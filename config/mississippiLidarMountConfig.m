function cfg=mississippiLidarMountConfig()
% mississippiLidarMountConfig Fixed installation transform shared with IMU export.
    cfg=jsondecode(fileread(fullfile(fileparts(mfilename('fullpath')), ...
        'mississippiLidarMount.json')));
    cfg.rotation=cfg.additionalRotation*cfg.baseRotation;
end
