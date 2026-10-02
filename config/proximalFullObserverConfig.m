function cfg=proximalFullObserverConfig()
% proximalFullObserverConfig MnCAV observer with robust proximal LiDAR updates.
% The established matched observer remains available via mncavFullObserverConfig.
    cfg=mncavFullObserverConfig();
    cfg.kind="mncav-synchronous-proximal-v1";
    cfg.lidar.updateLaw="proximal";
    cfg.lidar.proximal=proximalLidarConfig();
end
