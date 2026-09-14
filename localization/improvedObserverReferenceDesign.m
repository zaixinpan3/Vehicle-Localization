function design = improvedObserverReferenceDesign(cfg)
% improvedObserverReferenceDesign Load and recheck one continuous-mode design.
    arguments
        cfg (1,1) struct = improvedObserverConfig()
    end
    if cfg.mode=="gnss" && isfield(cfg.observer,'gnssChainGain')
        % A changed chain needs its own metric, recomputed and verified with
        % the same operating bounds. Never attach the reference P blindly.
        design=designImprovedObserverGains(cfg);
        return;
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    stored=jsondecode(fileread(fullfile(root,'config','continuousObserverCertificate.json')));
    source=stored.(cfg.mode);
    if cfg.mode=="lidar" && isfield(cfg.observer,'lidarGainProfile')
        assert(any(string(cfg.observer.lidarGainProfile)==["tracking","mncav"]), ...
            'VehicleLocalization:UnsupportedObserverProfile','Unknown LiDAR gain profile.');
        filename="lidarTrackingCertificate.json";
        if string(cfg.observer.lidarGainProfile)=="mncav",filename="mncavLidarCertificate.json";end
        source=jsondecode(fileread(fullfile(root,'config',filename)));
    end
    design=struct('kind',"continuous-mo-hgo-v1",'mode',cfg.mode,'theta',cfg.observer.theta);
    if isfield(cfg.synthesis,'certificateMethod')
        design.certificateMethod=cfg.synthesis.certificateMethod;
    end
    if cfg.mode=="gnss"
        design.P=source.P6;design.K=[source.K;zeros(1,2)];
        design.N=zeros(7,4);design.N(1:6,1:3)=source.N;
        design.N(7,4)=-cfg.observer.yawGain*design.theta^2;
    else
        for name=["P","Q","R","g","rate","K","N"],design.(name)=source.(name);end
    end
    design.verification=verifyImprovedObserverDesign(design,cfg);
    design.certified=design.verification.certified;
    assert(design.certified,'VehicleLocalization:CertificateMismatch', ...
        'The reference matrices do not certify the requested continuous configuration.');
end
