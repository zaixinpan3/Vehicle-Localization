function design=designFullObserverGains(cfg)
% designFullObserverGains Verify a common translation certificate by mode.
% GNSS yaw uses a causal position-displacement/motion heading reconstruction.
% It is NOT an independent heading sensor or the legacy GNSS-only theorem.
% Measurement reconstruction errors enter as bounded disturbances. Seven-state
% ISS also needs an informative heading channel and a local heading chart.
    arguments
        cfg (1,1) struct=fullObserverConfig()
    end
    base=designMotionAidedObserverGains(cfg);g=cfg.gains;
    validateattributes(cfg.gnss.positionGain,{'numeric'},{'scalar','finite','positive'});
    validateattributes(cfg.gnss.headingGain,{'numeric'},{'scalar','finite','positive'});
    validateattributes(cfg.gnss.minimumPositionWeight,{'numeric'},{'scalar','finite','>',0,'<=',1});
    validateattributes(cfg.gnss.headingErrorLimit,{'numeric'},{'scalar','finite','>',0,'<',pi/2});
    a=[cfg.gnss.positionGain*cfg.gnss.minimumPositionWeight, ...
        g(1)*cfg.lidar.minimumPoseWeight, ...
        cfg.gnss.positionGain*cfg.gnss.minimumPositionWeight+g(1)*cfg.lidar.minimumPoseWeight];
    matrices=zeros(3,3,2,3);margin=zeros(3,1);
    for mode=1:3
        for vertex=1:2
            q2=(vertex-1)*cfg.maximumTrackAngleRate^2;
            matrices(:,:,vertex,mode)=[2*a(mode),-1,0;-1,2*g(2),-(1+q2);0,-(1+q2),2*g(3)];
        end
        margin(mode)=min([min(eig(matrices(:,:,1,mode))),min(eig(matrices(:,:,2,mode)))]);
    end
    assert(all(margin>1e-9),'VehicleLocalization:InfeasibleFullCertificate', ...
        'GNSS, LiDAR and combined translation gains must share positive dissipation.');
    design=struct('kind',cfg.kind,'gains',g,'gnssPositionGain',cfg.gnss.positionGain, ...
        'gnssHeadingGain',cfg.gnss.headingGain,'translationP',eye(6), ...
        'modes',["gnss","lidar","both"],'comparisonMatrices',matrices, ...
        'translationMargins',margin,'commonTranslationMargin',min(margin), ...
        'lidarHeadingRate',base.headingDecayRate, ...
        'gnssHeadingRate',cfg.gnss.headingGain*sin(cfg.gnss.headingErrorLimit)/cfg.gnss.headingErrorLimit, ...
        'matrixCertificateVerified',true,'unconditionalIssClaimed',false, ...
        'scope',"Common continuous translation bound; conditional local heading ISS with available displacement/pose heading. No ISS during arbitrary simultaneous outages or stopped GNSS-only operation.");
end
