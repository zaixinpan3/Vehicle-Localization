function design=designFullObserverGains(cfg)
% designFullObserverGains Verify a common translation certificate by mode.
% GNSS and LiDAR correct position. Only qualified LiDAR yaw corrects the
% heading; otherwise the heading is the integrated gyro, so seven-state ISS
% needs the LiDAR heading channel and GNSS-only operation has no heading
% certificate. Measurement reconstruction errors enter as bounded disturbances.
    arguments
        cfg (1,1) struct=fullObserverConfig()
    end
    obsolete=intersect(string(fieldnames(cfg.gnss)).',["headingGain","courseWindow", ...
        "maximumCourseGap","minimumCourseDisplacement","minimumSpeed","headingErrorLimit"]);
    assert(isempty(obsolete),'VehicleLocalization:ObsoleteFullConfig', ...
        'GNSS does not correct the heading; remove cfg.gnss fields: %s.',strjoin(obsolete,', '));
    base=designMotionAidedObserverGains(cfg);g=cfg.gains;
    validateattributes(cfg.gnss.positionGain,{'numeric'},{'scalar','finite','positive'});
    validateattributes(cfg.gnss.minimumPositionWeight,{'numeric'},{'scalar','finite','>',0,'<=',1});
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
        'translationP',eye(6),'modes',["gnss","lidar","both"],'comparisonMatrices',matrices, ...
        'translationMargins',margin,'commonTranslationMargin',min(margin), ...
        'lidarHeadingRate',base.headingDecayRate, ...
        'matrixCertificateVerified',true,'unconditionalIssClaimed',false, ...
        'scope',"Common continuous translation bound; conditional local heading ISS only with qualified LiDAR yaw. GNSS-only operation integrates the gyro heading without correction; no ISS during arbitrary simultaneous outages.");
end
