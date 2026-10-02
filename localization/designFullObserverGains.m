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
    if isfield(cfg,'timing') && cfg.timing=="synchronous"
        design=matchedDesign(cfg);
        return;
    end
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

function design=matchedDesign(cfg)
% Only translation has a stable GNSS baseline in the current architecture.
% Extend that fixed storage to yaw to match the LiDAR adjoint, without
% relabeling the extension as a seven-state baseline ISS certificate.
    g=cfg.gains(:);
    validateattributes(g,{'numeric'},{'real','finite','positive','numel',4});
    validateattributes(cfg.maximumTrackAngleRate,{'numeric'},{'real','finite','nonnegative','scalar'});
    validateattributes(cfg.gnss.positionGain,{'numeric'},{'real','finite','positive','scalar'});
    validateattributes(cfg.gnss.minimumPositionWeight,{'numeric'},{'real','finite','>',0,'<=',1,'scalar'});
    filterLidarPoseInformation(zeros(3),cfg);
    alpha=cfg.gnss.positionGain*cfg.gnss.minimumPositionWeight;
    matrices=zeros(3,3,2);margins=zeros(2,1);
    for k=1:2
        q2=(k-1)*cfg.maximumTrackAngleRate^2;
        matrices(:,:,k)=[2*alpha,-1,0;-1,2*g(2),-(1+q2);0,-(1+q2),2*g(3)];
        margins(k)=min(eig(matrices(:,:,k)));
    end
    assert(all(margins>1e-9),'VehicleLocalization:InfeasibleFullCertificate', ...
        'GNSS baseline translation gains must have positive conditional dissipation.');
    matched=struct('P',blkdiag(eye(6),g(1)/g(4)),'T',eye(7),'theta',1, ...
        'scalingExponents',[1;2;3;1;2;3;1],'kappa',g(1), ...
        'baselineFullStateCertified',false, ...
        'metricSource',"Existing physical translation P=I6, extended with fixed yaw weight kp/kpsi");
    design=struct('kind',cfg.kind,'gains',g.','gnssPositionGain',cfg.gnss.positionGain, ...
        'translationP',eye(6),'comparisonMatrices',matrices, ...
        'translationMargins',margins,'commonTranslationMargin',min(margins), ...
        'matrixCertificateVerified',true,'unconditionalIssClaimed',false, ...
        'baselineFullStateCertified',false,'lidarMatched',matched, ...
        'scope',"Conditional GNSS baseline translation bound with heading/model errors as inputs; fixed-metric LiDAR dissipation only. No full-state certificate without an absolute heading channel or during arbitrary outages.");
end
