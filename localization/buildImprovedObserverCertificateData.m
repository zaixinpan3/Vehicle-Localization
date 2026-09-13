function data = buildImprovedObserverCertificateData(cfg)
% buildImprovedObserverCertificateData Matrices for the two continuous modes.
    arguments
        cfg (1,1) struct
    end
    assert(isfield(cfg,'mode') && any(string(cfg.mode)==["gnss","lidar"]), ...
        'VehicleLocalization:InvalidContinuousMode','Select gnss or lidar mode.');
    theta=cfg.observer.theta;
    assert(isscalar(theta) && isreal(theta) && isfinite(theta) && theta>=1, ...
        'VehicleLocalization:InvalidConfiguration','theta must be finite and >=1.');
    assert(isequal(cfg.observer.scalingExponents(:),[1;2;3;1;2;3;1]), ...
        'VehicleLocalization:InvalidConfiguration','Unexpected state scaling exponents.');
    limits=[cfg.operating.maximumSpeed,cfg.operating.maximumAcceleration,cfg.operating.maximumTrackAngleRate];
    assert(isreal(limits) && all(isfinite(limits)) && all(limits>=0) && all(limits(1:2)>0), ...
        'VehicleLocalization:InvalidConfiguration','Operating bounds must be finite and nonnegative.');
    scales=cfg.lidar.poseScales(:);
    assert(isreal(scales) && numel(scales)==3 && all(isfinite(scales) & scales>0), ...
        'VehicleLocalization:InvalidConfiguration','Pose scales must be positive.');
    assert(isfinite(cfg.lidar.minimumPoseWeight) && cfg.lidar.minimumPoseWeight>0 ...
        && cfg.lidar.minimumPoseWeight<=1 && isfinite(cfg.lidar.gainInformationScale) ...
        && cfg.lidar.gainInformationScale>0, ...
        'VehicleLocalization:InvalidConfiguration','Invalid information sector or scale.');
    chain=[0,1,0;0,0,1;0,0,0];
    data.A=blkdiag(chain,chain,0); data.T=diag(theta.^cfg.observer.scalingExponents);
    data.C=zeros(3,7);
    data.C(1,1)=1/scales(1);data.C(2,4)=1/scales(2);data.C(3,7)=1/scales(3);
    data.Cg=zeros(2,6);data.Cg(1,1)=1;data.Cg(2,4)=1;
    data.outputBound3=sqrt(12*limits(1)^2+4*limits(2)^2);
    data.outputBound4=sqrt(16*limits(1)^2+4*limits(2)^2+2);
    data.modelPerturbation=limits(3)*sqrt(limits(3)^2/theta^2+4);
end
