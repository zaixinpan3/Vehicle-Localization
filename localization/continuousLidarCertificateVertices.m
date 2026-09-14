function [vertices,uncertainty]=continuousLidarCertificateVertices(design,cfg)
% continuousLidarCertificateVertices Enclose the scaled drift for all rates.
% The structured method embeds (q,q^2) in [-b,b] x [0,b^2]. The Schur LMI
% is affine in both coordinates, so four vertices cover the entire curve,
% including arbitrarily time-varying q. P/Q/R are common to all vertices.
% Auxiliary-output and information uncertainty retain their norm enclosure.
    data=buildImprovedObserverCertificateData(cfg);theta=cfg.observer.theta;
    method="norm-ball";
    if isfield(design,'certificateMethod'),method=string(design.certificateMethod);end
    assert(isscalar(method) && any(method==["norm-ball","course-rate-polytope"]), ...
        'VehicleLocalization:CertificateMismatch','Unknown certificate method.');
    uncertainty=norm(design.N,2)*data.outputBound4+ ...
        theta*norm(design.K,2)*norm(data.C,2)*(1-cfg.lidar.minimumPoseWeight);
    if method=="norm-ball"
        vertices=theta*data.A;uncertainty=uncertainty+data.modelPerturbation;
        return;
    end
    bound=cfg.operating.maximumTrackAngleRate;vertices=zeros(7,7,4);k=0;
    for q=[-bound,bound]
        for squaredRate=[0,bound^2]
            k=k+1;A0=theta*data.A;
            A0(3,2)=squaredRate/theta;A0(6,5)=squaredRate/theta;
            A0(3,6)=-2*q;A0(6,3)=2*q;vertices(:,:,k)=A0;
        end
    end
end
