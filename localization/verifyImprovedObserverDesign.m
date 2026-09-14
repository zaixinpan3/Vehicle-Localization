function verification = verifyImprovedObserverDesign(design,cfg)
% verifyImprovedObserverDesign Recompute a constant-matrix ISS certificate.
% Stored certified flags are ignored. This verifies continuous equations,
% not numerical integration error, true-state bounds or unknown sensor errors.
    arguments
        design (1,1) struct
        cfg (1,1) struct
    end
    data=buildImprovedObserverCertificateData(cfg);
    assert(isfield(design,'kind') && string(design.kind)=="continuous-mo-hgo-v1" ...
        && string(design.mode)==cfg.mode && design.theta==cfg.observer.theta, ...
        'VehicleLocalization:CertificateMismatch','Design must match the continuous mode and theta.');
    n=7; outputs=3;
    if cfg.mode=="gnss",n=6;outputs=2;end
    requireMatrix(design.K,[7,outputs]);requireMatrix(design.N,[7,4]);requireMatrix(design.P,[n,n]);
    P=design.P;assert(norm(P-P.','fro')<1e-10 && min(eig(P))>0, ...
        'VehicleLocalization:CertificateMismatch','P must be symmetric positive definite.');
    theta=cfg.observer.theta;
    if cfg.mode=="gnss"
        assert(all(design.K(7,:)==0) && all(design.N(1:6,4)==0) ...
            && all(design.N(7,1:3)==0) && design.N(7,4)==-cfg.observer.yawGain*theta^2 ...
            && isfinite(cfg.observer.yawGain) && cfg.observer.yawGain>0, ...
            'VehicleLocalization:CertificateMismatch','GNSS requires the triangular gain and positive yaw gain.');
        assert(cfg.gnss.minimumSpeed>0 && isfinite(cfg.gnss.minimumSpeed) ...
            && cfg.gnss.headingErrorLimit>0 && cfg.gnss.headingErrorLimit<pi/2, ...
            'VehicleLocalization:InvalidConfiguration','GNSS needs positive speed and a local yaw sector.');
        A0=theta*(data.A(1:6,1:6)-design.K(1:6,:)*data.Cg);
        nominal=-max(eig(P*A0+A0.'*P));
        margin=nominal-2*norm(P,2)*(data.modelPerturbation+ ...
            norm(design.N(1:6,1:3),2)*data.outputBound3);
        yawRate=cfg.observer.yawGain*cfg.gnss.minimumSpeed* ...
            sin(cfg.gnss.headingErrorLimit)/cfg.gnss.headingErrorLimit;
        c=max(margin,0)*yawRate/(8*(cfg.observer.yawGain*theta^2)^2);
        rate=min(margin/(2*norm(P,2)),yawRate)/2;
        extra=struct('yawMetricWeight',c,'headingAdmissionRequired',true);
    else
        requireMatrix(design.Q,[7,7]);requireMatrix(design.R,[7,7]);
        Q=design.Q;R=design.R;
        assert(norm(Q-Q.','fro')<1e-10 && norm(R-R.','fro')<1e-10 ...
            && min([eig(Q);eig(R)])>0 && isfinite(design.g) && design.g>0 ...
            && isfinite(design.rate) && design.rate>0, ...
            'VehicleLocalization:CertificateMismatch','Invalid constant delay-functional matrices.');
        delay=cfg.measurement.fixedLidarDelay;
        assert(isreal(delay) && isscalar(delay) && isfinite(delay) && delay>=0, ...
            'VehicleLocalization:InvalidConfiguration','The LiDAR delay must be finite and nonnegative.');
        [vertices,uncertainty]=continuousLidarCertificateVertices(design,cfg);
        Ad=-theta*design.K*data.C;vertexMargins=zeros(size(vertices,3),1);
        for vertex=1:size(vertices,3)
            block=continuousObserverDelayLmi(vertices(:,:,vertex),Ad,P,Q,R,design.g,delay,design.rate);
            vertexMargins(vertex)=-max(eig((block+block.')/2));
        end
        nominal=min(vertexMargins);
        margin=nominal-2*(norm(P,2)+delay*norm(R,2))*uncertainty;
        rate=design.rate;extra=struct('delaySeconds',delay,'headingAdmissionRequired',false, ...
            'vertexMargins',vertexMargins,'remainingPerturbationBound',uncertainty);

    end
    verification=struct('certified',margin>cfg.synthesis.tolerance, ...
        'nominalMargin',nominal,'uniformMargin',margin,'rate',rate, ...
        'constantMetric',true,'details',extra, ...
        'scope',"Conditional continuous-time ISS matrix certificate; numerical and physical assumptions remain separate.");
end

function requireMatrix(value,shape)
    assert(isnumeric(value) && isreal(value) && isequal(size(value),shape) && all(isfinite(value),'all'), ...
        'VehicleLocalization:CertificateMismatch','Invalid certificate matrix shape or entries.');
end
