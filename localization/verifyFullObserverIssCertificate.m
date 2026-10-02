function verification=verifyFullObserverIssCertificate(certificate,cfg)
% verifyFullObserverIssCertificate Recheck the continuous LMIs without a solver.
% P certifies the complete continuous observer. It is distinct from the
% algebraic metric used to implement the LiDAR injection. Physical bounds,
% information availability and the local heading chart remain assumptions.
    arguments
        certificate (1,1) struct
        cfg (1,1) struct
    end
    p=fullObserverIssProblem(cfg);
    assert(isfield(certificate,'problem') && isequaln(certificate.problem,p), ...
        'VehicleLocalization:StaleFullIssCertificate', ...
        'Re-solve the continuous LMIs after changing gains or design assumptions.');
    w=certificate.weights(:);
    validateattributes(w,{'numeric'},{'real','finite','positive','numel',3});
    cp=w(1);cy=w(2);mu=w(3);g=p.gains;
    W=diag([cp,cy,1,mu]);root=sqrt(diag(W));
    matrices=zeros(4,4,2);margins=zeros(2,1);rates=zeros(2,1);
    for vertex=1:2
        q2=(vertex-1)*p.maximumCourseRate^2;
        a=cp*g(1);b=cy*g(4);cross=-(a+b)*p.maximumPositionHeadingCoupling;
        M=[2*a*p.minimumPositionStrength,cross,-cp,0; ...
            cross,2*b*p.minimumHeadingStrength-cp*p.gnssLeverPenalty, ...
            -g(2)*p.maximumSpeed,-mu*g(3)*p.maximumAcceleration; ...
            -cp,-g(2)*p.maximumSpeed,2*g(2),-(1+mu*q2); ...
            0,-mu*g(3)*p.maximumAcceleration,-(1+mu*q2),2*mu*g(3)];
        matrices(:,:,vertex)=M;
        margins(vertex)=min(eig((M-p.decayRate*W)./(root*root.')));
        rates(vertex)=min(eig(M./(root*root.')));
    end
    P=diag([cp,1,mu,cp,1,mu,cy]);
    verification=struct('certified',all(margins>p.tolerance), ...
        'weights',w,'P',P,'comparisonMatrices',matrices,'comparisonWeight',W, ...
        'normalizedMargins',margins,'verifiedDecayRate',min(rates), ...
        'requestedDecayRate',p.decayRate,'continuousTime',true, ...
        'sampledSystemCertified',false, ...
        'scope',"Seven-state continuous ISS under the declared maneuver and uniformly sufficient LiDAR information bounds; estimated-heading GNSS lever arm included. No arbitrary-outage or numerical-integration guarantee.");
end
