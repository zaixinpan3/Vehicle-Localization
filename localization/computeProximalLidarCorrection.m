function [delta,posterior,audit]=computeProximalLidarCorrection(measurement,pose,G,prior,cfg)
% computeProximalLidarCorrection Unique convex Huber proximal state update.
% Minimize .5*delta'*prior^-1*delta + huber(norm(L*(G*delta-d)),c).
% PRIOR is a propagated design uncertainty, not the old fixed Lyapunov metric.
% Geometry supplies only relative directional precision; POSESTD supplies the
% physical scale. Uniform rescaling of the geometric objective is immaterial.
% A PSD local Hessian preserves unobserved directions. The posterior is a
% local curvature metric, not an exact non-Gaussian posterior covariance.
    arguments
        measurement (1,1) struct
        pose (3,1) double {mustBeReal,mustBeFinite}
        G double {mustBeReal,mustBeFinite}
        prior double {mustBeReal,mustBeFinite}
        cfg (1,1) struct=proximalLidarConfig()
    end
    n=size(prior,1);assert(isequal(size(prior),[n,n]) && isequal(size(G),[3,n]));
    assert(norm(prior-prior.','fro')<=1e-10*max(1,norm(prior,'fro')));
    [~,flag]=chol(prior);assert(flag==0,'VehicleLocalization:InvalidProximalPrior','Prior must be SPD.');
    sigma=cfg.poseStd(:);validateattributes(sigma,{'double'},{'positive','finite','numel',3});
    c=cfg.huberThreshold;assert(isscalar(c) && isreal(c) && c>0);
    residualMode=isfield(measurement,'residual');
    if residualMode
        assert(isequal(measurement.linearizationPose(:),pose),'VehicleLocalization:StaleLidarLinearization','Require current-pose residuals.');
        J=measurement.jacobian;r=measurement.residual(:);W=measurement.weights;
        if isvector(W),W=diag(W);end
        assert(isequal(size(J),[numel(r),3]) && isequal(size(W),[numel(r),numel(r)]));
        assert(all(isfinite(J),'all') && all(isfinite(r)) && all(isfinite(W),'all'));
        assert(norm(W-W.','fro')<=1e-10*max(1,norm(W,'fro')) && min(eig((W+W.')/2))>=-1e-10);
        H=J.'*W*J;gradient=J.'*W*r;
    else
        H=measurement.information;d=measurement.pose(:)-pose;d(3)=atan2(sin(d(3)),cos(d(3)));
    end
    assert(isequal(size(H),[3,3]) && all(isfinite(H),'all') && norm(H-H.','fro')<=1e-10*max(1,norm(H,'fro')));
    D=diag(sigma);Hn=D*((H+H.')/2)*D;[U,E]=eig(Hn);[h,order]=sort(diag(E));U=U(:,order);
    assert(min(h)>=-1e-10*max(1,max(abs(h))),'VehicleLocalization:InvalidLidarInformation','Require PSD geometry.');h=max(h,0);
    rho=ones(3,1);
    if isfield(measurement,'directionReliability'),rho=measurement.directionReliability(:);end
    assert(numel(rho)==3 && all(isfinite(rho)) && all(rho>=0 & rho<=1));
    frame=1;if isfield(measurement,'frameReliability'),frame=measurement.frameReliability;end
    assert(isscalar(frame) && isfinite(frame) && frame>=0 && frame<=1);rho=rho*frame;
    % Reliability keeps the upstream information-coordinate convention, even
    % when noise whitening reorders or rotates the geometric eigenspaces.
    scales=cfg.informationScales(:);validateattributes(scales,{'double'},{'positive','finite','numel',3});
    Dg=diag(scales);[Ug,Eg]=eig(Dg*((H+H.')/2)*Dg);[hg,order]=sort(diag(Eg));Ug=Ug(:,order);hg=max(hg,0);
    tolerance=1e-10*max(hg(end),realmin);first=1;
    while first<=3
        last=first;while last<3 && hg(last+1)-hg(first)<=tolerance,last=last+1;end
        rho(first:last)=min(rho(first:last));first=last+1;
    end
    reliableH=Dg\(Ug*diag(rho.*hg)*Ug.')/Dg;
    reliableNormalized=D*reliableH*D;[Vg,Eg]=eig((reliableNormalized+reliableNormalized.')/2);shape=max(diag(Eg),0);
    active=shape>1e-10*max(max(shape),realmin);
    delta=zeros(n,1);posterior=prior;
    audit=struct('rank',nnz(active),'robustWeight',1,'residualNorm',0,'stationarityResidual',0, ...
        'incrementEnergy',0,'precision',zeros(3),'correctionMatrix',zeros(n,3), ...
        'geometryShape',zeros(3),'scope',"Convex frozen-measurement proximal map; no nonlinear whole-system certificate");
    if ~any(active),return;end
    L=diag(sqrt(shape(active)/h(end)))*Vg(:,active).'/D;
    if residualMode
        % Solve only in the observable normalized subspace; never fill a nullspace.
        observed=h>1e-10*h(end);
        d=-D*U(:,observed)*((U(:,observed).'*D*gradient)./h(observed));
    end
    assert(all(isfinite(d)));y=L*d;B=L*G;T=B*prior*B.';T=(T+T.')/2;
    [V,E]=eig(T);ev=max(diag(E),0);v=V.'*y;
    weight=1;residual=V*(v./(1+ev));
    if norm(residual)>c
        lo=0;hi=1;
        for j=1:60
            mid=(lo+hi)/2;
            if mid*norm(v./(1+mid*ev))>c,hi=mid;else,lo=mid;end
        end
        weight=(lo+hi)/2;residual=V*(v./(1+weight*ev));
    end
    delta=prior*B.'*(weight*residual);
    K=weight*(prior*B.')*((eye(numel(ev))+weight*T)\L);
    % Exact Huber curvature loses the clipped radial information direction.
    curvature=eye(numel(ev));
    if weight<1
        u=residual/norm(residual);curvature=weight*(eye(numel(ev))-u*u.');
    end
    [V,E]=eig((curvature+curvature.')/2);A=diag(sqrt(max(diag(E),0)))*V.'*B;
    KA=(prior*A.')/(eye(size(A,1))+A*prior*A.');M=eye(n)-KA*A;
    posterior=M*prior*M.'+KA*KA.';posterior=(posterior+posterior.')/2;
    audit.robustWeight=weight;audit.residualNorm=norm(residual);
    audit.stationarityResidual=norm(prior\delta-B.'*(weight*residual));
    audit.incrementEnergy=delta.'*(prior\delta);audit.precision=L.'*L;
    audit.correctionMatrix=K;audit.geometryShape=D*audit.precision*D;
end
