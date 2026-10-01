function [residual,J,precision,shapeUsed]=gaussianRegistrationResiduals(sourceMean,sourceCov,targetMean,targetCov,pose,noiseStd)
% gaussianRegistrationResiduals Normalized Gaussian overlap in planar SE(2).
% Squared residual = delta'/(A+B)*delta + log(det(A+B)/(4*sqrt(det(A)*det(B)))).
% A and B each receive half the isotropic noise variance. Both covariance
% axes contribute continuously; isotropic shapes provide no rotation factor.
% J differentiates the rotated covariance as well as the transformed center.
    n=size(sourceMean,1);r=[cos(pose(3)) -sin(pose(3));sin(pose(3)) cos(pose(3))];
    rotated=pagemtimes(pagemtimes(r,sourceCov),r.');
    ax=reshape(rotated(1,1,:),[],1);bx=reshape(rotated(1,2,:),[],1);dx=reshape(rotated(2,2,:),[],1);
    at=reshape(targetCov(1,1,:),[],1);bt=reshape(targetCov(1,2,:),[],1);dt=reshape(targetCov(2,2,:),[],1);
    noise=noiseStd^2/2;
    a=ax+at+2*noise;b=bx+bt;d=dx+dt+2*noise;detS=a.*d-b.^2;
    delta=sourceMean*r.'+pose(1:2)-targetMean;
    l11=sqrt(a);l21=b./l11;l22=sqrt(d-l21.^2);
    rx=delta(:,1)./l11;ry=(delta(:,2)-l21.*rx)./l22;
    detA=(ax+noise).*(dx+noise)-bx.^2;detB=(at+noise).*(dt+noise)-bt.^2;
    shape=max(0,log(detS)-log(4)-.5*(log(detA)+log(detB)));
    angle=.5*(atan2(2*bx,ax-dx)-atan2(2*bt,at-dt));
    gapA=hypot(ax-dx,2*bx);gapB=hypot(at-dt,2*bt);
    traceSum=a+d;alignedDet=(traceSum+gapA+gapB).*(traceSum-gapA-gapB)/4;
    kappa=gapA.*gapB./alignedDet;u=kappa.*sin(angle).^2;orientationCost=log1p(u);
    % Separate pose-invariant scale mismatch from smooth angular residuals.
    % A single sqrt(total shape cost) loses Gauss-Newton curvature whenever
    % unequal cloud scales leave a nonzero residual at aligned axes.
    ratio=ones(n,1);active=u>1e-12;ratio(active)=orientationCost(active)./u(active);
    orientation=sin(angle).*sqrt(kappa.*ratio);
    residual=[rx,ry,orientation,sqrt(max(0,shape-orientationCost))].';
    precision=zeros(2,2,n);precision(1,1,:)=1./l11;
    precision(2,1,:)=-l21./l11./l22;precision(2,2,:)=1./l22;
    J=zeros(4,3,n);J(1:2,1:2,:)=precision;
    meanYaw=sourceMean*[r(:,2),-r(:,1)].';
    ap=-2*bx;bp=ax-dx;dp=2*bx;
    l11p=ap./(2*l11);l21p=bp./l11-l21.*l11p./l11;
    l22p=(dp-2*l21.*l21p)./(2*l22);
    rxp=meanYaw(:,1)./l11-rx.*l11p./l11;
    ryp=(meanYaw(:,2)-l21p.*rx-l21.*rxp)./l22-ry.*l22p./l22;
    J(1,3,:)=rxp;J(2,3,:)=ryp;
    shapeYaw=sqrt(kappa).*cos(angle);active=orientationCost>1e-12;
    shapeYaw(active)=sign(sin(angle(active))).*kappa(active).*sin(angle(active)).*cos(angle(active))./ ...
        ((1+u(active)).*sqrt(orientationCost(active)));
    J(3,3,:)=shapeYaw;
    shapeUsed=gapA.*gapB>0;
end
