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
    signShape=sign(sin(angle));signShape(signShape==0)=1;
    residual=[rx,ry,signShape.*sqrt(shape)].';
    precision=zeros(2,2,n);precision(1,1,:)=1./l11;
    precision(2,1,:)=-l21./l11./l22;precision(2,2,:)=1./l22;
    J=zeros(3,3,n);J(1:2,1:2,:)=precision;
    meanYaw=sourceMean*[r(:,2),-r(:,1)].';
    ap=-2*bx;bp=ax-dx;dp=2*bx;
    l11p=ap./(2*l11);l21p=bp./l11-l21.*l11p./l11;
    l22p=(dp-2*l21.*l21p)./(2*l22);
    rxp=meanYaw(:,1)./l11-rx.*l11p./l11;
    ryp=(meanYaw(:,2)-l21p.*rx-l21.*rxp)./l22-ry.*l22p./l22;
    J(1,3,:)=rxp;J(2,3,:)=ryp;
    shapePrime=(d.*ap+a.*dp-2*b.*bp)./detS;
    shapeYaw=zeros(n,1);nonzero=shape>1e-12;
    shapeYaw(nonzero)=signShape(nonzero).*shapePrime(nonzero)./(2*sqrt(shape(nonzero)));
    % The signed square root has a finite derivative at identical shapes.
    % Eigenvalue gaps vanish continuously as either distribution becomes round.
    gapProduct=hypot(ax-dx,2*bx).*hypot(at-dt,2*bt);
    aligned=~nonzero & abs(sin(angle))<1e-5;
    shapeYaw(aligned)=cos(angle(aligned)).*sqrt(gapProduct(aligned)./detS(aligned));
    J(3,3,:)=shapeYaw;
    shapeUsed=gapProduct>0;
end
