function cloud=applyCurbBoundaryScatter(cloud,cfg)
% applyCurbBoundaryScatter Represent boundary scatter, not mixed road width.
% Preserve tangential scatter and propagate XY/Z cross terms by congruence.
% This is an engineering geometric covariance, not an empirical point moment
% or an independently calibrated covariance of the estimated boundary mean.
    c=cloud.components;ids=find(c.semanticName=="curb" & c.mixtureWeight>0);
    p=unique(c.mean(ids,:),'rows');support=cfg.lineSupport;
    for id=ids.'
        q=p(sum((p-c.mean(id,:)).^2,2)<=support.radius^2,:);
        if size(q,1)<support.minimumComponents,continue;end
        d=q-mean(q,1);[v,e]=eig(d.'*d/size(d,1),'vector');[major,j]=max(e);
        t=v(:,j);along=d*t;
        if major<support.minimumAnisotropy*max(min(e),1e-6) || max(along)-min(along)<support.minimumSpan,continue;end
        n=[-t(2);t(1)];near=c.semanticName=="curb" & sum((c.mean-c.mean(id,:)).^2,2)<=support.radius^2;
        delta=c.mean(near,:)-mean(c.mean(near,:),1);normalVariance=mean((delta*n).^2);
        original=n.'*c.covariance(:,:,id)*n;
        desired=max(cfg.minimumNormalVariance,min(original,normalVariance));
        L=t*t.'+sqrt(desired/original)*(n*n.');S=L*c.covariance(:,:,id)*L.';
        c.covariance(:,:,id)=(S+S.')/2;c.invCovariance(:,:,id)=S\eye(2);
        c.determinant(id)=det(S);c.logNormalizationConstant(id)=-log(2*pi)-.5*log(det(S));
        L3=blkdiag(L,1);c.covarianceXYZ(:,:,id)=L3*c.covarianceXYZ(:,:,id)*L3.';
    end
    cloud.components=c;
    cloud.curbGeometryModel="withinSelectedPillarBoundaryFitWithLocalNormalScatter";
    cloud.curbGeometryCovarianceCalibrated=false;
end
