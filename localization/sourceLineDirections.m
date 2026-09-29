function [tangent,valid,angularScatter]=sourceLineDirections(c,cfg)
% sourceLineDirections Estimate local unoriented curb axes from selected means.
% A direction requires multiple spatially separated, nearly collinear means.
% No reference pose, map geometry, point labels or frame index is used.
    values=[cfg.radius,cfg.minimumComponents,cfg.minimumAnisotropy,cfg.minimumSpan,cfg.standardDeviation];
    assert(isreal(values)&&all(isfinite(values)&values>0) && cfg.minimumComponents>=3 ...
        && cfg.minimumComponents==fix(cfg.minimumComponents) && cfg.minimumAnisotropy>1 ...
        && cfg.standardDeviation<pi/2,'VehicleLocalization:InvalidLineDirectionConfiguration', ...
        'Line-direction neighborhoods and angular scale must be finite and positive.');
    n=c.numComponents;tangent=zeros(n,2);valid=false(n,1);angularScatter=zeros(n,1);
    for name=["curb","facade"]
        eligible=c.semanticName==name & c.mixtureWeight>0;
        if isfield(c,'quality'),eligible=eligible & c.quality>0;end
        ids=find(eligible);centers=c.mean(ids,1:2);p=unique(centers,'rows');
        for k=1:numel(ids)
            keep=sum((p-centers(k,:)).^2,2)<=cfg.radius^2;q=p(keep,:);
            if size(q,1)<cfg.minimumComponents,continue;end
            d=q-mean(q,1);cov=d.'*d/size(d,1);[v,e]=eig(cov,'vector');[major,j]=max(e);
            axis=v(:,j);along=d*axis;
            if major<cfg.minimumAnisotropy*max(min(e),1e-6) || (max(along)-min(along))<cfg.minimumSpan,continue;end
            tangent(ids(k),:)=axis.';valid(ids(k))=true;
            angularScatter(ids(k))=sqrt(max(0,min(e))/max(major,eps));
        end
    end
end
