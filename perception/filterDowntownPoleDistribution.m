function mask=filterDowntownPoleDistribution(maps,cfg)
% filterDowntownPoleDistribution: Reject horizontally spread pillar context.
% Merge all members of neighboring pillars using population sufficient
% statistics. No point subset, height slice, fit or point neighborhood is used.
% Radial variance is the horizontal residual after linear dependence on Z;
% its threshold has units of square meters. The result gates seed pillars,
% before a small statistical candidate halo preserves sparse pole members.
    stats=maps.statistics;ids=double(stats.pillarIndices);dims=maps.mapSize;
    mask=false(dims);
    if isempty(ids),return;end
    count=zeros(dims);count(ids)=stats.count;
    kernel=ones(2*cfg.radiusCells+1);total=conv2(count,kernel,'same');
    n=reshape(total(ids),[],1);mean=zeros(numel(ids),3);
    for axis=1:3
        raster=zeros(dims);raster(ids)=stats.count.*stats.meanXYZ(:,axis);
        merged=conv2(raster,kernel,'same');
        mean(:,axis)=reshape(merged(ids),[],1)./max(n,1);
    end
    pairs=[1 1;1 2;2 2;1 3;2 3;3 3];covariance=zeros(numel(ids),6);
    for k=1:6
        a=pairs(k,1);b=pairs(k,2);raster=zeros(dims);
        raster(ids)=stats.count.*(stats.covarianceXYZ(:,k)+stats.meanXYZ(:,a).*stats.meanXYZ(:,b));
        merged=conv2(raster,kernel,'same');
        covariance(:,k)=reshape(merged(ids),[],1)./max(n,1)-mean(:,a).*mean(:,b);
    end
    varianceZ=max(0,covariance(:,6));
    vertical=varianceZ./max(covariance(:,1)+covariance(:,3)+varianceZ,eps);
    radial=max(0,covariance(:,1)+covariance(:,3)- ...
        (covariance(:,4).^2+covariance(:,5).^2)./max(varianceZ,eps));
    mask(ids)=vertical>=cfg.minimumVerticalVarianceFraction & radial<=cfg.maximumRadialVariance;
end
