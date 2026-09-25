function distribution=computePillarRadialDistribution(points,pillarIds,geometry,peaks,evaluate)
% computePillarRadialDistribution: Metric radial mass profiles around pillar peaks.
% Every radius integrates original returns in whole neighboring pillars. No
% finer XY grid or height voxelization is constructed. Outputs follow sorted
% occupied pillar indices; unevaluated peaks have zero statistics.
% Radial profiles distinguish a compact shaft from a sheet or diffuse cloud.
    [ids,~,group]=unique(double(pillarIds(:))); n=numel(ids);
    radii=[.10 .15 .20 .25 .35 .45 .60 .75 .90];
    if nargin<5 || isempty(evaluate), evaluate=true(n,1); end
    assert(isequal(size(peaks),[n 2]) && numel(evaluate)==n, ...
        'perception:InvalidRadialDistribution','One peak and evaluation flag per occupied pillar are required.');
    names={'count','ownCount','height','heightStd','tilt','radialStd','xyLinearity','meanHeight','zSkewness','zKurtosis'};
    distribution=struct('pillarIndices',ids,'radii',radii);
    for name=names, distribution.(name{1})=zeros(n,numel(radii)); end
    selected=find(evaluate(:) & all(isfinite(peaks),2));
    if isempty(points) || isempty(selected), return; end
    points=double(points); mapSize=double(geometry.mapSize); spacing=double(geometry.cellSize);
    counts=accumarray(group,1); [~,order]=sort(group); first=cumsum([1;counts(1:end-1)]);
    [row,col]=ind2sub(mapSize,ids); reach=ceil(max(radii)./spacing);
    [dc,dr]=meshgrid(-reach(1):reach(1),-reach(2):reach(2));
    nr=row(selected)+dr(:).'; nc=col(selected)+dc(:).';
    valid=nr>=1 & nr<=mapSize(1) & nc>=1 & nc<=mapSize(2);
    lookup=zeros(mapSize); lookup(ids)=1:n; neighbor=zeros(size(nr));
    neighbor(valid)=lookup(sub2ind(mapSize,nr(valid),nc(valid)));
    owner=repmat((1:numel(selected)).',1,numel(dc)); active=neighbor>0;
    neighbor=neighbor(active); owner=owner(active); neighbor=neighbor(:); owner=owner(:);
    lengths=counts(neighbor); expanded=reshape(repelem(owner,lengths),[],1);
    starts=reshape(repelem(first(neighbor),lengths),[],1);
    before=reshape(repelem(cumsum(lengths)-lengths,lengths),[],1);
    index=order(starts+(1:sum(lengths)).'-before-1);
    p=points(index,:); p(:,1:2)=p(:,1:2)-peaks(selected(expanded),:);
    distance=vecnorm(p(:,1:2),2,2); same=group(index)==selected(expanded);
    m=numel(selected);
    for r=1:numel(radii)
        keep=distance<=radii(r); k=expanded(keep); q=p(keep,:);
        count=accumarray(k,1,[m 1]); inv=1./max(count,1);
        mu=[accumarray(k,q(:,1),[m 1]),accumarray(k,q(:,2),[m 1]),accumarray(k,q(:,3),[m 1])].*inv;
        centered=q-mu(k,:);
        xx=accumarray(k,centered(:,1).^2,[m 1]).*inv;
        yy=accumarray(k,centered(:,2).^2,[m 1]).*inv;
        zz=accumarray(k,centered(:,3).^2,[m 1]).*inv;
        xy=accumarray(k,centered(:,1).*centered(:,2),[m 1]).*inv;
        xz=accumarray(k,centered(:,1).*centered(:,3),[m 1]).*inv;
        yz=accumarray(k,centered(:,2).*centered(:,3),[m 1]).*inv;
        distribution.count(selected,r)=count;
        distribution.ownCount(selected,r)=accumarray(expanded(keep & same),1,[m 1]);
        top=accumarray(k,q(:,3),[m 1],@max,NaN); bottom=accumarray(k,q(:,3),[m 1],@min,NaN);
        height=top-bottom; height(~isfinite(height))=0;
        distribution.height(selected,r)=height;
        distribution.heightStd(selected,r)=sqrt(zz);
        distribution.tilt(selected,r)=atand(hypot(xz,yz)./max(zz,eps));
        distribution.radialStd(selected,r)=sqrt(max(0,xx+yy-(xz.^2+yz.^2)./max(zz,eps)));
        distribution.xyLinearity(selected,r)=sqrt((xx-yy).^2+4*xy.^2)./max(xx+yy,eps);
        distribution.meanHeight(selected,r)=mu(:,3);
        distribution.zSkewness(selected,r)=accumarray(k,centered(:,3).^3,[m 1]).*inv./max(zz.^1.5,eps);
        distribution.zKurtosis(selected,r)=accumarray(k,centered(:,3).^4,[m 1]).*inv./max(zz.^2,eps);
    end
end
