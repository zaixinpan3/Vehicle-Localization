function result=measureAxialPillars(points,pillarIds,geometry,settings)
% measureAxialPillars: Robust axial distribution hypotheses of whole pillars.
% All returns belong to their original pillar. Height order statistics seed
% line fits; there is no vertical lattice or finer XY semantic detector.
    if nargin<4, settings=struct(); end
    defaults=struct('radius',0.15,'outerRadius',0.6,'gapBase',0.25,'gapRangeScale',0.012, ...
        'localSupport',false,'localHalfHeight',0.25,'localMinimumFraction',0.65,'localMinimumCount',3, ...
        'envelopeRadius',0.25,'maximumTiltDegrees',20,'minimumOwnPoints',6,'minimumOwnHeight',0.6);
    for name=string(fieldnames(defaults)).'
        if ~isfield(settings,name), settings.(name)=defaults.(name); end
    end
    p=double(points); [ids,~,group]=unique(double(pillarIds)); n=numel(ids);
    names={'height','robustHeight','count','ownCount','ownFraction','isolation','radialStd', ...
        'tilt','maximumGap','gapRatio','ownHeight','score','centerX','centerY','centerZ','slopeX','slopeY','minimumZ','maximumZ','envelopeIsolation','interquartileRatio','residualLinearity','envelopeFraction','medianGap','lowerIsolation','upperIsolation'};
    wholeNames={'wholeGapRatio','wholeInterquartileRatio','wholeSkewness','wholeKurtosis','wholeLowerHalfHeight','wholeUpperHalfHeight'};
    for name=names, result.(name{1})=zeros(n,1); end
    for name=wholeNames, result.(name{1})=zeros(n,1); end
    result.pillarIndices=ids;
    if isempty(p), return; end
    counts=accumarray(group,1); zmin=accumarray(group,p(:,3),[],@min); zmax=accumarray(group,p(:,3),[],@max);
    selected=find(counts>=settings.minimumOwnPoints & zmax-zmin>=settings.minimumOwnHeight);
    [~,order]=sort(group); first=cumsum([1;counts(1:end-1)]);
    lookup=zeros(geometry.mapSize); lookup(ids)=1:n;
    [rows,cols]=ind2sub(geometry.mapSize,ids);
    windows=[0 1;0 .5;.25 .75;.5 1;0 .25;.75 1];
    for j=selected.'
        ownIndex=order(first(j):first(j)+counts(j)-1); own=p(ownIndex,:);
        reach=ceil(settings.outerRadius./geometry.cellSize);
        rr=max(1,rows(j)-reach(2)):min(geometry.mapSize(1),rows(j)+reach(2));
        cc=max(1,cols(j)-reach(1)):min(geometry.mapSize(2),cols(j)+reach(1));
        neighbors=lookup(rr,cc); neighbors=neighbors(neighbors>0);
        pieces=arrayfun(@(b) order(first(b):first(b)+counts(b)-1),neighbors(:),'UniformOutput',false);
        index=vertcat(pieces{:}); q=p(index,:); owned=group(index)==j;
        [sortedZ,sortZ]=sort(own(:,3)); z0=median(own(:,3)); seeds=zeros(0,4);
        quantiles=quantile(sortedZ,[.05 .25 .5 .75 .95]); span=sortedZ(end)-sortedZ(1);
        centered=sortedZ-mean(sortedZ); sd=sqrt(mean(centered.^2));
        values=[max(diff(sortedZ))/span,(quantiles(4)-quantiles(2))/span, ...
            mean(centered.^3)/max(sd^3,eps),mean(centered.^4)/max(sd^4,eps), ...
            quantiles(3)-quantiles(1),quantiles(5)-quantiles(3)];
        for c=1:numel(wholeNames), result.(wholeNames{c})(j)=values(c); end
        for w=1:size(windows,1)
            a=max(1,1+floor(windows(w,1)*(size(own,1)-1))); b=max(a,ceil(windows(w,2)*size(own,1)));
            sample=own(sortZ(a:b),:);
            if max(sample(:,3))-min(sample(:,3))<0.5, continue; end
            fit=[ones(size(sample,1),1),sample(:,3)-z0]\sample(:,1:2);
            seeds(end+1,:)=[fit(1,:) fit(2,:)]; %#ok<AGROW>
        end
        % The vertical density mode retains thin stems beneath broad crowns.
        lower=geometry.origin+([cols(j) rows(j)]-1).*geometry.cellSize;
        bins=floor((own(:,1:2)-lower)/0.1); [~,~,bin]=unique(bins,'rows');
        binCount=accumarray(bin,1); [~,bestBin]=max(binCount);
        seeds(end+1,:)=[mean(own(bin==bestBin,1:2),1) 0 0]; %#ok<AGROW>
        bestScore=0; best=[];
        for s=1:size(seeds,1)
            center=seeds(s,1:2); slope=seeds(s,3:4);
            if norm(slope)>tand(settings.maximumTiltDegrees), continue; end
            for iteration=1:2
                distance=vecnorm(own(:,1:2)-center-(own(:,3)-z0).*slope,2,2);
                inlier=distance<=settings.radius;
                if nnz(inlier)<6 || max(own(inlier,3))-min(own(inlier,3))<0.5, break; end
                fit=[ones(nnz(inlier),1),own(inlier,3)-z0]\own(inlier,1:2);
                if norm(fit(2,:))>tand(settings.maximumTiltDegrees), break; end
                center=fit(1,:); slope=fit(2,:);
            end
            distance=vecnorm(q(:,1:2)-center-(q(:,3)-z0).*slope,2,2);
            core=find(distance<=settings.radius); [z,sortCore]=sort(q(core,3)); core=core(sortCore);
            if settings.localSupport && ~isempty(z)
                envelopeCount=heightWindowCount(q(distance<=settings.envelopeRadius,3),z,settings.localHalfHeight);
                contextCount=heightWindowCount(q(distance<=settings.outerRadius,3),z,settings.localHalfHeight);
                supported=envelopeCount>=settings.localMinimumCount & envelopeCount./max(contextCount,1)>=settings.localMinimumFraction;
                core=core(supported); z=z(supported);
            end
            if numel(z)<8, continue; end
            allowedGap=settings.gapBase+settings.gapRangeScale*norm(center);
            cuts=[0;find(diff(z)>allowedGap);numel(z)];
            for c=1:numel(cuts)-1
                part=core(cuts(c)+1:cuts(c+1)); zz=q(part,3); ownPart=part(owned(part));
                if numel(part)<8 || numel(ownPart)<4, continue; end
                height=max(zz)-min(zz); ownHeight=max(q(ownPart,3))-min(q(ownPart,3));
                if height<0.6 || ownHeight<0.3, continue; end
                within=q(:,3)>=min(zz) & q(:,3)<=max(zz);
                isolation=numel(part)/max(1,nnz(within & distance<=settings.outerRadius));
                fraction=numel(ownPart)/counts(j); radial=sqrt(mean(distance(part).^2));
                robust=quantile(zz,[.05 .95]); gap=max(diff(zz));
                envelope=within & distance<=settings.envelopeRadius;
                envelopeIsolation=nnz(envelope)/max(1,nnz(within & distance<=settings.outerRadius));
                quartiles=quantile(zz,[.25 .5 .75]); interquartileRatio=(quartiles(3)-quartiles(1))/height;
                residual=q(part,1:2)-center-(zz-z0).*slope; eigenvalues=eig(cov(residual,1));
                residualLinearity=(max(eigenvalues)-min(eigenvalues))/max(sum(eigenvalues),eps);
                envelopeFraction=nnz(envelope & owned)/max(1,nnz(within & owned));
                below=within & q(:,3)<=quartiles(2); above=within & q(:,3)>quartiles(2);
                lowerIsolation=nnz(envelope & below)/max(1,nnz(below & distance<=settings.outerRadius));
                upperIsolation=nnz(envelope & above)/max(1,nnz(above & distance<=settings.outerRadius));
                score=height*(0.5+envelopeIsolation)*fraction^0.25/(1+radial/0.1);
                if score>bestScore
                    bestScore=score;
                    best=[height,diff(robust),numel(part),numel(ownPart),fraction,isolation,radial, ...
                        atand(norm(slope)),gap,gap/max(height,eps),ownHeight,score,center,z0,slope,min(zz),max(zz),envelopeIsolation,interquartileRatio,residualLinearity,envelopeFraction,median(diff(zz)),lowerIsolation,upperIsolation];
                end
            end
        end
        if ~isempty(best)
            for c=1:numel(names), result.(names{c})(j)=best(c); end
        end
    end
end

function count=heightWindowCount(z,queries,halfHeight)
% Count a continuous height neighborhood using the empirical height CDF.
    if isempty(z), count=zeros(size(queries)); return; end
    [uniqueZ,~,group]=unique(z); cumulative=cumsum(accumarray(group,1));
    if isscalar(uniqueZ)
        count=double(abs(queries-uniqueZ)<=halfHeight)*numel(z); return;
    end
    lower=queries-halfHeight; upper=queries+halfHeight;
    hi=interp1(uniqueZ,cumulative,upper,'previous','extrap'); hi(upper<uniqueZ(1))=0;
    % Include values on the lower boundary in the window.
    lower=lower-4*eps(max(1,abs(lower)));
    lo=interp1(uniqueZ,cumulative,lower,'previous','extrap'); lo(lower<uniqueZ(1))=0;
    count=hi-lo;
end
