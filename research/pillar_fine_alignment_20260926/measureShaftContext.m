function context=measureShaftContext(points,pillarIds,geometry,modes,evaluate)
% measureShaftContext: Continuous context around an already proposed shaft.
% Statistics use raw returns in existing pillars, at the proposed shaft's
% heights. There is no auxiliary XY lattice or vertical occupancy grid.
% These are diagnostic features, not acceptance rules. Testing only a winning
% shaft afterward can miss another valid mode; production validation should
% eventually evaluate context before choosing the winning hypothesis.
    ids=double(modes.pillarIndices(:));n=numel(ids);
    if nargin<5 || isempty(evaluate),evaluate=modes.found;end
    assert(numel(evaluate)==n,'One evaluation flag is required per mode.');
    names={'supportCount','count25','count40','count75','coreFraction25_40', ...
        'coreFraction25_75','supportFraction25','supportFraction75', ...
        'densityContrast25_40','densityContrast25_75','supportDensityContrast75', ...
        'growthDimension25_75','wideMaximumStd25','wideMinimumStd25','wideAspect25', ...
        'balancedMaximumStd25','balancedAspect25','minimumHalfFraction25_75', ...
        'minimumQuarterFraction25_75','minimumQuarterSupport', ...
        'maximumSupportGap','supportCoverage20','supportedFraction25_75', ...
        'halfAxisDisplacement','halfSlopeDifference','halfMaximumRms','tiltDegrees', ...
        'ownerSupportCount','ownerSupportHeight','ownerRadialRms','ownerRadiusFraction', ...
        'ownerCenterOffset','ownerSupportFraction25','ownerCoverage20','ownerMaximumGap', ...
        'broadPeak25','broadPeak50','broadPeakExcess25','minimumHalfBroadPeak25','minimumHalfBroadPeak50', ...
        'wideMaximumStd40','wideMinimumStd40','wideAspect40','balancedMaximumStd40','balancedAspect40', ...
        'ordinaryFitTilt25','ordinaryFitRms25','balancedFitTilt25','balancedFitRms25', ...
        'denseSupportHeight3in50','longestDenseSupport3in50','denseCoreHeight3in50', ...
        'qualifiedContextHeight3in50','longestQualifiedContext3in50', ...
        'qualifiedContextMeanRatio','qualifiedContextMassRatio'};
    context=struct('pillarIndices',ids,'evaluated',false(n,1));
    for name=names,context.(name{1})=nan(n,1);end
    context.quarterFraction25_75=nan(n,4);context.quarterSupport=nan(n,4);
    p=double(points);[occupied,~,group]=unique(double(pillarIds(:)));
    assert(isequal(occupied,ids),'Point owners and mode ordering must agree.');
    if isempty(p),return;end
    counts=accumarray(group,1);[~,order]=sort(group);first=cumsum([1;counts(1:end-1)]);
    lookup=zeros(geometry.mapSize);lookup(ids)=1:n;
    selected=find(logical(evaluate(:)) & modes.found(:));
    for j=selected.'
        lo=modes.minimumZ(j);hi=modes.maximumZ(j);span=hi-lo;
        if ~isfinite(span) || span<=0 || any(~isfinite(modes.axisXY(j,:))),continue;end
        ends=modes.axisXY(j,:)+([lo;hi]-modes.axisZ(j)).*modes.slopeXY(j,:);
        lower=floor((min(ends,[],1)-.75-geometry.origin)./geometry.cellSize)+1;
        upper=floor((max(ends,[],1)+.75-geometry.origin)./geometry.cellSize)+1;
        rr=max(1,lower(2)):min(geometry.mapSize(1),upper(2));
        cc=max(1,lower(1)):min(geometry.mapSize(2),upper(1));
        neighbors=lookup(rr,cc);neighbors=neighbors(neighbors>0);
        pieces=arrayfun(@(b)order(first(b):first(b)+counts(b)-1),neighbors(:),'UniformOutput',false);
        index=vertcat(pieces{:});q=p(index,:);owned=group(index)==j;
        atHeight=q(:,3)>=lo & q(:,3)<=hi;q=q(atHeight,:);owned=owned(atHeight);
        residual=q(:,1:2)-modes.axisXY(j,:)-(q(:,3)-modes.axisZ(j)).*modes.slopeXY(j,:);
        radius=vecnorm(residual,2,2);inside=radius<=.75;
        q=q(inside,:);residual=residual(inside,:);radius=radius(inside);owned=owned(inside);
        support=radius<=modes.radius(j);r25=radius<=.25;r40=radius<=.40;
        ns=nnz(support);n25=nnz(r25);n40=nnz(r40);n75=numel(radius);
        context.evaluated(j)=true;context.supportCount(j)=ns;
        context.count25(j)=n25;context.count40(j)=n40;context.count75(j)=n75;
        context.coreFraction25_40(j)=n25/max(n40,1);
        context.coreFraction25_75(j)=n25/max(n75,1);
        context.supportFraction25(j)=ns/max(n25,1);
        context.supportFraction75(j)=ns/max(n75,1);
        context.densityContrast25_40(j)=n25/max(n40-n25,1)*((.40/.25)^2-1);
        context.densityContrast25_75(j)=n25/max(n75-n25,1)*8;
        context.supportDensityContrast75(j)=ns/max(n75-ns,1)*((.75/modes.radius(j))^2-1);
        context.growthDimension25_75(j)=log(max(n75,1)/max(n25,1))/log(3);
        [context.wideMaximumStd25(j),context.wideMinimumStd25(j),context.wideAspect25(j)]= ...
            transverseSpread(residual(r25,:),ones(n25,1));
        [context.balancedMaximumStd25(j),~,context.balancedAspect25(j)]= ...
            transverseSpread(residual(r25,:),heightWeights(q(r25,3)));
        [context.wideMaximumStd40(j),context.wideMinimumStd40(j),context.wideAspect40(j)]= ...
            transverseSpread(residual(r40,:),ones(n40,1));
        [context.balancedMaximumStd40(j),~,context.balancedAspect40(j)]= ...
            transverseSpread(residual(r40,:),heightWeights(q(r40,3)));
        middle=(lo+hi)/2;
        [independent,context.balancedFitRms25(j)]=fitHalf(q(r25,:),middle);
        context.balancedFitTilt25(j)=atand(norm(independent(4:5)));
        [context.ordinaryFitTilt25(j),context.ordinaryFitRms25(j)]=ordinaryFit(q(r25,:),middle);
        probe25=broadProbes(residual,.25,.25);probe50=broadProbes(residual,.25,.50);
        side25=max(sum(probe25,1));side50=max(sum(probe50,1));
        context.broadPeak25(j)=n25/max(side25,1);
        context.broadPeak50(j)=n25/max(side50,1);
        context.broadPeakExcess25(j)=n25-side25;
        [context.denseSupportHeight3in50(j),context.longestDenseSupport3in50(j), ...
            context.denseCoreHeight3in50(j),context.qualifiedContextHeight3in50(j), ...
            context.longestQualifiedContext3in50(j),context.qualifiedContextMeanRatio(j), ...
            context.qualifiedContextMassRatio(j)]=continuousDensitySupport(q(:,3),support,r25,lo,hi);
        context.tiltDegrees(j)=atand(norm(modes.slopeXY(j,:)));
        ownerSupport=owned & support;ownerZ=sort(q(ownerSupport,3));
        context.ownerSupportCount(j)=nnz(ownerSupport);
        context.ownerSupportFraction25(j)=nnz(ownerSupport)/max(nnz(owned & r25),1);
        if ~isempty(ownerZ)
            context.ownerSupportHeight(j)=ownerZ(end)-ownerZ(1);
            context.ownerRadialRms(j)=sqrt(mean(radius(ownerSupport).^2));
            context.ownerRadiusFraction(j)=context.ownerRadialRms(j)/modes.radius(j);
            context.ownerCenterOffset(j)=norm(mean(residual(ownerSupport,:),1));
            ownerCoverage=supportWindows(ownerZ,zeros(0,1),lo,hi,.10);
            context.ownerCoverage20(j)=ownerCoverage/span;
        end
        if numel(ownerZ)>=2,context.ownerMaximumGap(j)=max(diff(ownerZ));end

        % Relative height windows move with the proposed interval; they are
        % context comparisons, not a fixed vertical occupancy representation.
        halfFraction=zeros(1,2);halfFits=nan(2,5);halfRms=nan(2,1);
        halfPeak25=zeros(1,2);halfPeak50=zeros(1,2);
        for half=1:2
            if half==1,inHalf=q(:,3)<=middle;else,inHalf=q(:,3)>middle;end
            halfFraction(half)=nnz(r25 & inHalf)/max(nnz(inHalf),1);
            halfPeak25(half)=nnz(r25 & inHalf)/max(max(sum(probe25(inHalf,:),1)),1);
            halfPeak50(half)=nnz(r25 & inHalf)/max(max(sum(probe50(inHalf,:),1)),1);
            [halfFits(half,:),halfRms(half)]=fitHalf(q(support & inHalf,:),middle);
        end
        context.minimumHalfFraction25_75(j)=min(halfFraction);
        context.minimumHalfBroadPeak25(j)=min(halfPeak25);
        context.minimumHalfBroadPeak50(j)=min(halfPeak50);
        context.halfAxisDisplacement(j)=norm(halfFits(1,1:2)-halfFits(2,1:2));
        context.halfSlopeDifference(j)=norm(halfFits(1,4:5)-halfFits(2,4:5));
        context.halfMaximumRms(j)=max(halfRms);
        edges=lo+(0:4)*span/4;
        for quarter=1:4
            if quarter==1,lowerIncluded=q(:,3)>=edges(quarter);else,lowerIncluded=q(:,3)>edges(quarter);end
            inQuarter=lowerIncluded & q(:,3)<=edges(quarter+1);
            context.quarterFraction25_75(j,quarter)=nnz(r25 & inQuarter)/max(nnz(inQuarter),1);
            context.quarterSupport(j,quarter)=nnz(support & inQuarter);
        end
        context.minimumQuarterFraction25_75(j)=min(context.quarterFraction25_75(j,:));
        context.minimumQuarterSupport(j)=min(context.quarterSupport(j,:));
        z=sort(q(support,3));
        if numel(z)<2,continue;end
        context.maximumSupportGap(j)=max(diff(z));
        [coverage,atSupportedHeights]=supportWindows(z,q(:,3),lo,hi,.10);
        context.supportCoverage20(j)=coverage/span;
        context.supportedFraction25_75(j)=nnz(r25 & atSupportedHeights)/max(nnz(atSupportedHeights),1);
    end
end

function probes=broadProbes(residual,radius,offset)
% Translated disks expose a ridge that a small cropped cylinder can hide.
    directions=[1 0;0 1;1 1;1 -1;-1 0;0 -1;-1 -1;-1 1];
    directions=directions./vecnorm(directions,2,2);probes=false(size(residual,1),8);
    for k=1:8,probes(:,k)=sum((residual-offset*directions(k,:)).^2,2)<=radius^2;end
end

function [tilt,rms]=ordinaryFit(points,atZ)
    tilt=NaN;rms=NaN;
    if size(points,1)<3 || max(points(:,3))-min(points(:,3))<.1,return;end
    design=[ones(size(points,1),1),points(:,3)-atZ];coefficients=design\points(:,1:2);
    residual=points(:,1:2)-design*coefficients;
    tilt=atand(norm(coefficients(2,:)));rms=sqrt(mean(sum(residual.^2,2)));
end

function [supportHeight,longestSupport,coreHeight,qualifiedHeight,longestQualified,meanRatio,massRatio]= ...
        continuousDensitySupport(z,support,core,lo,hi)
% Integrate moving-window counts exactly between observed event endpoints.
% Each return contributes over z +/- 0.25 m. There is no fixed height phase.
    supportHeight=0;longestSupport=0;coreHeight=0;qualifiedHeight=0;
    longestQualified=0;meanRatio=NaN;massRatio=NaN;
    if isempty(z),return;end
    starts=max(lo,z-.25);ends=min(hi,z+.25);n=numel(z);
    [positions,~,event]=unique([starts;ends;lo;hi]);
    delta=[double(support),double(core),ones(n,1)];delta=[delta;-delta;zeros(2,3)];
    counts=zeros(numel(positions),3);
    for k=1:3,counts(:,k)=cumsum(accumarray(event,delta(:,k),[numel(positions) 1]));end
    widths=diff(positions);counts=counts(1:end-1,:);
    dense=counts(:,1)>=3;coreDense=counts(:,2)>=3;
    ratio=counts(:,2)./max(counts(:,3),1);qualified=coreDense & ratio>.60;
    supportHeight=sum(widths(dense));longestSupport=longestInterval(widths,dense);
    coreHeight=sum(widths(coreDense));qualifiedHeight=sum(widths(qualified));
    longestQualified=longestInterval(widths,qualified);
    if qualifiedHeight>0
        meanRatio=sum(widths(qualified).*ratio(qualified))/qualifiedHeight;
        massRatio=sum(widths(qualified).*counts(qualified,2))/sum(widths(qualified).*counts(qualified,3));
    end
end

function length=longestInterval(widths,selected)
    length=0;if ~any(selected),return;end
    edges=diff([false;selected;false]);first=find(edges==1);last=find(edges==-1);
    accumulated=[0;cumsum(widths)];length=max(accumulated(last)-accumulated(first));
end

function [maximumStd,minimumStd,aspect]=transverseSpread(residual,weights)
    maximumStd=NaN;minimumStd=NaN;aspect=NaN;
    if size(residual,1)<2 || ~any(weights>0),return;end
    weights=weights/sum(weights);center=sum(weights.*residual,1);
    centered=residual-center;matrix=centered.'*(weights.*centered);
    spread=max(0,sort(eig((matrix+matrix.')/2)));
    minimumStd=sqrt(spread(1));maximumStd=sqrt(spread(2));
    aspect=maximumStd/max(minimumStd,eps);
end

function weights=heightWeights(z)
% Equal physical height measure prevents one dense height from dominating.
    weights=zeros(size(z));if isempty(z),return;end
    [heights,~,group]=unique(z);counts=accumarray(group,1);
    if numel(heights)==1,weights(:)=1/numel(z);return;end
    gap=diff(heights);width=([gap;0]+[0;gap])/2;
    weights=width(group)./counts(group);weights=weights/sum(weights);
end

function [axis,rms]=fitHalf(points,atZ)
    axis=nan(1,5);rms=NaN;
    if size(points,1)<3 || max(points(:,3))-min(points(:,3))<.1,return;end
    weights=heightWeights(points(:,3));center=sum(weights.*points,1);dz=points(:,3)-center(3);
    slope=sum(weights.*dz.*(points(:,1:2)-center(1:2)),1)/max(sum(weights.*dz.^2),eps);
    xy=center(1:2)+(atZ-center(3))*slope;axis=[xy atZ slope];
    residual=points(:,1:2)-xy-(points(:,3)-atZ).*slope;
    rms=sqrt(sum(weights.*sum(residual.^2,2)));
end

function [coverage,selected]=supportWindows(z,query,lo,hi,halfWidth)
% Merge metric neighborhoods of observed returns; no vertical grid is used.
    starts=max(lo,z-halfWidth);ends=min(hi,z+halfWidth);
    intervals=zeros(numel(z),2);count=1;intervals(1,:)=[starts(1) ends(1)];
    for k=2:numel(z)
        if starts(k)<=intervals(count,2)
            intervals(count,2)=max(intervals(count,2),ends(k));
        else
            count=count+1;intervals(count,:)=[starts(k) ends(k)];
        end
    end
    intervals=intervals(1:count,:);coverage=sum(intervals(:,2)-intervals(:,1));
    selected=false(size(query));
    for k=1:count,selected=selected | query>=intervals(k,1) & query<=intervals(k,2);end
end
