function [x,names]=measureFacadeGroupEvidence(offGround,pillars)
% measureFacadeGroupEvidence: Plane support and competing-line statistics.
% Fits use raw members of existing 0.6 m pillars. No finer spatial grid exists.
    facade=offGround.facade;maps=offGround.columnMaps;selected=find(offGround.facadeCellMask);
    names=["group_pointCount","group_planeCount","group_planeFraction","group_planeRms", ...
        "group_planeHeight","group_planeLength","group_normalVertical","group_normalAgreement", ...
        "group_planeQualified","group_orientationCos2","group_orientationSin2","group_distance", ...
        "group_continuousHeight","group_maxHeightGap","group_heightQ90Gap","group_cellCount", ...
        "group_medianCellHeight","group_verticalHeightSum","group_pointFraction","group_verticalFraction", ...
        "group_parallelDominance","group_strengthRank","group_totalLines", ...
        "owner_planeCount","owner_planeFraction","owner_planeHeight","owner_planeQ10","owner_planeQ50", ...
        "owner_planeQ90","owner_planeZStd","owner_planeAlongStd"];
    x=zeros(numel(selected),numel(names));if isempty(selected),return;end
    points=double(pillars.points);ids=double(pillars.pointPillarLinIdx);groups=double(facade.lineMap(ids));
    structural=true(size(ids));
    if isfield(pillars.pointAttributes,'intensity')
        intensity=double(pillars.pointAttributes.intensity);structural=~(isfinite(intensity)&intensity>1800);
    end
    lineCount=size(facade.detectedLines,1);summary=zeros(lineCount,18);ownerFeatures=zeros(prod(maps.mapSize),8);
    angles=zeros(lineCount,1);stats=maps.statistics;statIds=double(stats.pillarIndices);
    for k=1:lineCount
        at=find(groups==k & structural);p=points(at,:);if isempty(p),continue;end
        line=facade.detectedLines(k,:);direction=line(3:4)-line(1:2);direction=direction/max(norm(direction),eps);
        seed=[-direction(2);direction(1);0];normal=seed;center=median(p,1);
        mask=abs((p-center)*normal)<=.2;
        for iteration=1:3
            if nnz(mask)<12,break;end
            center=mean(p(mask,:),1);a=p(mask,:)-center;[basis,values]=eig((a.'*a)/size(a,1));
            [~,smallest]=min(diag(values));normal=basis(:,smallest);distance=abs((p-center)*normal);
            mid=median(distance(mask));sigma=max(.02,1.4826*median(abs(distance(mask)-mid)));
            mask=distance<=min(.2,mid+3*sigma);
        end
        distance=abs((p-center)*normal);support=p(mask,:);angles(k)=atan2(direction(2),direction(1));
        planeHeight=0;planeLength=0;rms=0;continuous=0;maxGap=0;qGap=0;
        if ~isempty(support)
            planeHeight=max(support(:,3))-min(support(:,3));along=support(:,1:2)*direction.';
            planeLength=max(along)-min(along);rms=sqrt(mean(distance(mask).^2));
            z=sort(support(:,3));gaps=diff(z);
            if ~isempty(gaps)
                maxGap=max(gaps);qGap=prctile(gaps,90);cut=[0;find(gaps>.5);numel(z)];
                continuous=max(z(cut(2:end))-z(cut(1:end-1)+1));
            end
        end
        members=facade.lineMap(statIds)==k;heights=stats.maximumXYZ(members,3)-stats.minimumXYZ(members,3);
        qualified=nnz(mask)>=12 && abs(normal(3))<=sind(20) && abs(normal.'*seed)>=cosd(20) && planeHeight>=1 && planeLength>=1;
        medianHeight=0;if ~isempty(heights),medianHeight=median(heights);end
        summary(k,:)=[size(p,1),nnz(mask),mean(mask),rms,planeHeight,planeLength,abs(normal(3)),abs(normal.'*seed), ...
            qualified,cos(2*angles(k)),sin(2*angles(k)),abs(center*normal),continuous,maxGap,qGap,nnz(members),medianHeight,sum(min(heights,8))];
        for id=unique(ids(at)).'
            own=ids(at)==id;near=own & mask;ownHeight=0;zStd=0;alongStd=0;
            if any(near)
                q=p(near,:);ownHeight=max(q(:,3))-min(q(:,3));zStd=std(q(:,3),1);alongStd=std(q(:,1:2)*direction.',1);
            end
            ownerFeatures(id,:)=[nnz(near),nnz(near)/nnz(own),ownHeight,prctile(distance(own),[10 50 90]),zStd,alongStd];
        end
    end
    strengths=summary(:,18).*summary(:,3);[~,order]=sort(strengths,'descend');rank=zeros(lineCount,1);rank(order)=1:lineCount;
    context=zeros(lineCount,5);
    for k=1:lineCount
        parallel=abs(cos(angles-angles(k)))>=cosd(10);
        context(k,:)=[summary(k,1)/max(sum(summary(:,1)),1),summary(k,18)/max(sum(summary(:,18)),eps), ...
            strengths(k)/max(max(strengths(parallel)),eps),rank(k),lineCount];
    end
    label=double(facade.lineMap(selected));x=[summary(label,:),context(label,:),ownerFeatures(selected,:)];
    x(~isfinite(x))=0;x=floor(x*1e8+.5)/1e8;
end
