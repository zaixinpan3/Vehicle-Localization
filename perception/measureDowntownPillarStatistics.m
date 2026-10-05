function [features,names,ids]=measureDowntownPillarStatistics(ground,offGround,semantic)
% measureDowntownPillarStatistics: Describe every occupied branch pillar.
% Inputs contain aggregate distributions and their raster context only.
% Raw points, reference masks, frame IDs and absolute XY coordinates are not
% features. IDs address the branch raster, before shared-lattice publication.
    semantic=string(semantic);
    assert(isscalar(semantic) && any(semantic==["curb","pole","facade","trafficSign"]), ...
        'perception:InvalidSemanticClass','Unknown Downtown semantic channel.');
    if semantic=="curb"
        ground.curbCellMask=ground.stats.countMap>0;
        [features,names,ids]=measureSemanticPillarFeatures(ground,offGround,'curb');
        [context,contextNames]=curbOffGroundContext(ground,offGround,ids);
        features=[features,context];names=[names,contextNames];
        [features,names]=appendShape(features,names,ids,ground);
        if isfield(ground,'populationStatistics')
            [population,columns]=measurePillarMomentContext(ground.populationStatistics,size(ground.curbCellMask),ids);
            features=[features,population];names=[names,"population_"+columns];
        end
        return;
    end
    % The common structural descriptor requires no Hough line or point fit.
    % Reuse its moment/bound/radiometric features for each structural class.
    offGround.poleCellMask=offGround.columnMaps.occupiedMask;
    [features,names,ids]=measureSemanticPillarFeatures(ground,offGround,'pole');
    maps=offGround.columnMaps;
    names=[names,"pillarPointScore","pillarLineScore"];
    features=[features,reshape(double(maps.pointScore(ids)),[],1),reshape(double(maps.lineScore(ids)),[],1)];
    [context,contextNames]=measureDowntownDistributionContext(maps,ids);
    features=[features,context];names=[names,contextNames];
    [features,names]=appendShape(features,names,ids,maps);
    if isfield(maps,'distributionShape')
        [population,columns]=measurePillarMomentContext(maps.statistics,maps.mapSize,ids);
        features=[features,population];names=[names,"population_"+columns];
    end
    features(~isfinite(features))=0;
end

function [features,names]=appendShape(features,names,ids,branch)
    if ~isfield(branch,'distributionShape'),return;end
    summary=branch.distributionShape;
    [found,rows]=ismember(double(ids),double(summary.pillarIndices));assert(all(found));
    features=[features,summary.values(rows,:)];names=[names,"distribution_"+summary.names];
end

function [features,names]=curbOffGroundContext(ground,offGround,ids)
% Describe elevated context from complete off-ground pillar distributions.
% Neighborhood moments are count-weighted population moments. Heights use
% the evaluated ground cell as reference. Two-cell padding preserves context
% when ground and off-ground rasters were compacted to different bounds.
    maps=offGround.columnMaps;stats=maps.statistics;
    spacing=double(ground.cellSize(:).');
    assert(all(abs(spacing-[maps.dx,maps.dy])<1e-10), ...
        'perception:ContextLatticeMismatch','Branch statistics must share XY spacing.');
    offset=(double(ground.cellOrigin)-double(maps.origin))./spacing;
    assert(all(abs(offset-round(offset))<1e-8), ...
        'perception:ContextLatticeMismatch','Branch statistics must share XY lattice phase.');
    dims=double(maps.mapSize);padded=dims+4;
    [row,col]=ind2sub(dims,double(stats.pillarIndices));
    at=sub2ind(padded,row+2,col+2);
    count=zeros(padded);sumZ=count;secondZ=count;span=count;
    minZ=inf(padded);maxZ=-inf(padded);
    count(at)=double(stats.count);
    sumZ(at)=stats.count.*stats.meanXYZ(:,3);
    secondZ(at)=stats.count.*(stats.covarianceXYZ(:,6)+stats.meanXYZ(:,3).^2);
    minZ(at)=stats.minimumXYZ(:,3);maxZ(at)=stats.maximumXYZ(:,3);
    span(at)=stats.maximumXYZ(:,3)-stats.minimumXYZ(:,3);
    occupied=count>0;
    [groundRow,groundCol]=ind2sub(size(ground.curbCellMask),ids);
    contextRow=groundRow+round(offset(2))+2;
    contextCol=groundCol+round(offset(1))+2;
    valid=contextRow>=1 & contextRow<=padded(1) & contextCol>=1 & contextCol<=padded(2);
    contextIds=sub2ind(padded,contextRow(valid),contextCol(valid));
    groundZ=double(ground.stats.heightMap(ids));
    features=zeros(numel(ids),27);names=strings(1,27);
    columns=["countSum","occupiedPillars","meanCount","minimumAboveGround", ...
        "meanAboveGround","maximumAboveGround","heightVariance","meanPillarHeight","maximumPillarHeight"];
    for radius=0:2
        kernel=ones(2*radius+1);
        n=conv2(count,kernel,'same');cells=conv2(double(occupied),kernel,'same');
        meanZ=conv2(sumZ,kernel,'same')./max(n,1);
        variance=max(0,conv2(secondZ,kernel,'same')./max(n,1)-meanZ.^2);
        minimum=imerode(minZ,kernel);maximum=imdilate(maxZ,kernel);
        meanHeight=conv2(span,kernel,'same')./max(cells,1);
        maxHeight=imdilate(span,kernel);
        values=zeros(numel(ids),9);
        values(valid,:)=[n(contextIds),cells(contextIds),n(contextIds)./max(cells(contextIds),1), ...
            minimum(contextIds)-groundZ(valid),meanZ(contextIds)-groundZ(valid), ...
            maximum(contextIds)-groundZ(valid),variance(contextIds),meanHeight(contextIds),maxHeight(contextIds)];
        values(values(:,1)==0,:)=0;
        indices=radius*9+(1:9);
        features(:,indices)=values;
        names(indices)="offGround_r"+radius+"_"+columns;
    end
    features(~isfinite(features))=0;
    features=floor(features*1e8+.5)/1e8;
end
