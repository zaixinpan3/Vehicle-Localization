function result=analyzeStructuralPillars(pillars,cfg,cloudCfg)
% analyzeStructuralPillars: Classify whole pillars using XYZ distributions.
% No vertical index, occupancy sequence, local voxel grid or semantic point
% selection exists here. All off-ground returns contribute to each pillar.
    useNative=isfield(cfg,'useNativeKernels') && cfg.useNativeKernels;
    stats=aggregatePillarStatistics(pillars.points,pillars.pointPillarLinIdx, ...
        pillars.pointAttributes,useNative);
    geometry=pillars.pillarGeometry;
    mapSize=geometry.mapSize;
    ids=double(stats.pillarIndices);
    maps=struct('origin',geometry.origin,'dx',geometry.cellSize(1), ...
        'dy',geometry.cellSize(2),'mapSize',mapSize,'statistics',stats);
    maps.pillarCounts=zeros(mapSize); maps.pillarCounts(ids)=stats.count;
    maps.pillarZRange=zeros(mapSize); maps.pillarZRange(ids)=stats.maximumXYZ(:,3)-stats.minimumXYZ(:,3);
    maps.occupiedMask=maps.pillarCounts>0;
    [maps.xMap,maps.yMap]=meshgrid(single(geometry.origin(1)+((1:mapSize(2))-.5)*maps.dx), ...
        single(geometry.origin(2)+((1:mapSize(1))-.5)*maps.dy));
    maps.xMap(~maps.occupiedMask)=NaN; maps.yMap(~maps.occupiedMask)=NaN;
    % Whole-pillar height spread supplies XY shape and Hough evidence.
    maps.supportEvidence=single(maps.pillarCounts);
    maps.supportEvidence(maps.pillarCounts<3)=0;
    maps.pointScore=zeros(mapSize,'single'); maps.lineScore=maps.pointScore;
    maps.normalOrientation=nan(mapSize,'single'); maps.blobness=maps.pointScore;
    if any(ismember(cloudCfg.semanticNames,["pole","facade"]))
        [maps.pointScore,maps.lineScore,maps.normalOrientation,maps.blobness]= ...
            buildPillarShapeScores(maps.supportEvidence,maps.occupiedMask,cfg,maps.dx,maps.dy);
    end
    maps.moments=projectStatistics(stats,prod(mapSize),cloudCfg);
    facade=struct('mask',false(mapSize));
    if any(cloudCfg.semanticNames=="facade")
        facadeCfg=cfg;
        % Facade voting uses return support along a line; pole contrast uses
        % vertical spread. Both maps remain one statistic per whole pillar.
        facadeMaps=maps;
        facadeMaps.supportEvidence=single(maps.pillarCounts);
        [~,facadeMaps.lineScore,facadeMaps.normalOrientation]=buildPillarShapeScores( ...
            facadeMaps.supportEvidence,maps.occupiedMask,cfg,maps.dx,maps.dy);
        facade=extractFacadeFeatures(facadeMaps,facadeCfg);
        facade=expandFacadePillarSupport(facade,maps,cfg);
        maps.facadeLineScore=facadeMaps.lineScore;
    end
    poleMask=false(mapSize); candidates=struct();
    if any(cloudCfg.semanticNames=="pole")
        candidates=detectPolePillars(maps,facade.mask,cfg.pole);
        poleMask=candidates.candidateMask;
    end
    signMask=false(mapSize); signEvidence=zeros(mapSize);
    if any(cloudCfg.semanticNames=="trafficSign") && isfield(stats,'intensity')
        maximum=stats.intensity.maximum;
        selected=isfinite(maximum) & maximum>cfg.trafficSignIntensityThreshold;
        signMask(ids(selected))=true;
        signEvidence(ids(selected))=max(0,1-cfg.trafficSignIntensityThreshold./maximum(selected));
    end
    maps.trafficSignCellMask=signMask;
    maps.trafficSignEvidence=signEvidence;
    maps.trafficSignMoments=maps.moments;
    floorProbability=cloudCfg.minimumSemanticProbability;
    poleEvidence=double(maps.pointScore).*(1-double(maps.lineScore));
    facadeEvidence=zeros(mapSize);
    if isfield(maps,'facadeLineScore'), facadeEvidence=double(maps.facadeLineScore); end
    result=struct('columnMaps',maps,'facade',facade,'facadeCellMask',facade.mask, ...
        'facadeProbability',single(facade.mask.*(floorProbability+(1-floorProbability)*facadeEvidence)), ...
        'poleCellMask',poleMask,'poleProbability',single(poleMask.*(floorProbability+(1-floorProbability)*poleEvidence)), ...
        'trafficSignCellMask',signMask,'trafficSignProbability',single(signMask.*(floorProbability+(1-floorProbability)*signEvidence)), ...
        'candidates',candidates);
end

function moments=projectStatistics(stats,numCells,cfg)
% projectStatistics: Transform complete XYZ distributions and expose XY margins.
    ids=double(stats.pillarIndices);
    r=cfg.projectionRotation;
    mu=stats.meanXYZ*r.'+cfg.projectionTranslation;
    packed=stats.covarianceXYZ;
    projected=zeros(size(packed));
    pairs=[1 1;1 2;2 2;1 3;2 3;3 3];
    for k=1:6
        a=r(pairs(k,1),:); b=r(pairs(k,2),:);
        factors=[a(1)*b(1),a(1)*b(2)+a(2)*b(1),a(2)*b(2), ...
            a(1)*b(3)+a(3)*b(1),a(2)*b(3)+a(3)*b(2),a(3)*b(3)];
        projected(:,k)=sum(packed.*factors,2);
    end
    moments=struct('count',zeros(numCells,1),'mean',zeros(numCells,2), ...
        'covariance',zeros(numCells,3),'meanZ',zeros(numCells,1),'heightCovariance',zeros(numCells,3));
    moments.count(ids)=stats.count; moments.mean(ids,:)=mu(:,1:2);
    moments.meanZ(ids)=mu(:,3); moments.covariance(ids,:)=projected(:,1:3);
    moments.heightCovariance(ids,:)=projected(:,4:6);
end
function facade = expandFacadePillarSupport(facade,maps,cfg)
% Include the refinement halo into the coarse candidate stage so every
% point later considered offline already belongs to a published candidate.
% This expansion uses only pillar occupancy and distances between XY centers.
    if ~any(facade.mask(:)), return; end
    radius = max(0,round(cfg.facadeSupportRadiusCells));
    seedMap = facade.lineMap;
    bestDistance = inf(maps.mapSize);
    [cols,rows] = meshgrid(1:maps.mapSize(2),1:maps.mapSize(1));
    x = maps.origin(1)+(cols-0.5)*maps.dx;
    y = maps.origin(2)+(rows-0.5)*maps.dy;
    for k = 1:size(facade.detectedLines,1)
        support = imdilate(seedMap==k,true(2*radius+1)) & maps.pillarCounts>0;
        line = facade.detectedLines(k,:);
        direction = line(3:4)-line(1:2);
        distance = abs((x-line(1))*direction(2)-(y-line(2))*direction(1))/max(norm(direction),eps);
        update = support & distance<bestDistance;
        facade.lineMap(update) = uint16(k);
        bestDistance(update) = distance(update);
    end
    facade.mask = facade.lineMap>0;
    facade.pillarLinIdx = find(facade.mask);
    facade.pillarLineIdx = double(facade.lineMap(facade.pillarLinIdx));
end
