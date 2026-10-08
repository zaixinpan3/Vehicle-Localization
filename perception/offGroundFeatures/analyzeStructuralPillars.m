function result=analyzeStructuralPillars(pillars,cfg,cloudCfg)
% analyzeStructuralPillars: Classify whole pillars using XYZ distributions.
% No vertical index, occupancy sequence or local voxel grid is constructed.
% The coarse pole detector measures supporting subsets, while all off-ground
% returns still contribute to each pillar's output moments.
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
    % Pole evidence: either the legacy density core or a supported shaft.
    maps.coreFraction=zeros(mapSize); maps.coreHeight=zeros(mapSize); maps.coreIsolation=zeros(mapSize);
    maps.corePeakX=nan(mapSize); maps.corePeakY=nan(mapSize);
    maps.corePointCount=zeros(mapSize);
    validatedMode=isfield(cfg.pole,'detector') && cfg.pole.detector=="validatedShaft";
    jointMode=validatedMode && isfield(cfg.pole,'distributionValidation') && cfg.pole.distributionValidation.enabled;
    shaftMode=isfield(cfg.pole,'detector') && any(cfg.pole.detector==["shaft","validatedShaft"]);
    subsetMode=isfield(cfg.pole,'detector') && any(cfg.pole.detector==["subset","shaft","validatedShaft"]);
    if any(cloudCfg.semanticNames=="pole") && subsetMode
        if shaftMode
            polePointScore=maps.pointScore;poleLineScore=maps.lineScore;
            if jointMode
                [polePointScore,poleLineScore]=originalLatticeShape(pillars,maps,cfg);
            end
            shaftCfg=cfg.pole.shaft;shaftCfg.useNativeKernels=useNative;
            shaftCfg.columnMinimumScores=min(shaftCfg.strongScore, ...
                shaftCfg.minimumShapeProduct./max(double(polePointScore(ids)),eps));
            maps.poleModes=findPillarShaftModes(pillars.points,pillars.pointPillarLinIdx,geometry,shaftCfg);
            [maps.poleSubset,maps.poleShaftGroups]=assignPillarShaftSupport( ...
                pillars.points,pillars.pointPillarLinIdx,geometry,maps.poleModes,shaftCfg);
            if validatedMode
                structuralMask=true(size(pillars.points,1),1);
                if isfield(pillars.pointAttributes,'intensity')
                    intensity=pillars.pointAttributes.intensity;
                    structuralMask=~(isfinite(intensity) & intensity>cfg.trafficSignIntensityThreshold);
                end
                % Reuse the inexpensive legacy statistic gates only to limit
                % extra proposals. No legacy label bypasses validation.
                tall=stats.count>=cfg.pole.minimumPoints & stats.maximumXYZ(:,3)-stats.minimumXYZ(:,3)>=cfg.pole.minimumHeight;
                [maps.coreFraction(ids),maps.coreHeight(ids),maps.coreIsolation(ids),peak,maps.corePointCount(ids)]=computePillarDensityCore( ...
                    pillars.points,pillars.pointPillarLinIdx,geometry,cfg.pole.densityPeakBinMeters,cfg.pole.coreRadius,cfg.pole.isolationRadius,tall);
                maps.corePeakX(ids)=peak(:,1);maps.corePeakY(ids)=peak(:,2);
                proposalCfg=cfg.pole;proposalCfg.detector="pillar";
                proposalCandidates=detectPolePillars(maps,false(mapSize),proposalCfg);
                maps.poleValidationProposals=completePoleProposals(maps.poleSubset,stats,cfg.pole,proposalCandidates.candidateMask(ids));
                if jointMode
                    maps.poleValidation=classifyPillarPoleSupport(pillars.points,pillars.pointPillarLinIdx, ...
                        geometry,maps.poleValidationProposals,cfg.pole.validation,structuralMask, ...
                        polePointScore,poleLineScore,cfg.pole.distributionValidation);
                else
                    maps.poleValidation=validatePillarPoleSupport(pillars.points,pillars.pointPillarLinIdx, ...
                        geometry,maps.poleValidationProposals,cfg.pole.validation,structuralMask,double(maps.pointScore(ids)));
                end
            end
        else
            subsetCfg=cfg.pole.subset; subsetCfg.useNativeKernels=useNative;
            maps.poleSubset=findPillarPoleSubsets(pillars.points,pillars.pointPillarLinIdx,geometry,subsetCfg);
        end
        maps.coreHeight(ids)=maps.poleSubset.height;
        maps.corePointCount(ids)=maps.poleSubset.supportCount;
        maps.corePeakX(ids)=maps.poleSubset.axisXY(:,1); maps.corePeakY(ids)=maps.poleSubset.axisXY(:,2);
    end
    if any(cloudCfg.semanticNames=="pole") && (~subsetMode || (shaftMode && ~validatedMode))
        % The core is measured only where the cheap whole-pillar gates can pass.
        tall=stats.count>=cfg.pole.minimumPoints & stats.maximumXYZ(:,3)-stats.minimumXYZ(:,3)>=cfg.pole.minimumHeight;
        [maps.coreFraction(ids),maps.coreHeight(ids),maps.coreIsolation(ids),peak,maps.corePointCount(ids)]=computePillarDensityCore( ...
            pillars.points,pillars.pointPillarLinIdx,geometry,cfg.pole.densityPeakBinMeters,cfg.pole.coreRadius,cfg.pole.isolationRadius,tall);
        maps.corePeakX(ids)=peak(:,1); maps.corePeakY(ids)=peak(:,2);
    end
    facade=struct('mask',false(mapSize));
    if any(cloudCfg.semanticNames=="facade")
        facadeCfg=cfg;
        % Facade voting uses return support along a line; pole contrast uses
        % vertical spread. Both maps remain one statistic per whole pillar.
        facadeMaps=maps;
        facadeMaps.supportEvidence=single(maps.pillarCounts);
        if isfield(cfg,'facadeContinuousSupportMinimumPoints')
            minimumSpan=1;
            if isfield(cfg,'facadeContinuousSupportMinimumSpan'),minimumSpan=cfg.facadeContinuousSupportMinimumSpan;end
            facadeMaps.supportEvidence=continuousPillarHeightSupport(pillars,mapSize,cfg.facadeContinuousSupportMinimumPoints,minimumSpan);
            facadeMaps.occupiedMask=facadeMaps.supportEvidence>0;
        end
        [~,facadeMaps.lineScore,facadeMaps.normalOrientation]=buildPillarShapeScores( ...
            facadeMaps.supportEvidence,facadeMaps.occupiedMask,cfg,maps.dx,maps.dy);
        elevated=false(mapSize);
        if isfield(cfg,'facadeMaximumBaseHeight') && isfield(pillars,'groundHeightMap')
            % Facades reach down to the ground; tree canopies and other
            % overhanging structure start well above it. A pillar whose lowest
            % nonground return lies more than facadeMaximumBaseHeight above
            % the local ground (perceiveFrame's ground-height map) gives no
            % facade evidence. Pillars with unknown ground are kept.
            base=nan(mapSize);base(ids)=stats.minimumXYZ(:,3);
            elevated=base-double(pillars.groundHeightMap)>cfg.facadeMaximumBaseHeight;
            facadeMaps.supportEvidence(elevated)=0;
            facadeMaps.occupiedMask=facadeMaps.occupiedMask & ~elevated;
        end
        facade=extractFacadeFeatures(facadeMaps,facadeCfg);
        facade=expandFacadePillarSupport(facade,maps,cfg);
        if any(elevated(:) & facade.mask(:))
            facade.mask(elevated)=false;facade.lineMap(elevated)=0;
            facade.pillarLinIdx=find(facade.mask);
            facade.pillarLineIdx=double(facade.lineMap(facade.pillarLinIdx));
        end
        maps.facadeLineScore=facadeMaps.lineScore;
    end
    poleMask=false(mapSize); candidates=struct();
    if any(cloudCfg.semanticNames=="pole")
        candidates=detectPolePillars(maps,facade.mask,cfg.pole);
        poleMask=candidates.candidateMask;
    end
    signMask=false(mapSize); signEvidence=zeros(mapSize);
    maps.trafficSignMoments=maps.moments;
    if any(cloudCfg.semanticNames=="trafficSign") && isfield(stats,'intensity')
        maximum=stats.intensity.maximum;
        selected=isfinite(maximum) & maximum>cfg.trafficSignIntensityThreshold;
        signMask(ids(selected))=true;
        signEvidence(ids(selected))=max(0,1-cfg.trafficSignIntensityThreshold./maximum(selected));
        if any(selected)
            % The sign component keeps only the sign-like part of the pillar:
            % intensity-responsibility moments, whose count is the soft sign mass.
            softness=structuralPillarConfig(maps.dx).trafficSignIntensitySoftness;
            if isfield(cfg,'trafficSignIntensitySoftness'),softness=cfg.trafficSignIntensitySoftness;end
            signStats=measureSignResponsibilityMoments(pillars.points,pillars.pointPillarLinIdx, ...
                double(pillars.pointAttributes.intensity),ids(selected),cfg.trafficSignIntensityThreshold,softness);
            weighted=projectStatistics(signStats,prod(mapSize),cloudCfg);rows=double(signStats.pillarIndices);
            for field=["count","mean","covariance","meanZ","heightCovariance"]
                maps.trafficSignMoments.(field)(rows,:)=weighted.(field)(rows,:);
            end
        end
    end
    maps.trafficSignCellMask=signMask;
    maps.trafficSignEvidence=signEvidence;
    floorProbability=cloudCfg.minimumSemanticProbability;
    poleEvidence=double(maps.pointScore).*(1-double(maps.lineScore));
    if validatedMode && isfield(maps,'poleValidation') && cfg.pole.probabilityEvidence=="validatedShaft"
        poleEvidence=zeros(mapSize);poleEvidence(ids)=maps.poleValidation.score;
    elseif subsetMode && isfield(maps,'poleSubset') && any(cfg.pole.probabilityEvidence==["subset","shaft"])
        poleEvidence=zeros(mapSize); poleEvidence(ids)=maps.poleSubset.score;
        if shaftMode && isfield(candidates,'legacyCandidateMask') && any(candidates.legacyCandidateMask(:))
            peak=[maps.corePeakX(ids),maps.corePeakY(ids)];
            legacyEvidence=scorePolePillarDistributions(pillars.points,pillars.pointPillarLinIdx, ...
                geometry,peak,candidates.legacyCandidateMask(ids),cfg.pole.distribution);
            poleEvidence(ids)=max(poleEvidence(ids),legacyEvidence.score);
        end
    elseif isfield(cfg.pole,'probabilityEvidence') && cfg.pole.probabilityEvidence=="distribution" && any(poleMask(:))
        peak=[maps.corePeakX(ids),maps.corePeakY(ids)];
        maps.poleDistribution=scorePolePillarDistributions(pillars.points,pillars.pointPillarLinIdx, ...
            geometry,peak,poleMask(ids),cfg.pole.distribution);
        poleEvidence=zeros(mapSize); poleEvidence(ids)=maps.poleDistribution.score;
    end
    maps.poleEvidence=single(poleEvidence);
    facadeEvidence=zeros(mapSize);
    if isfield(maps,'facadeLineScore'), facadeEvidence=double(maps.facadeLineScore); end
    result=struct('columnMaps',maps,'facade',facade,'facadeCellMask',facade.mask, ...
        'facadeProbability',single(facade.mask.*(floorProbability+(1-floorProbability)*facadeEvidence)), ...
        'poleCellMask',poleMask,'poleProbability',single(poleMask.*(floorProbability+(1-floorProbability)*poleEvidence)), ...
        'trafficSignCellMask',signMask,'trafficSignProbability',single(signMask.*(floorProbability+(1-floorProbability)*signEvidence)), ...
        'candidates',candidates);
end

function [point,line]=originalLatticeShape(pillars,maps,cfg)
% Empty-margin cropping must not change the model's spatial context boundary.
% Padding restores only the original coarse raster; no finer grid is created.
    point=maps.pointScore;line=maps.lineScore;
    if ~isfield(pillars,'sourcePillarGeometry'),return;end
    original=pillars.sourcePillarGeometry;g=pillars.pillarGeometry;
    if isequal(original.origin,g.origin)&&isequal(original.mapSize,g.mapSize),return;end
    offset=round((g.origin-original.origin)./g.cellSize);
    rows=(1:g.mapSize(1))+offset(2);cols=(1:g.mapSize(2))+offset(1);
    support=zeros(original.mapSize,'single');occupied=false(original.mapSize);
    support(rows,cols)=maps.supportEvidence;occupied(rows,cols)=maps.occupiedMask;
    [fullPoint,fullLine]=buildPillarShapeScores(support,occupied,cfg,g.cellSize(1),g.cellSize(2));
    point=fullPoint(rows,cols);line=fullLine(rows,cols);
end

function modes=completePoleProposals(modes,stats,cfg,eligible)
% Whole-pillar axes rescue search omissions; they still require validation.
% This is a proposal mechanism, never a union with legacy accepted labels.
    covariance=stats.covarianceXYZ;
    slope=covariance(:,4:5)./max(covariance(:,6),eps);
    variance=max(0,covariance(:,1)+covariance(:,3)- ...
        sum(covariance(:,4:5).^2,2)./max(covariance(:,6),eps));
    selected=eligible & ~modes.found & stats.count>=cfg.minimumPoints & ...
        stats.maximumXYZ(:,3)-stats.minimumXYZ(:,3)>=cfg.minimumHeight & ...
        covariance(:,6)>=cfg.minimumHeightStd^2 & ...
        vecnorm(slope,2,2)<=tand(cfg.maximumTiltDegrees) & variance<=cfg.maximumRadialStd^2;
    modes.found(selected)=true;modes.axisXY(selected,:)=stats.meanXYZ(selected,1:2);
    modes.axisZ(selected)=stats.meanXYZ(selected,3);modes.slopeXY(selected,:)=slope(selected,:);
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
