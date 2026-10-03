function result=analyzeDowntownPillarStatistics(pillars,cfg,cloudCfg,groundContext)
% analyzeDowntownPillarStatistics: Form Downtown candidates from whole pillars.
% Raw returns are aggregated once into count, moments, bounds and intensity
% maxima and radiometric conditional distributions. Every subsequent decision
% uses those statistics or neighboring pillar statistics. No height slices, planes, density peaks,
% point neighborhoods or semantic point labels are evaluated here.
% Optional cfg.pillarStatistics supplies broad candidate thresholds; a
% downstream statistics model may rank or filter these candidate masks.
    useNative=isfield(cfg,'useNativeKernels') && cfg.useNativeKernels;
    if nargin<4,groundContext=struct();end
    stats=aggregatePillarStatistics(pillars.points,pillars.pointPillarLinIdx, ...
        pillars.pointAttributes,useNative);
    geometry=pillars.pillarGeometry;dims=double(geometry.mapSize);
    ids=double(stats.pillarIndices);height=stats.maximumXYZ(:,3)-stats.minimumXYZ(:,3);
    maps=struct('origin',geometry.origin,'dx',geometry.cellSize(1), ...
        'dy',geometry.cellSize(2),'mapSize',dims,'statistics',stats);
    maps.radiometry=aggregateDowntownRadiometry(pillars,cfg.trafficSignIntensityThreshold,groundContext);
    maps.pillarCounts=zeros(dims);maps.pillarCounts(ids)=stats.count;
    maps.pillarZRange=zeros(dims);maps.pillarZRange(ids)=height;
    maps.occupiedMask=maps.pillarCounts>0;
    [maps.xMap,maps.yMap]=meshgrid(single(geometry.origin(1)+((1:dims(2))-.5)*maps.dx), ...
        single(geometry.origin(2)+((1:dims(1))-.5)*maps.dy));
    maps.xMap(~maps.occupiedMask)=NaN;maps.yMap(~maps.occupiedMask)=NaN;
    maps.supportEvidence=single(maps.pillarCounts);
    maps.pointScore=zeros(dims,'single');maps.lineScore=maps.pointScore;
    maps.normalOrientation=nan(dims,'single');maps.blobness=maps.pointScore;
    % Describe the same scene regardless of which semantic outputs are asked
    % for, so a statistics model does not change when another class is omitted.
    [maps.pointScore,maps.lineScore,maps.normalOrientation,maps.blobness]= ...
        buildPillarShapeScores(maps.supportEvidence,maps.occupiedMask,cfg,maps.dx,maps.dy);
    maps.facadeLineScore=maps.lineScore;
    maps.moments=projectMoments(stats,prod(dims),cloudCfg);
    gates=struct('poleMinimumPoints',3,'poleMinimumHeight',0.5, ...
        'poleMinimumHeightStd',0.05,'facadeMinimumPoints',3,'facadeMinimumHeight',0.5);
    if isfield(cfg,'pillarStatistics')
        for name=fieldnames(gates).'
            if isfield(cfg.pillarStatistics,name{1}),gates.(name{1})=cfg.pillarStatistics.(name{1});end
        end
    end
    covariance=stats.covarianceXYZ;
    verticalFraction=covariance(:,6)./max(covariance(:,1)+covariance(:,3)+covariance(:,6),eps);
    verticalFraction=min(max(verticalFraction,0),1);
    poleMask=false(dims);facadeMask=false(dims);signMask=false(dims);
    poleEvidence=zeros(dims);facadeEvidence=zeros(dims);signEvidence=zeros(dims);
    if any(string(cloudCfg.semanticNames)=="pole")
        poleMask(ids)=stats.count>=gates.poleMinimumPoints & height>=gates.poleMinimumHeight & ...
            covariance(:,6)>=gates.poleMinimumHeightStd^2;
        poleEvidence(ids)=verticalFraction.*height./(height+1);
    end
    if any(ismember(string(cloudCfg.semanticNames),["facade","pole"]))
        facadeMask(ids)=stats.count>=gates.facadeMinimumPoints & height>=gates.facadeMinimumHeight;
        lineScore=reshape(double(maps.lineScore(ids)),[],1);
        facadeEvidence(ids)=height./(height+2).*max(lineScore,0.1);
    end
    if any(string(cloudCfg.semanticNames)=="trafficSign") && isfield(stats,'intensity')
        maximum=stats.intensity.maximum;
        signMask(ids)=isfinite(maximum) & maximum>cfg.trafficSignIntensityThreshold;
        evidence=max(0,1-cfg.trafficSignIntensityThreshold./max(maximum,eps));
        evidence(~isfinite(evidence))=0;signEvidence(ids)=evidence;
    end
    maps.poleEvidence=single(poleEvidence);
    maps.trafficSignCellMask=signMask;maps.trafficSignEvidence=signEvidence;
    maps.trafficSignMoments=maps.moments;
    facade=struct('mask',facadeMask,'lineMap',zeros(dims,'uint16'), ...
        'detectedLines',zeros(0,4),'pillarLinIdx',find(facadeMask), ...
        'pillarLineIdx',zeros(nnz(facadeMask),1));
    floorProbability=cloudCfg.minimumSemanticProbability;
    probability=@(mask,evidence) single(mask.*(floorProbability+(1-floorProbability)*evidence));
    result=struct('columnMaps',maps,'facade',facade,'facadeCellMask',facadeMask, ...
        'facadeProbability',probability(facadeMask,facadeEvidence), ...
        'poleCellMask',poleMask,'poleProbability',probability(poleMask,poleEvidence), ...
        'trafficSignCellMask',signMask,'trafficSignProbability',probability(signMask,signEvidence), ...
        'candidates',struct('eligibleMask',maps.occupiedMask,'candidateMask',poleMask), ...
        'classificationStage',"wholePillarDistributionStatistics");
end

function moments=projectMoments(stats,number,cfg)
% Rotate complete XYZ moments before publishing their exact XY marginal.
    ids=double(stats.pillarIndices);rotation=cfg.projectionRotation;
    center=stats.meanXYZ*rotation.'+cfg.projectionTranslation;
    packed=stats.covarianceXYZ;projected=zeros(size(packed));
    pairs=[1 1;1 2;2 2;1 3;2 3;3 3];
    for k=1:6
        a=rotation(pairs(k,1),:);b=rotation(pairs(k,2),:);
        coefficients=[a(1)*b(1),a(1)*b(2)+a(2)*b(1),a(2)*b(2), ...
            a(1)*b(3)+a(3)*b(1),a(2)*b(3)+a(3)*b(2),a(3)*b(3)];
        projected(:,k)=sum(packed.*coefficients,2);
    end
    moments=struct('count',zeros(number,1),'mean',zeros(number,2), ...
        'covariance',zeros(number,3),'meanZ',zeros(number,1),'heightCovariance',zeros(number,3));
    moments.count(ids)=stats.count;moments.mean(ids,:)=center(:,1:2);
    moments.meanZ(ids)=center(:,3);moments.covariance(ids,:)=projected(:,1:3);
    moments.heightCovariance(ids,:)=projected(:,4:6);
end
