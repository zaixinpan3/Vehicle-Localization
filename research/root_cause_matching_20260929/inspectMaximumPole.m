function inspectMaximumPole(frameIndex)
% inspectMaximumPole Inspect detection evidence separately from map associations.
    root=setupVehicleLocalization();cfg=perceptionConfig('Mississippi');cfg.coarseProbabilityCloud.storeDiagnostics=true;
    raw=loadPointCloudFrame(fullfile(root,'data/raw/MissisipiPointClouds.mat'),frameIndex);p=perceiveFrame(raw,cfg);
    q=p.diagnostics.offGroundPointContext;m=p.diagnostics.offGround.columnMaps;g=q.pillarGeometry;original=q.sourcePillarGeometry;
    offset=round((g.origin-original.origin)./g.cellSize);rows=(1:g.mapSize(1))+offset(2);cols=(1:g.mapSize(2))+offset(1);
    support=zeros(original.mapSize,'single');occupied=false(original.mapSize);support(rows,cols)=m.supportEvidence;occupied(rows,cols)=m.occupiedMask;
    shapeCfg=cfg.offGroundFeatures;shapeCfg.useNativeKernels=perceptionNativeAvailable(cfg.executionBackend);[point,line]=buildPillarShapeScores(support,occupied,shapeCfg,g.cellSize(1),g.cellSize(2));point=point(rows,cols);line=line(rows,cols);
    mask=~(isfinite(q.pointAttributes.intensity)&q.pointAttributes.intensity>cfg.offGroundFeatures.trafficSignIntensityThreshold);
    [e,d]=classifyPillarPoleSupport(q.points,q.pointPillarLinIdx,g,m.poleValidationProposals,cfg.offGroundFeatures.pole.validation,mask,point,line,cfg.offGroundFeatures.pole.distributionValidation);
    assert(isequaln(e,m.poleValidation));model=loadPillarPoleModel(cfg.offGroundFeatures.pole.distributionValidation.modelFile);
    t=array2table(d.features,VariableNames=string(model.featureNames));t.score=d.scores;t.owner=d.owners;t.hypothesis=d.hypothesisIds;
    [r,c]=ind2sub(g.mapSize,t.owner);t.globalPillar=sub2ind(original.mapSize,r+offset(2),c+offset(1));
    path=fullfile(fileparts(mfilename('fullpath')),'viewer',sprintf('features_%04d.json',frameIndex));reference=jsondecode(fileread(path));ids=reference.features.pole.referencePillarIds;
    t.hasReferencePole=ismember(t.globalPillar,ids);t.selected=ismember(t.globalPillar,reference.features.pole.selectedPillarIds);
    writetable(t,fullfile(fileparts(mfilename('fullpath')),sprintf('maximum_pole_%d.csv',frameIndex)));
    disp(t(t.hasReferencePole|t.selected,intersect(string(t.Properties.VariableNames),["globalPillar","score","selected","hasReferencePole","ownerCount","ownerHeight","ownerSupportFraction","supportHeight","radialRms","isolation","longestHeight","maximumGap","tilt","minimumQuarterCount"])));
end
