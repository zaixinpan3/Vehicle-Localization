function diagnoseReplayDifference(frameId,dataset,prefix)
% diagnoseReplayDifference: Compare captured and recomputed feature values.
    if nargin<2,dataset='Mississippi';end
    if nargin<3,prefix='moments_';end
    root=setupVehicleLocalization();cfg=geometryCandidateConfig(dataset,prefix);
    cfg.voxel.useNativeKernels=true;cfg.groundSegmentation.useNativeKernels=true;cfg.offGroundFeatures.useNativeKernels=true;
    file='MissisipiPointClouds.mat';if strcmpi(dataset,'Downtown'),file='downTownPointClouds.mat';end
    source=matfile(fullfile(root,'data','raw',file));frame=source.pointClouds(1,frameId);
    g=pillarizePointCloud(frame,cfg.voxel);ground=segmentGround(g,cfg.groundSegmentation);keep=~ismember(g.pointIndices,ground);
    p=struct('points',g.points(keep,:),'pointPillarLinIdx',g.pointPillarLinIdx(keep),'pillarGeometry',g.pillarGeometry, ...
        'pointAttributes',struct('intensity',g.pointAttributes.intensity(keep)));
    cloud=coarseSemanticProbabilityCloudConfig(cfg.voxel);cloud.semanticNames="pole";
    old=cfg.offGroundFeatures;old.pole.distributionValidation.enabled=false;off=analyzeStructuralPillars(p,old,cloud);m=off.columnMaps;
    mask=~(isfinite(p.pointAttributes.intensity)&p.pointAttributes.intensity>cfg.offGroundFeatures.trafficSignIntensityThreshold);
    [e,d]=classifyPillarPoleSupport(p.points,p.pointPillarLinIdx,g.pillarGeometry,m.poleValidationProposals, ...
        cfg.offGroundFeatures.pole.validation,mask,m.pointScore,m.lineScore,cfg.offGroundFeatures.pole.distributionValidation);
    label=lower(dataset);a=readtable(fullfile(root,'output','pole_geometry_20260927',[label '_features.csv']),'TextType','string');b=readtable(fullfile(root,'output','pole_geometry_20260927',[label '_moments.csv']),'TextType','string');
    keys=setdiff(b.Properties.VariableNames,{'dataset','frame','pillar','hypothesis'},'stable');T=[a b(:,keys)];T=T(T.frame==frameId,:);
    model=loadPillarPoleModel(cfg.offGroundFeatures.pole.distributionValidation.modelFile);
    assert(isequal(T.pillar,d.owners)&&isequal(T.hypothesis,d.hypothesisIds));X=T{:,model.featureNames};
    expected=scorePillarPoleModel(X,model);delta=abs(X-d.features);[i,j]=find(delta>1e-12);
    disp(table(T.pillar,d.scores,expected,'VariableNames',{'owner','current','cached'}));
    disp(table(i,string(model.featureNames(j)),X(sub2ind(size(X),i,j)),d.features(sub2ind(size(X),i,j)), ...
        'VariableNames',{'row','feature','cached','current'}));
    tag=sprintf('%s_%d_%s',label,frameId,prefix);
    [i,j]=find(delta>0);D=table(i,string(model.featureNames(j)),X(sub2ind(size(X),i,j)),d.features(sub2ind(size(X),i,j)), ...
        'VariableNames',{'row','feature','cached','current'});writetable(D,fullfile(root,'output','pole_geometry_20260927',[tag 'roundoff.csv']));
    save(fullfile(root,'output','pole_geometry_20260927',[tag 'difference.mat']),'d','X','model','e');
    cfg.coarseProbabilityCloud.storeDiagnostics=true;pipeline=perceiveFrame(frame,cfg);disp('Full-pipeline owners:');disp(pipeline.candidates.pillarIndices{pipeline.candidates.semanticNames=="pole"});
    pp=pipeline.diagnostics.offGround.columnMaps;
    fprintf('Full/cropped map sizes: %s / %s\n',mat2str(m.mapSize),mat2str(pp.mapSize));
    if isfield(model,'featureScale')
        [i,j]=find(floor(X*model.featureScale+.5)~=floor(d.features*model.featureScale+.5));
        disp('Canonical feature mismatches:');disp(table(i,string(model.featureNames(j)),X(sub2ind(size(X),i,j)),d.features(sub2ind(size(X),i,j)), ...
            'VariableNames',{'row','feature','cached','current'}));
    end
end
