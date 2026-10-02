function report=buildCarlaFeatureMap(datasetFolder,outputFolder,options)
% buildCarlaFeatureMap Semantic map of a prepared CARLA mapping drive.
% The production chain of buildMississippiFeatureMap with CARLA inputs:
% offline observations (collectCarlaFeatureObservations), the repeated-
% observation Gaussian map (buildSlidingWindowMap), its published probability
% cloud and the landmark-view conditioned cloud used by localization.
% ObservationFiles lists blocks collected in parallel processes; they are
% merged in sweep order. Without it every sweep is collected here.
% Thinning (default on) keeps, per sweep and class, the single return of each
% 0.10 m XY voxel closest to the voxel centre (ties: smaller x, then y). This
% is the representative buildTemporalStabilityGmmMap itself selects for every
% observation block (one block per sweep), so the map is unchanged while the
% builder no longer sorts millions of redundant facade returns.
% Outputs in outputFolder: feature_observations.mat, probability_cloud_map.mat,
% probability_cloud.mat, view_conditioned_cloud.mat, map_layer_summary.csv
% and validation.json.
    arguments
        datasetFolder (1,1) string
        outputFolder (1,1) string
        options.ObservationFiles (1,:) string=strings(1,0)
        options.FrameIndices (1,:) double=[]
        options.Thinning (1,1) logical=true
    end
    setupVehicleLocalization();
    if ~isfolder(outputFolder),mkdir(outputFolder);end
    perception=perceptionConfig("Carla","offline");
    cfg=carlaMapBuildConfig(perception);
    if isempty(options.ObservationFiles)
        frames=options.FrameIndices;
        if isempty(frames)
            poses=readtable(fullfile(datasetFolder,'poses.csv'),'TextType','string');frames=1:height(poses);
        end
        featureData=collectCarlaFeatureObservations(datasetFolder,frames);
    else
        featureData=mergeObservations(options.ObservationFiles);
    end
    save(fullfile(outputFolder,'feature_observations.mat'),'featureData','-v7.3');
    observedPoints=sum(featureData.counts,1);
    if options.Thinning
        featureData=thinToRepresentatives(featureData,cfg);
    end
    timer=tic;probabilityCloudMap=buildSlidingWindowMap(featureData,cfg);buildSeconds=toc(timer);
    probabilityCloudMap.sourceObservationPath=fullfile(outputFolder,'feature_observations.mat');
    probabilityCloudMap.poseSource="CARLA_ground_truth";
    mappingSupport.validateRepeatedObservationMap(probabilityCloudMap.canonicalMap);
    cloud=temporalMapToProbabilityCloud(probabilityCloudMap);
    assert(cloud.components.numComponents>0,'No published structure.');
    save(fullfile(outputFolder,'probability_cloud_map.mat'),'probabilityCloudMap','-v7.3');
    save(fullfile(outputFolder,'probability_cloud.mat'),'cloud','-v7.3');
    publishedComponents=cloud.components.numComponents;
    cloud=buildViewConditionedLandmarkMap(cloud,featureData,cfg.landmarkViews);
    save(fullfile(outputFolder,'view_conditioned_cloud.mat'),'cloud','-v7.3');
    writetable(probabilityCloudMap.layerSummaryTable,fullfile(outputFolder,'map_layer_summary.csv'));
    names=cloud.components.semanticName;
    report=struct('datasetFolder',datasetFolder,'frames',numel(featureData.frameIndices), ...
        'featureNames',featureData.featureNames,'observedPoints',observedPoints, ...
        'mapInputPoints',sum(featureData.counts,1),'thinning',options.Thinning, ...
        'publishedComponents',publishedComponents,'viewConditionedComponents',cloud.components.numComponents, ...
        'componentsByClass',struct('curb',nnz(names=="curb"),'pole',nnz(names=="pole"),'facade',nnz(names=="facade")), ...
        'buildSeconds',buildSeconds,'poseSource',"CARLA ground truth at the kinematic reference point", ...
        'frameCalibration',perception.frameCalibration.identifier);
    fid=fopen(fullfile(outputFolder,'validation.json'),'w');assert(fid>=0);
    fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));fclose(fid);
end

function featureData=thinToRepresentatives(featureData,cfg)
% One return per sweep, class and representative voxel (see the header).
    gmm=temporalStabilityMapConfig();if isfield(cfg,'temporalMap'),gmm=cfg.temporalMap;end
    for c=1:numel(featureData.featureNames)
        params=gmm.defaultParams;
        match=[gmm.classParams.classLabel]==featureData.featureNames(c);
        if any(match),params=gmm.classParams(match);end
        resolution=params.representativeResolution;origin=params.tileOrigin;
        assert(abs(params.tileSize/resolution-round(params.tileSize/resolution))<1e-10);
        for f=1:size(featureData.pointsByFeatureFrame,2)
            xyz=featureData.pointsByFeatureFrame{c,f};
            if size(xyz,1)<2,continue;end
            voxel=floor((xyz(:,1:2)-origin)/resolution);
            centre=origin+(voxel+0.5)*resolution;
            [~,~,group]=unique(voxel,'rows');
            [~,order]=sortrows([group sum((xyz(:,1:2)-centre).^2,2) xyz(:,1:2)],[1 2 3 4]);
            chosen=order([true;diff(group(order))~=0]);
            featureData.pointsByFeatureFrame{c,f}=xyz(sort(chosen),:);
            featureData.counts(f,c)=numel(chosen);
        end
    end
    featureData.frameSummaryTable{:,2:end}=featureData.counts;
end

function featureData=mergeObservations(files)
% Concatenate observation blocks in ascending sweep order.
    parts=cell(numel(files),1);
    for k=1:numel(files),loaded=load(files(k),'featureData');parts{k}=loaded.featureData;end
    [~,order]=sort(cellfun(@(p) p.frameIndices(1),parts));parts=parts(order);
    featureData=parts{1};
    for k=2:numel(parts)
        p=parts{k};
        assert(isequal(p.featureNames,featureData.featureNames),'Observation blocks use different classes.');
        assert(isequal(p.frameCalibration,featureData.frameCalibration),'Observation blocks use different calibrations.');
        featureData.frameIndices=[featureData.frameIndices,p.frameIndices];
        featureData.framePoseTable=[featureData.framePoseTable;p.framePoseTable];
        featureData.pointsByFeatureFrame=[featureData.pointsByFeatureFrame,p.pointsByFeatureFrame];
        featureData.counts=[featureData.counts;p.counts];
        featureData.frameSummaryTable=[featureData.frameSummaryTable;p.frameSummaryTable];
    end
    assert(all(diff(featureData.frameIndices)>0),'Observation blocks overlap or are out of order.');
end
