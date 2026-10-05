function report=buildDowntownReferenceFeatureMap(datasetFolder,outputFolder)
% buildDowntownReferenceFeatureMap Four-class map from selected offline poses.
% Only odd original raw frame IDs enter mapping. Even queries remain excluded.
% Mapping deskew uses reference SE3 interpolation; source clouds retain raw axes.
    setupVehicleLocalization();if ~isfolder(outputFolder),mkdir(outputFolder);end
    protocol=downtownReferenceReplayConfig();poses=readtable(fullfile(datasetFolder,'mapping_poses.csv'));
    frames=readtable(fullfile(datasetFolder,'frames.csv'));matPath=buildDowntownReferenceClouds(datasetFolder);
    chosen=poses.frame_index(mod(poses.frame_index,2)==protocol.mapFrameParity & poses.lidar_stamp_sec<poses.lidar_stamp_sec(end)-.1002);
    p=perceptionConfig("Downtown","offline");p.featureNames=protocol.featureNames;
    cfg=featureMapBuildConfig();cfg.perceptionProfile="Downtown";cfg.frameCalibration=p.frameCalibration;cfg.featureNames=protocol.featureNames;cfg.logEnabled=false;
    cfg.temporalMap.frameCalibration=p.frameCalibration;
    featureData=struct('featureNames',protocol.featureNames,'frameIndices',chosen.', ...
        'frameCalibration',p.frameCalibration,'framePoseTable',readFramePoseTable(fullfile(datasetFolder,'mapping_poses.csv'),chosen), ...
        'pointsByFeatureFrame',{cell(4,numel(chosen))},'counts',zeros(numel(chosen),4));
    timer=tic;
    for j=1:numel(chosen)
        frame=loadPointCloudFrame(matPath,chosen(j));frame=deskewReferenceMappingFrame(frame,poses);
        perception=perceiveFrame(frame,p);xyz=[double(frame.x(:)),double(frame.y(:)),double(frame.z(:))];
        for c=1:4
            mask=perception.featureMasks.(protocol.featureNames(c));valid=logical(mask(:)) & all(isfinite(xyz),2);
            points=registerPointsToGlobalFrame(xyz(valid,:),featureData.framePoseTable(j,:));featureData.counts(j,c)=size(points,1);
            % Exactly the temporal map's deterministic per-acquisition representatives.
            if ~isempty(points)
                origin=cfg.temporalMap.defaultParams.tileOrigin;resolution=cfg.temporalMap.defaultParams.representativeResolution;
                voxel=floor((points(:,1:2)-origin)/resolution);center=origin+(voxel+.5)*resolution;
                [~,~,group]=unique(voxel,'rows');[~,order]=sortrows([group,sum((points(:,1:2)-center).^2,2),points(:,1:2)],[1,2,3,4]);
                keep=order([true;diff(group(order))~=0]);points=points(sort(keep),:);
            end
            featureData.pointsByFeatureFrame{c,j}=points;
        end
        if mod(j,25)==0 || j==numel(chosen),fprintf('map %s %d/%d\n',datasetFolder,j,numel(chosen));end
    end
    perceptionSeconds=toc(timer);save(fullfile(outputFolder,'feature_observations.mat'),'featureData','-v7.3');
    timer=tic;probabilityCloudMap=buildSlidingWindowMap(featureData,cfg);mapSeconds=toc(timer);
    probabilityCloudMap.evaluationTrainingFrames=chosen;probabilityCloudMap.poseSource="independent_offline_pseudo_ground_truth";
    cloud=temporalMapToProbabilityCloud(probabilityCloudMap);
    cloud.trainingFrameIndices=chosen;cloud.referenceKind=protocol.referenceKind;
    save(fullfile(outputFolder,'probability_cloud_map.mat'),'probabilityCloudMap','-v7.3');
    save(fullfile(outputFolder,'probability_cloud.mat'),'cloud','-v7.3');
    writetable(probabilityCloudMap.layerSummaryTable,fullfile(outputFolder,'map_layer_summary.csv'));
    report=struct('datasetFolder',datasetFolder,'trainingFrames',chosen,'featureNames',protocol.featureNames, ...
        'observedPoints',sum(featureData.counts,1),'components',cloud.components.numComponents, ...
        'layerSummary',probabilityCloudMap.layerSummaryTable,'perceptionSeconds',perceptionSeconds,'mapSeconds',mapSeconds, ...
        'poseReferencePoint',"front LiDAR origin",'referenceKind',protocol.referenceKind, ...
        'gnssUsed',false,'mappingDeskew',"Reference SE3 interpolation within each scan; no extrapolation", ...
        'thinning',"Same 0.1 m deterministic XY representative used by the temporal map",'queryFrames',frames.frame_index(mod(frames.frame_index,2)==0 & frames.available==1));
    f=fopen(fullfile(outputFolder,'map_summary.json'),'w');fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));fclose(f);
end
