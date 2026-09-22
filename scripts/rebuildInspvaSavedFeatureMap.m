function report = rebuildInspvaSavedFeatureMap(outputFolder,options)
% rebuildInspvaSavedFeatureMap Rebuild the map from unchanged local features.
% Cached global features are reprojected with INSPVA poses at LiDAR times.
% Original perception/map artifacts remain the historical baseline.
    arguments
        outputFolder (1,1) string="output/mississippi_mapping_calibrated"
        options.ObservationFile (1,1) string="output/mississippi_perception_video_20260912/feature_observations.mat"
        options.PoseFile (1,1) string=""
        options.FrameCalibration (1,1) struct=struct()
    end
    setupVehicleLocalization();
    if ~isfolder(outputFolder),mkdir(outputFolder);end
    cfg=featureMapBuildConfig();cfg.frameIndices=1:1170;
    if strlength(options.PoseFile)==0,options.PoseFile=fullfile("data",cfg.poseMatchCsvPath);end
    poses=readFramePoseTable(options.PoseFile,cfg.frameIndices);
    assert(all(poses.pose_source=="INSPVA"),'Expected INSPVA-only mapping poses.');
    root=fileparts(fileparts(mfilename('fullpath')));
    clock=loadReceiverClock(fullfile(root,'data','raw','Missisipi','gnss', ...
        'raw_data_2024-06-07-12-09-31_0_inspva.csv'));
    assert(ismember('clock_model_id',poses.Properties.VariableNames) && ...
        all(string(poses.clock_model_id)==string(clock.modelId)), ...
        'VehicleLocalization:ClockMismatch','Rebuild requires synchronized pose epochs.');
    loaded=load(options.ObservationFile,'featureData');original=loaded.featureData;
    calibration=cfg.frameCalibration;
    if ~isempty(fieldnames(options.FrameCalibration)),calibration=validateLidarFrameCalibration(options.FrameCalibration);end
    [featureData,reprojection]=reprojectSavedFeatureObservations(original,poses,FrameCalibration=calibration);
    assert(isequal(sum(featureData.counts,1),[133836,194300,70950]),'Unexpected perception point counts.');
    cfg.featureNames=featureData.featureNames;cfg.frameCalibration=featureData.frameCalibration;
    save(fullfile(outputFolder,'feature_observations.mat'),'featureData','-v7.3');
    save(fullfile(outputFolder,'mapping_configuration.mat'),'cfg','options');
    timer=tic;probabilityCloudMap=buildSlidingWindowMap(featureData,cfg);buildSeconds=toc(timer);
    probabilityCloudMap.sourceObservationPath=fullfile(outputFolder,'feature_observations.mat');
    probabilityCloudMap.poseMatchCsvPath=options.PoseFile;
    probabilityCloudMap.clockModelId=string(clock.modelId);
    probabilityCloudMap.receiverClock=clock;
    probabilityCloudMap.poseSource="INSPVA";
    probabilityCloudMap.poseCrs="EPSG:32615";
    probabilityCloudMap.heightDatum="ellipsoidal";
    mappingSupport.validateRepeatedObservationMap(probabilityCloudMap.canonicalMap);
    cloud=temporalMapToProbabilityCloud(probabilityCloudMap);
    assert(cloud.components.numComponents>0,'No published structure.');
    rows=unique(round(linspace(1,cloud.components.numComponents,min(100,cloud.components.numComponents))));
    queries=cloud.components.mean(rows,:);
    [scores,details]=queryTemporalStabilityGmmMap(probabilityCloudMap,queries);
    intensity=zeros(size(scores));layers=probabilityCloudMap.canonicalMap.layers;
    for j=1:numel(layers)
        keep=find(cloud.components.semanticName==layers(j).classLabel);
        for k=keep(:).'
            covariance=cloud.components.covariance(:,:,k);delta=queries-cloud.components.mean(k,:);
            intensity(:,j)=intensity(:,j)+cloud.classTotalMass(j)*cloud.components.classMixtureWeight(k)/ ...
                (2*pi*sqrt(det(covariance)))*exp(-.5*sum((delta/covariance).*delta,2));
        end
    end
    reconstructed=intensity./(intensity+cloud.clutterIntensity.');
    assert(all(abs(intensity-details.intensity)<=details.intensityErrorBound+1e-10,'all'));
    assert(all(abs(reconstructed(details.valid)-scores(details.valid))<=details.scoreErrorBound(details.valid)+1e-10));
    report=struct('sourceObservationFile',options.ObservationFile,'poseFile',options.PoseFile, ...
        'frames',height(poses),'poseSource',"INSPVA",'poseCrs',"EPSG:32615",'heightDatum',"ellipsoidal", ...
        'clock',clock,'reprojection',reprojection,'buildSeconds',buildSeconds,'publishedComponents',cloud.components.numComponents, ...
        'totalPublishedMass',cloud.totalMass,'maximumIntensityReconstructionError',max(abs(intensity-details.intensity),[],'all'), ...
        'maximumScoreReconstructionError',max(abs(reconstructed(details.valid)-scores(details.valid))), ...
        'schemaAndMassChecksPassed',true,'perceptionRerun',false, ...
        'referenceLimitation',"Same-drive INSPVA map and query observations; consistency experiment, not independent ground truth");
    save(fullfile(outputFolder,'probability_cloud_map.mat'),'probabilityCloudMap','-v7.3');
    save(fullfile(outputFolder,'probability_cloud.mat'),'cloud','-v7.3');
    writetable(probabilityCloudMap.layerSummaryTable,fullfile(outputFolder,'map_layer_summary.csv'));
    fid=fopen(fullfile(outputFolder,'validation.json'),'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));
    disp(report);
end
