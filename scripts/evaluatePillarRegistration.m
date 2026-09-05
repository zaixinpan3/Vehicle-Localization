function report = evaluatePillarRegistration(dataRoot, outputFolder)
% evaluatePillarRegistration: Recorded map consistency and offline integration.
% The saved-map experiment uses the same trajectory as mapping and is NOT an
% independent localization accuracy benchmark. A second six-frame map built
% through the new offline path excludes the query frame from map fitting.
% Recorded attitude supplies known tilt; SE(2) predictions are perturbed by
% deterministic offsets. Output differences reference mapping poses only.
    root=fileparts(fileparts(mfilename('fullpath')));
    run(fullfile(root,'setupVehicleLocalization.m'));
    if nargin<1 || strlength(string(dataRoot))==0, dataRoot=fullfile(root,'data'); end
    if nargin<2 || strlength(string(outputFolder))==0, outputFolder=fullfile(root,'output','perception_redesign'); end
    if ~isfolder(outputFolder), mkdir(outputFolder); end
    loaded=load(fullfile(dataRoot,'missisipiSemanticTemporalStabilityGMMProbabilityCloudMap_allFrames.mat'),'probabilityCloudMap');
    map=loaded.probabilityCloudMap;
    mapCfg=featureMapBuildConfig(); mapCfg.logEnabled=false;
    matPath=fullfile(dataRoot,mapCfg.pointCloudMatPath);
    posePath=fullfile(dataRoot,mapCfg.poseMatchCsvPath);
    frames=[260 300 326 150 600 900 1100];
    poses=readFramePoseTable(posePath,frames);
    offsets=[0 0 0;0.5 -0.4 deg2rad(2);-0.5 0.4 -deg2rad(2)];
    rows=cell(numel(frames)*size(offsets,1)+1,13);
    cfg=struct('perception',perceptionConfig(),'registration',distributionRegistrationConfig());
    cfg.perception.coarseProbabilityCloud.coordinateFrame="gravityAlignedVehicleXY";
    index=0;
    for k=1:numel(frames)
        frame=loadPointCloudFrame(matPath,frames(k));
        [pose,tilt]=poseRowToPlanarPose(poses(k,:));
        cfg.perception.coarseProbabilityCloud.projectionRotation=tilt;
        containsFrame=arrayfun(@(b) ismember(frames(k),b.frameIndices),map.batchMaps);
        batchIndex=find(containsFrame,1,'first');
        fixed=temporalMapToProbabilityCloud(map,batchIndex);
        for j=1:size(offsets,1)
            [~,result]=localizeLidarFrame(frame,fixed,pose+offsets(j,:),0,cfg);
            index=index+1;
            rows(index,:)=metricRow("savedMapSameTrajectory",frames(k),batchIndex,offsets(j,:),pose,result);
        end
    end
    mappingFrames=261:266;
    mappingPoses=readFramePoseTable(posePath,mappingFrames);
    observations=collectFeatureObservations(matPath,mappingFrames,mappingPoses,perceptionConfig(),mapCfg);
    rebuiltMap=buildSlidingWindowMap(observations,mapCfg);
    fixed=temporalMapToProbabilityCloud(rebuiltMap,1);
    frame=loadPointCloudFrame(matPath,260);
    [pose,tilt]=poseRowToPlanarPose(poses(1,:));
    cfg.perception.coarseProbabilityCloud.projectionRotation=tilt;
    [~,result]=localizeLidarFrame(frame,fixed,pose+offsets(2,:),0,cfg);
    rows(end,:)=metricRow("newMapQueryFrameExcluded",260,1,offsets(2,:),pose,result);
    report.metrics=cell2table(rows,'VariableNames',{'experiment','frameIndex','batchIndex','initialTranslationOffset','initialYawOffsetDegrees','accepted','reason','initialSimilarity','similarity','mapPoseTranslationDifference','mapPoseYawDifferenceDegrees','perceptionSeconds','registrationSeconds'});
    report.mappingFrames=mappingFrames;
    report.description="Map-pose consistency, not independent ground-truth accuracy. Six-frame rebuilt map excludes query frame 260.";
    writetable(report.metrics,fullfile(outputFolder,'registration_consistency.csv'));
    save(fullfile(outputFolder,'registration_evaluation.mat'),'report','rebuiltMap');
    disp(report.metrics);
end

function row=metricRow(experiment,frame,batch,offset,pose,result)
    yaw=result.poseXYTheta(3)-pose(3);
    row={experiment,frame,batch,norm(offset(1:2)),rad2deg(offset(3)),result.accepted,result.reason, ...
        result.initialSimilarity,result.similarity,norm(result.poseXYTheta(1:2)-pose(1:2)), ...
        rad2deg(atan2(sin(yaw),cos(yaw))),result.perceptionSeconds,result.registrationSeconds};
end
