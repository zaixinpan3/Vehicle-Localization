function validation=validate_five_frame_horizon()
% validate_five_frame_horizon Verify the deployed window on real coarse scans.
    root=setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    saved=load(fullfile(root,'output/temporal_perception_20260922/five_frame_matching/report.mat'),'cfg','report');
    window=localizationSourceWindowConfig();
    assert(isequaln(window,saved.cfg.sourceWindow) && window.maximumFrames==5);
    assert(window.maximumAgeSeconds==.45 && window.minimumDetectionFrames==2);
    cfg=saved.cfg;cfg=rmfield(cfg,'sourceWindow'); % Exercise the online default.
    mapCfg=featureMapBuildConfig();frames=955:959;
    poses=readFramePoseTable(fullfile(root,'data',mapCfg.poseMatchCsvPath),frames);
    loaded=load(fullfile(root,'output/mississippi_mapping_calibrated/probability_cloud.mat'),'cloud');
    cloud=registrationSupport.projectSemanticProbabilityCloud(loaded.cloud,2);
    h=[];
    for k=1:numel(frames)
        frame=frames(k);row=saved.report.calls(frame,:);motion=saved.report.deadReckoning(frame,:);
        [~,tilt]=poseRowToPlanarPose(poses(k,:));
        cfg.perception.coarseProbabilityCloud.projectionRotation=tilt;
        input=loadPointCloudFrame(fullfile(root,'data',mapCfg.pointCloudMatPath),frame);
        [~,result,h]=localizeLidarFrame(input,cloud,[row.predictedX row.predictedY row.predictedPsi], ...
            row.timeSeconds,cfg,h,[motion.x motion.y motion.psi]);
    end
    c=result.probabilityCloud.components;
    assert(numel(h.time)==5 && result.sourceWindow.frameCount==5);
    assert(all(c.detectionFrameCount>=2) && all(c.detectionFrameCount<=5));
    assert(all(abs(c.temporalStability-c.detectionFrameCount/5)<1e-12));
    assert(c.numComponents==saved.report.sourceWindows.components(959));
    tests=readtable(fullfile(dest,'tests.csv'));assert(height(tests)==39 && all(tests.Passed));
    calls=saved.report.calls;
    validation=struct('testsPassed',height(tests),'defaultWindow',window, ...
        'freshRealFrames',frames,'onlineDefaultUsed',true,'retainedRealFrames',numel(h.time), ...
        'confirmedComponents',c.numComponents,'rejectedSingletons',result.sourceWindow.rejectedSingletons, ...
        'minimumSupport',min(c.detectionFrameCount),'maximumSupport',max(c.detectionFrameCount), ...
        'identicalToExistingFullSequenceConfiguration',true,'fullSequenceRerunThisChange',false, ...
        'existingReplayFrames',height(calls),'existingReplayFullPoses',nnz(calls.accepted), ...
        'existingReplayTrajectoryRmseM',sqrt(mean(calls.positionErrorM.^2)), ...
        'existingReplayFullPoseRmseM',sqrt(mean(calls.positionErrorM(calls.accepted).^2)), ...
        'existingReplayMaximumM',max(calls.positionErrorM));
    fid=fopen(fullfile(dest,'validation.json'),'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(validation,PrettyPrint=true));disp(validation);
end
