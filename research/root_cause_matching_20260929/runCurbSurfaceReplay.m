function runCurbSurfaceReplay()
% runCurbStepReplay Evaluate continuous curb-edge geometry on all raw scans.
    root=setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));out='output/root_cause_matching_20260929';
    cfg=rootCausePerceptionConfig();cfg.coarseProbabilityCloud.storeDiagnostics=true;mc=featureMapBuildConfig();
    poses=readFramePoseTable(fullfile(root,'data',mc.poseMatchCsvPath),1:1170);store=matfile(fullfile(root,'data',mc.pointCloudMatPath));
    old=load('output/line_direction_matching_20260928/production/report.mat','report');calls=old.report.calls;odom=load('output/line_direction_matching_20260928/sources.mat','motion');
    baseline=load('output/pole_boundary_recovery_20260929/replay.mat','currentClouds');currentClouds=cell(1170,1);sources=currentClouds;wc=localizationSourceWindowConfig();history=[];frames=[];first=0;rows=cell(1170,4);
    for k=1:1170
        if isempty(frames)||k>=first+numel(frames),first=k;frames=store.pointClouds(1,k:min(1170,k+49));end
        [~,tilt]=poseRowToPlanarPose(poses(k,:));cfg.coarseProbabilityCloud.projectionRotation=tilt;p=perceiveFrame(frames(k-first+1),cfg);
        assert(isequaln(p.probabilityCloud,baseline.currentClouds{k}));
        g=p.diagnostics.ground;timer=tic;[g.moments,detail]=estimateCurbSurfaceMoments(g,p.diagnostics.groundPointContext,p.probabilityCloud.projectionRotation,p.probabilityCloud.projectionTranslation);ms=1000*toc(timer);
        cc=cfg.coarseProbabilityCloud;cc.semanticNames=cfg.featureNames;cc.projectionRotation=p.probabilityCloud.projectionRotation;cc.projectionTranslation=p.probabilityCloud.projectionTranslation;cc.frameCalibration=p.probabilityCloud.frameCalibration;
        currentClouds{k}=buildCoarseSemanticProbabilityCloud(g,p.diagnostics.offGround,cc);
        [sources{k},history]=updateLocalizationSourceWindow(currentClouds{k},calls.timeSeconds(k),odom.motion(k,:),history,wc);
        rows(k,:)={k,nnz(g.curbCellMask),nnz(detail.accepted),ms};
        if mod(k,100)==0,fprintf('Curb surface %d/1170\n',k);end
    end
    file=fullfile(out,'curbSurface_sources.mat');save(file,'sources','currentClouds','cfg','-v7.3');
    writetable(cell2table(rows,VariableNames={'frame','curbPillars','localizedSteps','extraMs'}),fullfile(dest,'curb_surface_metrics.csv'));
    rootCauseReplay(file,"curbSurface",distributionRegistrationConfig());
end
