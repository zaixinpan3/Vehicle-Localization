function runCurbStepVariants()
% runCurbStepReplay Evaluate continuous curb-edge geometry on all raw scans.
    root=setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));out='output/root_cause_matching_20260929';
    cfg=rootCausePerceptionConfig();cfg.coarseProbabilityCloud.storeDiagnostics=true;mc=featureMapBuildConfig();
    poses=readFramePoseTable(fullfile(root,'data',mc.poseMatchCsvPath),1:1170);store=matfile(fullfile(root,'data',mc.pointCloudMatPath));
    old=load('output/line_direction_matching_20260928/production/report.mat','report');calls=old.report.calls;odom=load('output/line_direction_matching_20260928/sources.mat','motion');
    baseline=load('output/pole_boundary_recovery_20260929/replay.mat','currentClouds');currentClouds=cell(1170,1);sources=currentClouds;wc=localizationSourceWindowConfig();histories=cell(3,1);allCurrent=cell(1170,3);allSources=allCurrent;frames=[];first=0;rows=cell(1170*3,5);
    for k=1:1170
        if isempty(frames)||k>=first+numel(frames),first=k;frames=store.pointClouds(1,k:min(1170,k+49));end
        [~,tilt]=poseRowToPlanarPose(poses(k,:));cfg.coarseProbabilityCloud.projectionRotation=tilt;p=perceiveFrame(frames(k-first+1),cfg);
        assert(isequaln(p.probabilityCloud,baseline.currentClouds{k}));
        for mode=2:4
        g=p.diagnostics.ground;timer=tic;[g.moments,detail]=estimateCurbStepMoments(g,p.diagnostics.groundPointContext,p.probabilityCloud.projectionRotation,p.probabilityCloud.projectionTranslation,mode);ms=1000*toc(timer);
        cc=cfg.coarseProbabilityCloud;cc.semanticNames=cfg.featureNames;cc.projectionRotation=p.probabilityCloud.projectionRotation;cc.projectionTranslation=p.probabilityCloud.projectionTranslation;cc.frameCalibration=p.probabilityCloud.frameCalibration;
        currentClouds{k}=buildCoarseSemanticProbabilityCloud(g,p.diagnostics.offGround,cc);
        [sources{k},histories{mode-1}]=updateLocalizationSourceWindow(currentClouds{k},calls.timeSeconds(k),odom.motion(k,:),histories{mode-1},wc);
        allCurrent{k,mode-1}=currentClouds{k};allSources{k,mode-1}=sources{k};
        rows((mode-2)*1170+k,:)={mode,k,nnz(g.curbCellMask),nnz(detail.accepted),ms};
        end
        if mod(k,100)==0,fprintf('Curb step %d/1170\n',k);end
    end
    writetable(cell2table(rows,VariableNames={'mode','frame','curbPillars','localizedSteps','extraMs'}),fullfile(dest,'curb_step_variants_metrics.csv'));
    for mode=2:4
        sources=allSources(:,mode-1);currentClouds=allCurrent(:,mode-1);label="curbStep"+mode;
        file=fullfile(out,label+"_sources.mat");save(file,'sources','currentClouds','cfg','-v7.3');
        rootCauseReplay(file,label,distributionRegistrationConfig());
    end
end
