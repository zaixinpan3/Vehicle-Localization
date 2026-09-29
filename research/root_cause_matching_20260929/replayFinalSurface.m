function replayFinalSurface()
% replayFinalSurface Recompute raw production geometry and unchanged detections.
    root=setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));out='output/root_cause_matching_20260929';
    cfg=perceptionConfig('Mississippi');cfg.coarseProbabilityCloud.storeDiagnostics=true;baseCfg=cfg;baseCfg.curbBoundary.enabled=false;mc=featureMapBuildConfig();
    poses=readFramePoseTable(fullfile(root,'data',mc.poseMatchCsvPath),1:1170);store=matfile(fullfile(root,'data',mc.pointCloudMatPath));
    old=load('output/line_direction_matching_20260928/production/report.mat','report');calls=old.report.calls;odom=load('output/line_direction_matching_20260928/sources.mat','motion');
    baseline=load('output/pole_boundary_recovery_20260929/replay.mat','currentClouds');expected=load(fullfile(out,'boundaryScatter2_sources.mat'),'currentClouds','sources');currentClouds=cell(1170,1);sources=currentClouds;wc=localizationSourceWindowConfig();history=[];frames=[];first=0;rows=cell(1170,9);
    for k=1:1170
        if isempty(frames)||k>=first+numel(frames),first=k;frames=store.pointClouds(1,k:min(1170,k+49));end
        raw=frames(k-first+1);[~,tilt]=poseRowToPlanarPose(poses(k,:));cfg.coarseProbabilityCloud.projectionRotation=tilt;baseCfg.coarseProbabilityCloud.projectionRotation=tilt;
        if mod(k,2)
            timer=tic;a=perceiveFrame(raw,baseCfg);oldMs=1000*toc(timer);timer=tic;p=perceiveFrame(raw,cfg);newMs=1000*toc(timer);
        else
            timer=tic;p=perceiveFrame(raw,cfg);newMs=1000*toc(timer);timer=tic;a=perceiveFrame(raw,baseCfg);oldMs=1000*toc(timer);
        end
        assert(isequaln(a.probabilityCloud,baseline.currentClouds{k}));
        assert(isequaln(a.candidates,p.candidates));assert(isequaln(a.diagnostics.ground,p.diagnostics.ground));assert(isequaln(a.diagnostics.offGround,p.diagnostics.offGround));
        c=p.probabilityCloud.components;e=expected.currentClouds{k}.components;
        sameComponents=isequal(c.semanticName,e.semanticName);meanError=NaN;covError=NaN;
        if sameComponents,meanError=max(abs(c.mean-e.mean),[],'all');covError=max(abs(c.covariance-e.covariance),[],'all');end
        currentClouds{k}=p.probabilityCloud;[sources{k},history]=updateLocalizationSourceWindow(currentClouds{k},calls.timeSeconds(k),odom.motion(k,:),history,wc);
        rows(k,:)={k,true,nnz(p.diagnostics.ground.curbCellMask),nnz(p.diagnostics.curbBoundaryFit.accepted),meanError,covError,oldMs,newMs,sameComponents};
        if mod(k,100)==0,fprintf('Final raw verification %d/1170\n',k);end
    end
    file=fullfile(out,'finalSurface_sources.mat');save(file,'sources','currentClouds','cfg','wc','-v7.3');
    writetable(cell2table(rows,VariableNames={'frame','unchangedDetections','curbPillars','localizedBoundaries','meanDifference','covarianceDifference','baselinePerceptionMs','finalPerceptionMs','samePrototypeComponents'}),fullfile(dest,'final_raw_verification.csv'));
    rootCauseReplay(file,"finalSurface",distributionRegistrationConfig());
end
