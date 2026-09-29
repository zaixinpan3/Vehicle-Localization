function runPoleAxisReplay()
% runPoleAxisReplay Locate accepted poles from the already fitted shaft axis.
% Pillar selection, support counts, scores and scatter remain unchanged.
    root=setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));out='output/root_cause_matching_20260929';
    cfg=rootCausePerceptionConfig();cfg.coarseProbabilityCloud.storeDiagnostics=true;mc=featureMapBuildConfig();
    poses=readFramePoseTable(fullfile(root,'data',mc.poseMatchCsvPath),1:1170);store=matfile(fullfile(root,'data',mc.pointCloudMatPath));
    old=load('output/line_direction_matching_20260928/production/report.mat','report');calls=old.report.calls;odom=load('output/line_direction_matching_20260928/sources.mat','motion');
    currentClouds=cell(1170,1);sources=currentClouds;wc=localizationSourceWindowConfig();history=[];frames=[];first=0;rows=cell(0,6);
    for k=1:1170
        if isempty(frames)||k>=first+numel(frames),first=k;frames=store.pointClouds(1,k:min(1170,k+49));end
        [~,tilt]=poseRowToPlanarPose(poses(k,:));cfg.coarseProbabilityCloud.projectionRotation=tilt;p=perceiveFrame(frames(k-first+1),cfg);
        off=p.diagnostics.offGround;e=off.columnMaps.poleValidation;m=off.columnMaps.moments;
        for id=find(off.poleCellMask).'
            i=find(e.pillarIndices==id);axis=[e.axisXY(i,:),e.axisZ(i)]*p.probabilityCloud.projectionRotation.'+p.probabilityCloud.projectionTranslation;
            if isempty(i)||any(~isfinite(axis)),continue;end
            rows(end+1,:)={k,id,m.mean(id,1),m.mean(id,2),axis(1),axis(2)}; %#ok<AGROW>
            m.mean(id,:)=axis(1:2);
        end
        off.columnMaps.moments=m;
        cc=cfg.coarseProbabilityCloud;cc.semanticNames=cfg.featureNames;cc.projectionRotation=p.probabilityCloud.projectionRotation;cc.projectionTranslation=p.probabilityCloud.projectionTranslation;cc.frameCalibration=p.probabilityCloud.frameCalibration;
        currentClouds{k}=buildCoarseSemanticProbabilityCloud(p.diagnostics.ground,off,cc);
        [sources{k},history]=updateLocalizationSourceWindow(currentClouds{k},calls.timeSeconds(k),odom.motion(k,:),history,wc);
        if mod(k,100)==0,fprintf('Pole axis %d/1170\n',k);end
    end
    file=fullfile(out,'poleAxis_sources.mat');save(file,'sources','currentClouds','cfg','-v7.3');
    writetable(cell2table(rows,VariableNames={'frame','pillar','wholeX','wholeY','axisX','axisY'}),fullfile(dest,'pole_axis_all.csv'));
    rootCauseReplay(file,"poleAxis",distributionRegistrationConfig());
end
