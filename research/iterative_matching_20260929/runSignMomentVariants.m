function runSignMomentVariants()
% runSignMomentVariants Test intensity-weighted whole-pillar moments only.
% Every return retains positive weight. Masks, grid and hit counts are fixed.
    root=setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));out='output/iterative_matching_20260929';
    b=load('output/pole_boundary_recovery_20260929/replay.mat','currentClouds');clouds=repmat({b.currentClouds},3,1);
    saved=load(fullfile(out,'softNeighborhood2.mat'),'cfg');reg=saved.cfg;
    old=load('output/line_direction_matching_20260928/production/report.mat','report');calls=old.report.calls;
    odom=load('output/line_direction_matching_20260928/sources.mat','motion');mc=featureMapBuildConfig();
    poses=readFramePoseTable(fullfile(root,'data',mc.poseMatchCsvPath),1:1170);store=matfile(fullfile(root,'data',mc.pointCloudMatPath));
    cfg=perceptionConfig('Mississippi');cfg.coarseProbabilityCloud.storeDiagnostics=true;floors=[.5 .25 .05];first=0;frames=[];
    for k=1:1170
        if isempty(frames)||k>=first+numel(frames),first=k;frames=store.pointClouds(1,k:min(1170,k+49));end
        [~,tilt]=poseRowToPlanarPose(poses(k,:));cfg.coarseProbabilityCloud.projectionRotation=tilt;
        p=perceiveFrame(frames(k-first+1),cfg);g=p.diagnostics.ground;off=p.diagnostics.offGround;ctx=p.diagnostics.offGroundPointContext;
        selected=find(off.trafficSignCellMask);cc=cfg.coarseProbabilityCloud;cc.semanticNames=cfg.featureNames;
        cc.projectionRotation=p.probabilityCloud.projectionRotation;cc.projectionTranslation=p.probabilityCloud.projectionTranslation;
        cc.frameCalibration=p.probabilityCloud.frameCalibration;
        for j=1:3
            adjusted=off;mom=off.columnMaps.trafficSignMoments;
            for id=selected.'
                keep=ctx.pointPillarLinIdx==id;xyz=double(ctx.points(keep,:))*cc.projectionRotation.'+cc.projectionTranslation;
                bright=double(ctx.pointAttributes.intensity(keep))>cfg.offGroundFeatures.trafficSignIntensityThreshold;
                w=floors(j)+(1-floors(j))*bright;w=w/sum(w);mu=sum(xyz.*w,1);delta=xyz-mu;cv=delta.'*(delta.*w);
                mom.mean(id,:)=mu(1:2);mom.meanZ(id)=mu(3);mom.covariance(id,:)=[cv(1,1),cv(1,2),cv(2,2)];
                mom.heightCovariance(id,:)=[cv(1,3),cv(2,3),cv(3,3)];
            end
            adjusted.columnMaps.trafficSignMoments=mom;clouds{j}{k}=buildCoarseSemanticProbabilityCloud(g,adjusted,cc);
        end
        if mod(k,100)==0,fprintf('Weighted sign moments %d/1170\n',k);end
    end
    summaries=table();wc=localizationSourceWindowConfig();
    for j=1:3
        currentClouds=clouds{j};sources=cell(1170,1);history=[];
        for k=1:1170,[sources{k},history]=updateLocalizationSourceWindow(currentClouds{k},calls.timeSeconds(k),odom.motion(k,:),history,wc);end
        label="signMoments"+j;file=fullfile(out,label+"_sources.mat");save(file,'sources','currentClouds','floors','j','-v7.3');
        row=replayCloudVariant(file,label,reg);summaries=[summaries;row]; %#ok<AGROW>
        writetable(summaries,fullfile(dest,'sign_moment_screen.csv'));
    end
end
