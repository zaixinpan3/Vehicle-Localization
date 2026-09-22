function summary=review_frame959()
% review_frame959 Recheck the prior pole-driven peak with the five-scan default.
% Class exclusions, stricter support and reference motion/seed are diagnostics.
    root=setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    out=fullfile(root,'output/frame959_five_frame_review_20260922');
    old=load('output/frame959_matching_diagnosis_20260922/diagnostic.mat');
    saved=load('output/temporal_perception_20260922/five_frame_matching/report.mat','cfg','report');
    cfg=saved.cfg;assert(isequaln(cfg.sourceWindow,localizationSourceWindowConfig()));
    call=saved.report.calls(959,:);seed=[call.predictedX call.predictedY call.predictedPsi];
    reference=[call.referenceX call.referenceY call.referencePsi];
    assert(max(abs(reference-old.reference))<1e-10);
    frames=955:959;mapCfg=featureMapBuildConfig();
    poses=readFramePoseTable(fullfile(root,'data',mapCfg.poseMatchCsvPath),frames);
    history=[];referenceHistory=[];
    for k=1:numel(frames)
        frame=frames(k);row=saved.report.calls(frame,:);motion=saved.report.deadReckoning(frame,:);
        [~,tilt]=poseRowToPlanarPose(poses(k,:));cfg.perception.coarseProbabilityCloud.projectionRotation=tilt;
        input=loadPointCloudFrame(fullfile(root,'data',mapCfg.pointCloudMatPath),frame);
        current=perceiveCoarseProbabilityCloud(input,cfg.perception);
        [source,history,window]=updateLocalizationSourceWindow(current,row.timeSeconds,[motion.x motion.y motion.psi],history,cfg.sourceWindow);
        [referenceSource,referenceHistory]=updateLocalizationSourceWindow(current,row.timeSeconds, ...
            [row.referenceX row.referenceY row.referencePsi],referenceHistory,cfg.sourceWindow);
    end
    crop=vecnorm(old.full.components.mean-seed(1:2),2,2)<=100;
    fixed=subset(old.full,crop);globalTargets=find(crop);
    reproducedOld=registerSemanticProbabilityCloud(old.fixed,old.source,old.seed,old.cfg.registration);
    assert(max(abs(reproducedOld.poseXYTheta-old.results{1}.poseXYTheta))<1e-7);
    labels=["five_recorded_seed","five_old_seed","without_pole","without_sign", ...
        "without_curb","only_curb","minimum_support_3","minimum_support_4", ...
        "minimum_support_5","reference_motion","reference_seed", ...
        "without_selected_sign_target","without_colocated_pole"];
    results=cell(numel(labels),1);
    results{1}=registerSemanticProbabilityCloud(fixed,source,seed,cfg.registration);
    results{2}=registerSemanticProbabilityCloud(fixed,source,old.seed,cfg.registration);
    names=source.components.semanticName;
    masks={names~="pole",names~="trafficSign",names~="curb",names=="curb", ...
        source.components.detectionFrameCount>=3,source.components.detectionFrameCount>=4, ...
        source.components.detectionFrameCount>=5};
    for k=1:numel(masks)
        results{k+2}=registerSemanticProbabilityCloud(fixed,subset(source,masks{k}),seed,cfg.registration);
    end
    results{10}=registerSemanticProbabilityCloud(fixed,referenceSource,seed,cfg.registration);
    results{11}=registerSemanticProbabilityCloud(fixed,source,reference,cfg.registration);
    selected=results{1}.correspondences;
    signTarget=selected.target(selected.semanticName=="trafficSign");assert(isscalar(signTarget));
    results{12}=registerSemanticProbabilityCloud(subset(fixed,(1:fixed.components.numComponents).'~=signTarget),source,seed,cfg.registration);
    signXY=source.components.mean(names=="trafficSign",:);assert(size(signXY,1)==1);
    colocated=names=="pole" & vecnorm(source.components.mean-signXY,2,2)<.1;
    results{13}=registerSemanticProbabilityCloud(fixed,subset(source,~colocated),seed,cfg.registration);
    reproduction=max(abs(results{1}.poseXYTheta-[call.x call.y call.psi]));assert(reproduction<1e-7);
    rows=cell(numel(labels),9);
    for k=1:numel(labels)
        r=results{k};e=r.poseXYTheta-reference;
        rows(k,:)={labels(k),r.accepted,r.directionalAccepted,r.reason,norm(e(1:2)), ...
            rad2deg(atan2(sin(e(3)),cos(e(3)))),r.observableRank,r.similarity,r.iterations};
    end
    controls=cell2table(rows,VariableNames={'variant','accepted','directionalAccepted','reason', ...
        'errorM','yawErrorDeg','rank','similarity','iterations'});
    writetable(controls,fullfile(dest,'controls.csv'));
    c=source.components;ids=(1:c.numComponents).';
    sources=table(ids,c.semanticName,c.mean(:,1),c.mean(:,2),c.detectionFrameCount, ...
        c.temporalStability,c.detectionFrameMask,VariableNames={'source','class','x','y','detections','stability','frameMask'});
    writetable(sources,fullfile(dest,'sources.csv'));
    r=results{1};pairs=r.correspondences;rotation=[cos(reference(3)) -sin(reference(3));sin(reference(3)) cos(reference(3))];
    pairs.globalTarget=globalTargets(pairs.target);
    pairs.sourceXY=c.mean(pairs.source,:);pairs.targetBodyXY=(fixed.components.mean(pairs.target,:)-reference(1:2))*rotation;
    pairs.referenceDistanceM=vecnorm(pairs.sourceXY-pairs.targetBodyXY,2,2);
    pairs.detections=c.detectionFrameCount(pairs.source);pairs.frameMask=c.detectionFrameMask(pairs.source,:);
    writetable(pairs,fullfile(dest,'correspondences.csv'));
    writetable(r.classDiagnostics,fullfile(dest,'class_diagnostics.csv'));
    alternative=results{2}.correspondences;
    ids=[signTarget;alternative.target(alternative.semanticName=="trafficSign")];
    means=(fixed.components.mean(ids,:)-reference(1:2))*rotation;
    signTargets=table(["current_seed";"old_seed"],globalTargets(ids),means(:,1),means(:,2), ...
        vecnorm(means-signXY,2,2),fixed.components.mixtureWeight(ids), ...
        VariableNames={'seed','globalTarget','bodyX','bodyY','referenceDistanceM','mapMixtureWeight'});
    writetable(signTargets,fullfile(dest,'sign_targets.csv'));
    suspectIds=[100 142];suspects=zeros(2,8);lastMotion=history.motion(end,:);
    R=[cos(lastMotion(3)) -sin(lastMotion(3));sin(lastMotion(3)) cos(lastMotion(3))];
    for q=1:2
        xy=old.source.components.mean(suspectIds(q),:);
        suspects(q,1)=suspectIds(q);suspects(q,2)=min(vecnorm(c.mean(names=="pole",:)-xy,2,2));
        suspects(q,3)=nnz(vecnorm(c.mean(names=="pole",:)-xy,2,2)<=.75);
        for k=1:5
            raw=history.clouds{k}.components;delta=history.motion(k,:)-lastMotion;
            rxy=[cos(delta(3)) -sin(delta(3));sin(delta(3)) cos(delta(3))];
            aligned=raw.mean*rxy.'+delta(1:2)*R;
            suspects(q,k+3)=nnz(raw.semanticName=="pole" & vecnorm(aligned-xy,2,2)<=.75);
        end
    end
    suspectTable=array2table(suspects,VariableNames={'oldGaussian','nearestConfirmedPoleM','confirmedPolesWithinGate', ...
        'raw955','raw956','raw957','raw958','raw959'});
    writetable(suspectTable,fullfile(dest,'old_suspects.csv'));
    neighborhood=saved.report.calls(949:969,{'frame','positionErrorM','yawErrorDeg','accepted','directionalAccepted', ...
        'predictedX','predictedY','referenceX','referenceY'});
    neighborhood.initialErrorM=hypot(neighborhood.predictedX-neighborhood.referenceX,neighborhood.predictedY-neighborhood.referenceY);
    writetable(neighborhood,fullfile(dest,'neighborhood.csv'));
    summary=struct('frame',959,'sourceFrames',frames,'window',window,'reproductionMaxAbs',reproduction, ...
        'oldErrorM',old.call.positionErrorM,'oldSeedErrorM',norm(old.seed(1:2)-reference(1:2)), ...
        'currentErrorM',call.positionErrorM,'currentSeedErrorM',norm(seed(1:2)-reference(1:2)), ...
        'oldSuspectsExcluded',all(suspects(:,3)==0),'productionChanged',false,'fullSequenceRerun',false);
    fid=fopen(fullfile(dest,'summary.json'),'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(summary,PrettyPrint=true));
    save(fullfile(out,'review.mat'),'source','fixed','seed','reference','history','results','controls','summary','pairs','cfg');
    disp(controls);disp(sources(names=="pole",:));disp(pairs(pairs.semanticName=="pole",:));disp(suspectTable);disp(summary);
end
function cloud=subset(cloud,keep)
    c=cloud.components;c.mean=c.mean(keep,:);c.covariance=c.covariance(:,:,keep);
    for field=["semanticName","mixtureWeight","repeatability","semanticProbability","occupancyProbability", ...
            "supportAmplitude","temporalStability","detectionFrameCount","detectionFrameMask"]
        if isfield(c,field),c.(field)=c.(field)(keep,:);end
    end
    c.numComponents=nnz(keep);cloud.components=c;
end
