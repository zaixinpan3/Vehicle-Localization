function summary=diagnose_frame()
% diagnose_frame Reproduce frame 959 and isolate registration inputs.
% Reference-seeded/fine/motion controls are diagnostics, not online inputs.
    root=setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    out=fullfile(root,'output/frame959_matching_diagnosis_20260922');
    if ~isfolder(out),mkdir(out);end
    matching=load('output/lidar_origin_20260922/coarse_pipeline/matching/report.mat','report','cfg');
    frame=959;call=matching.report.calls(frame,:);cfg=matching.cfg;
    seed=[call.predictedX,call.predictedY,call.predictedPsi];reference=[call.referenceX,call.referenceY,call.referencePsi];
    rotation=[cos(reference(3)),-sin(reference(3));sin(reference(3)),cos(reference(3))];
    loaded=load('output/mississippi_mapping_calibrated/probability_cloud.mat','cloud');
    full=registrationSupport.projectSemanticProbabilityCloud(loaded.cloud,2);
    globalTargets=find(vecnorm(full.components.mean-seed(1:2),2,2)<=100);
    fixed=subset(full,vecnorm(full.components.mean-seed(1:2),2,2)<=100);
    frames=957:959;mapCfg=featureMapBuildConfig();
    poses=readFramePoseTable(fullfile(root,'data',mapCfg.poseMatchCsvPath),frames);history=[];referenceHistory=[];
    for k=1:numel(frames)
        f=frames(k);input=loadPointCloudFrame(fullfile(root,'data',mapCfg.pointCloudMatPath),f);
        [~,tilt]=poseRowToPlanarPose(poses(k,:));cfg.perception.coarseProbabilityCloud.projectionRotation=tilt;
        current=perceiveCoarseProbabilityCloud(input,cfg.perception);
        d=matching.report.deadReckoning(f,:);c=matching.report.calls(f,:);
        [source,history,window]=updateLocalizationSourceWindow(current,c.timeSeconds,[d.x,d.y,d.psi],history,cfg.sourceWindow);
        [referenceSource,referenceHistory]=updateLocalizationSourceWindow(current,c.timeSeconds, ...
            [c.referenceX,c.referenceY,c.referencePsi],referenceHistory,cfg.sourceWindow);
    end
    names=["reproduced_recorded_seed","reference_seed","single_recorded_seed","single_reference_seed", ...
        "fine_recorded_seed","fine_reference_seed","without_curb","without_pole","without_sign", ...
        "only_curb","only_pole","only_sign","uniform_map_priors","uniform_reference_seed", ...
        "distance_gate_1m","distance_gate_0p5m","reference_window_motion"];
    results=cell(numel(names),1);
    results{1}=registerSemanticProbabilityCloud(fixed,source,seed,cfg.registration);
    reproduction=max(abs(results{1}.poseXYTheta-[call.x,call.y,call.psi]));assert(reproduction<1e-7);
    results{2}=registerSemanticProbabilityCloud(fixed,source,reference,cfg.registration);
    results{3}=registerSemanticProbabilityCloud(fixed,current,seed,cfg.registration);
    results{4}=registerSemanticProbabilityCloud(fixed,current,reference,cfg.registration);
    data=load('output/mississippi_mapping_calibrated/feature_observations.mat','featureData');
    adapter=load('output/saved_perception_inspva_20260915/experiment.mat','pcfg');
    fine=buildSavedFeatureProbabilityCloud(data.featureData,frame,reference,adapter.pcfg);
    results{5}=registerSemanticProbabilityCloud(fixed,fine,seed,cfg.registration);
    results{6}=registerSemanticProbabilityCloud(fixed,fine,reference,cfg.registration);
    classes=["curb","pole","trafficSign"];
    for k=1:3
        results{6+k}=registerSemanticProbabilityCloud(fixed,subset(source,source.components.semanticName~=classes(k)),seed,cfg.registration);
        results{9+k}=registerSemanticProbabilityCloud(fixed,subset(source,source.components.semanticName==classes(k)),seed,cfg.registration);
    end
    flat=fixed;flat.components.mixtureWeight(:)=1/flat.components.numComponents;
    results{13}=registerSemanticProbabilityCloud(flat,source,seed,cfg.registration);
    results{14}=registerSemanticProbabilityCloud(flat,source,reference,cfg.registration);
    for k=1:2
        gated=cfg.registration;gated.geometric.maximumMatchDistance=1/k;
        results{14+k}=registerSemanticProbabilityCloud(fixed,source,seed,gated);
    end
    results{17}=registerSemanticProbabilityCloud(fixed,referenceSource,seed,cfg.registration);
    rows=cell(numel(names),12);
    for k=1:numel(names)
        r=results{k};e=r.poseXYTheta-reference;e(3)=wrap(e(3));body=e(1:2)*rotation;
        rows(k,:)={names(k),r.accepted,r.reason,norm(e(1:2)),body(1),body(2),rad2deg(e(3)), ...
            r.similarity,r.initialSimilarity,r.observableRank,r.iterations,pairCount(r)};
    end
    controls=cell2table(rows,VariableNames={'variant','accepted','reason','errorM','forwardErrorM','leftErrorM', ...
        'yawErrorDeg','similarity','initialSimilarity','rank','iterations','pairs'});
    writetable(controls,fullfile(dest,'controls.csv'));
    writetable(results{1}.classDiagnostics,fullfile(dest,'class_diagnostics.csv'));
    neighborhood=matching.report.calls(949:969,:);writetable(neighborhood,fullfile(dest,'neighborhood.csv'));
    pairs=results{1}.correspondences;pairs.globalTarget=globalTargets(pairs.target);
    pairs.sourceX=source.components.mean(pairs.source,1);pairs.sourceY=source.components.mean(pairs.source,2);
    pairs.targetBodyXY=(fixed.components.mean(pairs.target,:)-reference(1:2))*rotation;
    pairs.sourceAtReference=transform(source.components.mean(pairs.source,:),reference);
    pairs.sourceAtSolution=transform(source.components.mean(pairs.source,:),results{1}.poseXYTheta);
    pairs.targetWorldXY=fixed.components.mean(pairs.target,:);
    pairs.referenceDistanceM=vecnorm(pairs.sourceAtReference-pairs.targetWorldXY,2,2);
    pairs.solutionDistanceM=vecnorm(pairs.sourceAtSolution-pairs.targetWorldXY,2,2);
    writetable(pairs,fullfile(dest,'correspondences.csv'));
    classRows=cell(3,6);
    for k=1:3
        use=pairs.semanticName==classes(k);
        classRows(k,:)={classes(k),nnz(source.components.semanticName==classes(k)),nnz(use), ...
            numel(unique(pairs.target(use))),sum(pairs.weight(use)),sum(pairs.robustWeight(use))};
    end
    classSummary=cell2table(classRows,VariableNames={'class','sourceComponents','pairs','uniqueTargets','totalWeight','robustWeight'});
    writetable(classSummary,fullfile(dest,'class_summary.csv'));
    summary=struct('frame',frame,'positionErrorM',call.positionErrorM,'initialErrorM',norm(seed(1:2)-reference(1:2)), ...
        'initialYawErrorDeg',rad2deg(wrap(seed(3)-reference(3))),'yawErrorDeg',call.yawErrorDeg, ...
        'reproductionMaxAbs',reproduction,'sourceWindow',window,'informationEigenvaluesScaled',results{1}.curvatureEigenvalues, ...
        'maximumClassCorrection',cfg.registration.geometric.maximumClassCorrection,'minimumSimilarity',cfg.registration.minimumSimilarity, ...
        'calibration',cfg.perception.frameCalibration,'productionChanged',false);
    writeJson(fullfile(dest,'summary.json'),summary);
    save(fullfile(out,'diagnostic.mat'),'results','source','current','referenceSource','fixed','full','fine','cfg', ...
        'call','seed','reference','history','pairs','globalTargets','controls','classSummary','summary','-v7.3');
    disp(controls);disp(classSummary);disp(results{1}.classDiagnostics);
end
function cloud=subset(cloud,keep)
    c=cloud.components;c.mean=c.mean(keep,:);c.covariance=c.covariance(:,:,keep);
    for field=["semanticName","mixtureWeight","repeatability","semanticProbability","occupancyProbability","supportAmplitude"]
        if isfield(c,field),c.(field)=c.(field)(keep,:);end
    end
    c.numComponents=nnz(keep);cloud.components=c;
end
function n=pairCount(r)
    n=0;if isfield(r,'correspondences'),n=height(r.correspondences);end
end
function points=transform(points,pose)
    R=[cos(pose(3)),-sin(pose(3));sin(pose(3)),cos(pose(3))];points=points*R.'+pose(1:2);
end
function value=wrap(value)
    value=atan2(sin(value),cos(value));
end
function writeJson(file,value)
    fid=fopen(file,'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(value,PrettyPrint=true));
end
