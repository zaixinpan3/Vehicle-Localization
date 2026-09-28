function summary=diagnoseFrame601()
% diagnoseFrame601 Reconstruct the maximum-error scan and isolate its inputs.
% Reference-motion and class removals are diagnostic controls, not deployment.
    root=setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    out=fullfile(root,'output/frame601_matching_diagnosis_20260928');
    baseline=load('output/mississippi_matching_20260928/recursive/report.mat','report','cfg');
    cfg=baseline.cfg;calls=baseline.report.calls;frame=601;call=calls(frame,:);
    seed=[call.predictedX call.predictedY call.predictedPsi];ref=[call.referenceX call.referenceY call.referencePsi];
    rotation=[cos(ref(3)) -sin(ref(3));sin(ref(3)) cos(ref(3))];
    loaded=load(cfgMapPath());full=registrationSupport.projectSemanticProbabilityCloud(loaded.cloud,2);
    keep=vecnorm(full.components.mean-seed(1:2),2,2)<=100;ids=find(keep);fixed=subset(full,keep);
    mapcfg=featureMapBuildConfig();frames=580:610;
    poses=readFramePoseTable(fullfile(root,'data',mapcfg.poseMatchCsvPath),frames);
    store=matfile(fullfile(root,'data',mapcfg.pointCloudMatPath));raw=store.pointClouds(1,frames);
    history=[];refHistory=[];sources=cell(numel(frames),1);currents=sources;referenceSources=sources;confirmed=sources;
    resultsByFrame=sources;windows=sources;
    for k=1:numel(frames)
        f=frames(k);[~,tilt]=poseRowToPlanarPose(poses(k,:));cfg.perception.coarseProbabilityCloud.projectionRotation=tilt;
        currents{k}=perceiveCoarseProbabilityCloud(raw(k),cfg.perception);
        d=baseline.report.deadReckoning(f,:);c=calls(f,:);
        [sources{k},history,windows{k},confirmed{k}]=updateLocalizationSourceWindow(currents{k},c.timeSeconds,[d.x d.y d.psi],history,cfg.sourceWindow);
        [referenceSources{k},refHistory]=updateLocalizationSourceWindow(currents{k},c.timeSeconds,[c.referenceX c.referenceY c.referencePsi],refHistory,cfg.sourceWindow);
        if f>=585
            resultsByFrame{k}=registerSemanticProbabilityCloud(fixed,sources{k},[c.predictedX c.predictedY c.predictedPsi],cfg.registration);
        end
    end
    k=find(frames==frame);source=sources{k};current=currents{k};referenceSource=referenceSources{k};confirmedCurrent=confirmed{k};
    names=["production_reproduced","exact_reference_seed","single_scan","single_scan_reference_seed", ...
        "confirmed_current","reference_window_motion","without_curb","without_pole","without_sign", ...
        "only_curb","only_pole","only_sign","uniform_map_priors","no_canonical_merge","no_refinement_trust_limit"];
    results=cell(numel(names),1);results{1}=registerSemanticProbabilityCloud(fixed,source,seed,cfg.registration);
    reproduced=max(abs(results{1}.poseXYTheta-[call.x call.y call.psi]));assert(reproduced<1e-7);
    results{2}=registerSemanticProbabilityCloud(fixed,source,ref,cfg.registration);
    results{3}=registerSemanticProbabilityCloud(fixed,current,seed,cfg.registration);
    results{4}=registerSemanticProbabilityCloud(fixed,current,ref,cfg.registration);
    results{5}=registerSemanticProbabilityCloud(fixed,confirmedCurrent,seed,cfg.registration);
    results{6}=registerSemanticProbabilityCloud(fixed,referenceSource,seed,cfg.registration);
    classes=["curb","pole","trafficSign"];
    for j=1:3
        results{6+j}=registerSemanticProbabilityCloud(fixed,subset(source,source.components.semanticName~=classes(j)),seed,cfg.registration);
        results{9+j}=registerSemanticProbabilityCloud(fixed,subset(source,source.components.semanticName==classes(j)),seed,cfg.registration);
    end
    flat=fixed;flat.components.mixtureWeight(:)=1/flat.components.numComponents;
    results{13}=registerSemanticProbabilityCloud(flat,source,seed,cfg.registration);
    change=cfg.registration;change.pyramid.mapMergeRadius=0;change.pyramid.sourceMergeRadius=0;
    results{14}=registerSemanticProbabilityCloud(fixed,source,seed,change);
    change=cfg.registration;change.pyramid.trustRadius=1e6;
    results{15}=registerSemanticProbabilityCloud(fixed,source,seed,change);
    controls=score(names,results,ref);writetable(controls,fullfile(dest,'controls.csv'));
    writetable(calls(585:610,:),fullfile(dest,'neighborhood.csv'));
    writetable(results{1}.classDiagnostics,fullfile(dest,'class_diagnostics.csv'));
    pairs=results{1}.correspondences;pairs.globalTarget=ids(pairs.target);
    pairs.sourceBody=source.components.mean(pairs.source,:);
    pairs.targetBody=(fixed.components.mean(pairs.target,:)-ref(1:2))*rotation;
    pairs.sourceAtReference=transform(pairs.sourceBody,ref);
    pairs.sourceAtSolution=transform(pairs.sourceBody,results{1}.poseXYTheta);
    pairs.referenceDistanceM=vecnorm(pairs.sourceAtReference-fixed.components.mean(pairs.target,:),2,2);
    pairs.solutionDistanceM=vecnorm(pairs.sourceAtSolution-fixed.components.mean(pairs.target,:),2,2);
    writetable(pairs,fullfile(dest,'correspondences.csv'));
    rows=cell(0,8);
    for j=1:numel(frames)
        if frames(j)<585,continue;end
        r=resultsByFrame{j};c=calls(frames(j),:);s=sources{j}.components;
        rows(end+1,:)={frames(j),nnz(s.semanticName=="curb"),nnz(s.semanticName=="pole"),nnz(s.semanticName=="trafficSign"), ...
            norm(r.poseXYTheta(1:2)-[c.referenceX c.referenceY]),r.observableRank,r.reason,max(abs(r.poseXYTheta-[c.x c.y c.psi]))}; %#ok<AGROW>
    end
    neighborhood=cell2table(rows,VariableNames={'frame','curb','pole','sign','errorM','rank','reason','reproductionMaxAbs'});
    writetable(neighborhood,fullfile(dest,'source_neighborhood.csv'));
    summary=struct('frame',frame,'reproductionMaxAbs',reproduced,'seedErrorM',norm(seed(1:2)-ref(1:2)), ...
        'seedYawErrorDeg',rad2deg(wrap(seed(3)-ref(3))),'window',windows{k},'pyramid',results{1}.pyramid, ...
        'curvatureEigenvalues',results{1}.curvatureEigenvalues,'productionChanged',false);
    writeJson(fullfile(dest,'summary.json'),summary);
    save(fullfile(out,'diagnostic.mat'),'baseline','cfg','fixed','full','ids','source','current','referenceSource','confirmedCurrent', ...
        'ref','seed','frames','sources','currents','referenceSources','confirmed','resultsByFrame','results','controls','pairs','summary','-v7.3');
    disp(controls);disp(results{1}.classDiagnostics);disp(summary);
end
function path=cfgMapPath()
    c=featureMapBuildConfig();path=c.probabilityCloudPath;
end
function cloud=subset(cloud,keep)
    c=cloud.components;n=c.numComponents;
    for name=string(fieldnames(c)).'
        value=c.(name);
        if name=="covariance",c.(name)=value(:,:,keep);
        elseif size(value,1)==n,c.(name)=value(keep,:);end
    end
    c.numComponents=nnz(keep);cloud.components=c;
    if isfield(cloud,'heightEvidence')
        h=cloud.heightEvidence;h.mean=h.mean(keep,:);h.covariance=h.covariance(:,:,keep);h.available=h.available(keep);cloud.heightEvidence=h;
    end
end
function tableOut=score(names,results,ref)
    rotation=[cos(ref(3)) -sin(ref(3));sin(ref(3)) cos(ref(3))];rows=cell(numel(names),13);
    for k=1:numel(names)
        r=results{k};delta=r.poseXYTheta-ref;body=delta(1:2)*rotation;n=0;if isfield(r,'correspondences'),n=height(r.correspondences);end
        rows(k,:)={names(k),r.accepted,r.directionalAccepted,r.reason,norm(delta(1:2)),body(1),body(2), ...
            rad2deg(wrap(delta(3))),r.similarity,r.observableRank,n,r.pyramid.coarseRetained,r.pyramid.refinementShiftM};
    end
    tableOut=cell2table(rows,VariableNames={'variant','accepted','directional','reason','errorM','forwardErrorM','leftErrorM','yawErrorDeg','similarity','rank','pairs','coarseRetained','refinementShiftM'});
end
function p=transform(p,pose)
    r=[cos(pose(3)) -sin(pose(3));sin(pose(3)) cos(pose(3))];p=p*r.'+pose(1:2);
end
function x=wrap(x)
    x=atan2(sin(x),cos(x));
end
function writeJson(path,value)
    fid=fopen(path,'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(value,PrettyPrint=true));
end
