function diagnoseRoundOne()
% diagnoseRoundOne Isolate the deployed post-startup maximum matching error.
% Oracle reference seeds and class removals are diagnostic controls only.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    out='output/iterative_matching_20260929/round1';
    s=load('output/line_direction_matching_20260928/sources.mat');
    mapCfg=featureMapBuildConfig();mapData=load(mapCfg.probabilityCloudPath,'cloud');
    s.fixed=registrationSupport.projectSemanticProbabilityCloud(mapData.cloud,2);
    latest=load('output/pole_boundary_recovery_20260929/replay.mat');m=latest.maximum;
    cfg=distributionRegistrationConfig();frame=m.frame;
    seed=m.predicted;ref=m.reference;expected=m.result.poseXYTheta;
    R=[cos(ref(3)) -sin(ref(3));sin(ref(3)) cos(ref(3))];
    [fixed,ids]=selectLocalProbabilityCloud(s.fixed,seed,cfg.localMapRadius);
    source=m.source;current=m.current;
    names=["production","reference_seed","unlimited_refinement","no_map_merge","no_source_merge", ...
        "no_merge","without_direction","single_scan","without_curb","without_pole","without_sign", ...
        "only_curb","only_pole","only_sign","uniform_map_priors"];
    results=cell(numel(names),1);
    for k=1:numel(names)
        reg=cfg;f=fixed;m=source;initial=seed;
        switch names(k)
            case "reference_seed",initial=ref;
            case "unlimited_refinement",reg.pyramid.trustRadius=1e6;
            case "no_map_merge",reg.pyramid.mapMergeRadius=0;
            case "no_source_merge",reg.pyramid.sourceMergeRadius=0;
            case "no_merge",reg.pyramid.mapMergeRadius=0;reg.pyramid.sourceMergeRadius=0;
            case "without_direction",reg.lineDirection.enabled=false;
            case "single_scan",m=current;
            case "without_curb",m=subset(m,m.components.semanticName~="curb");
            case "without_pole",m=subset(m,m.components.semanticName~="pole");
            case "without_sign",m=subset(m,m.components.semanticName~="trafficSign");
            case "only_curb",m=subset(m,m.components.semanticName=="curb");
            case "only_pole",m=subset(m,m.components.semanticName=="pole");
            case "only_sign",m=subset(m,m.components.semanticName=="trafficSign");
            case "uniform_map_priors",f.components.mixtureWeight(:)=1/f.components.numComponents;
        end
        results{k}=registerSemanticProbabilityCloud(f,m,initial,reg);
    end
    reproduction=max(abs(results{1}.poseXYTheta-expected));assert(reproduction<1e-7);
    rows=cell(numel(names),12);
    for k=1:numel(names)
        r=results{k};delta=r.poseXYTheta-ref;body=delta(1:2)*R;
        rows(k,:)={names(k),r.accepted,r.directionalAccepted,r.reason,norm(delta(1:2)),body(1),body(2), ...
            rad2deg(atan2(sin(delta(3)),cos(delta(3)))),r.observableRank,r.similarity,r.pyramid.coarseRetained,r.pyramid.refinementShiftM};
    end
    controls=cell2table(rows,VariableNames={'variant','accepted','directional','reason','errorM','forwardErrorM','leftErrorM','yawErrorDeg','rank','similarity','coarseRetained','refinementShiftM'});
    writetable(controls,fullfile(dest,'controls.csv'));disp(controls);
    writetable(latest.replay(max(1,frame-7):min(height(latest.replay),frame+8),:),fullfile(dest,'neighborhood.csv'));
    writetable(results{1}.classDiagnostics,fullfile(dest,'class_diagnostics.csv'));
    [coarseFixed,mapGroups]=canonicalizeSemanticCloud(fixed,cfg.pyramid.mapMergeRadius,cfg.pyramid.pointClasses);
    [coarseSource,sourceGroups]=canonicalizeSemanticCloud(source,cfg.pyramid.sourceMergeRadius,cfg.pyramid.pointClasses);
    assert(results{1}.pyramid.coarseRetained,'This export expects retained coarse geometry.');
    % Recompute coarse associations against actual merged means. Returned
    % target representatives alone do not describe the merged geometry.
    model=prepareSemanticRegistrationGeometry(coarseFixed,coarseSource,seed,cfg);
    scale=[1;1;1/cfg.yawLeverArm];pose=results{1}.poseXYTheta;
    sys=model.linearize([pose(1:2)-seed(1:2) pose(3)],scale);
    pairs=struct2table(sys.pairs);pairs.sourceBody=coarseSource.components.mean(pairs.source,:);
    pairs.targetReferenceBody=(coarseFixed.components.mean(pairs.target,:)-ref(1:2))*R;
    pairs.sourceAtSolutionBody=(transform(pairs.sourceBody,pose)-ref(1:2))*R;
    pairs.mapMembers=cellfun(@(g)strjoin(string(ids(g)),";"),mapGroups(pairs.target));
    pairs.sourceMembers=cellfun(@(g)strjoin(string(g),";"),sourceGroups(pairs.source));
    pairs.referenceCenterDistance=vecnorm(pairs.sourceBody-pairs.targetReferenceBody,2,2);
    pairs.solutionCenterDistance=vecnorm(pairs.sourceAtSolutionBody-pairs.targetReferenceBody,2,2);
    writetable(pairs,fullfile(dest,'coarse_pairs.csv'));
    q=coarseFixed.components;mr=cell(q.numComponents,6);
    for k=1:q.numComponents
        members=mapGroups{k};xy=(q.mean(k,:)-ref(1:2))*R;
        mr(k,:)={k,q.semanticName(k),xy(1),xy(2),numel(members),strjoin(string(ids(members)),";")};
    end
    writetable(cell2table(mr,VariableNames={'index','class','x','y','memberCount','globalMembers'}),fullfile(dest,'canonical_map.csv'));
    q=fixed.components;xy=(q.mean-ref(1:2))*R;
    mapTable=table(ids,string(q.semanticName),xy(:,1),xy(:,2),q.mixtureWeight,VariableNames={'globalId','class','x','y','weight'});
    writetable(mapTable,fullfile(dest,'original_map.csv'));
    costRows=cell(0,6);poses=[seed;ref;results{1}.poseXYTheta;results{3}.poseXYTheta];poseNames=["prediction","reference","coarse_output","fine_unlimited"];
    for level=1:2
        f=fixed;m=source;if level==1,f=coarseFixed;m=coarseSource;end
        mdl=prepareSemanticRegistrationGeometry(f,m,seed,cfg);
        for j=1:4
            a=poses(j,:);st=mdl.linearize([a(1:2)-seed(1:2) a(3)],scale);
            for name=unique(st.pairs.semanticName).'
                sel=st.pairs.semanticName==name;u=cfg.geometric.robustStandardizedDistance;
                costRows(end+1,:)={level,poseNames(j),name,nnz(sel),sum(st.weights(sel)),sum(st.weights(sel)*u^2.*log1p(st.pairs.squaredStandardizedResidual(sel)/u^2))}; %#ok<AGROW>
            end
        end
    end
    writetable(cell2table(costRows,VariableNames={'levelCoarse1Fine2','pose','class','pairs','weight','cost'}),fullfile(dest,'class_costs.csv'));
    delta=seed-ref;body=delta(1:2)*R;
    summary=struct('frame',frame,'reproductionMaxAbs',reproduction,'seedErrorM',norm(delta(1:2)), ...
        'seedBodyError',body,'seedYawErrorDeg',rad2deg(atan2(sin(delta(3)),cos(delta(3)))), ...
        'pyramid',results{1}.pyramid,'sourceComponents',source.components.numComponents, ...
        'canonicalSourceComponents',coarseSource.components.numComponents,'curvatureEigenvalues',results{1}.curvatureEigenvalues, ...
        'productionChanged',false,'referenceUsedOnlyForDiagnostics',true);
    fid=fopen(fullfile(dest,'summary.json'),'w');fprintf(fid,'%s\n',jsonencode(summary,PrettyPrint=true));fclose(fid);
    save(fullfile(out,'diagnostic.mat'),'cfg','fixed','ids','source','current','seed','ref','results','controls','pairs','summary','coarseFixed','coarseSource','mapGroups','sourceGroups','-v7.3');
    disp(summary);fprintf('MAXIMUM_DIAGNOSIS_COMPLETED\n');
end
function cloud=subset(cloud,keep)
    c=cloud.components;n=c.numComponents;
    for field=string(fieldnames(c)).'
        value=c.(field);
        if field=="covariance",c.(field)=value(:,:,keep);
        elseif size(value,1)==n,c.(field)=value(keep,:);end
    end
    c.numComponents=nnz(keep);cloud.components=c;
    if isfield(cloud,'heightEvidence'),cloud=rmfield(cloud,'heightEvidence');end
end
function xy=transform(xy,pose)
    r=[cos(pose(3)) -sin(pose(3));sin(pose(3)) cos(pose(3))];xy=xy*r.'+pose(1:2);
end
