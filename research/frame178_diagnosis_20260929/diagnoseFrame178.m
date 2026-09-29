function diagnoseFrame178()
% diagnoseFrame178 Controlled source and objective interventions at a fixed frame.
% Reference-seed and reference-motion cases are diagnostic oracles only.
    setupVehicleLocalization(); dest=fileparts(mfilename('fullpath'));
    out='output/frame178_diagnosis_20260929';
    if ~isfolder(out),mkdir(out);end
    b=load('output/matching_objective_20260929/productionAll.mat');
    d=load('output/root_cause_matching_20260929/finalSurface_sources.mat','sources','currentClouds');
    old=load('output/line_direction_matching_20260928/production/report.mat','report'); calls=old.report.calls;
    od=load('output/line_direction_matching_20260928/sources.mat','motion');
    saved=load('output/matching_objective_20260929/production_frame178.mat','initial','ref');
    initial=saved.initial; ref=saved.ref; cfg=b.cfg; cfg.pyramid.mapMergeRadius=0;
    map=load(b.mapFile,'cloud'); fixed=conditionSemanticMapOnView(map.cloud,initial);
    fixed=rmfield(fixed,'landmarkViews'); % Freeze both view model and support across seeds.
    source=strip(d.sources{178}); current=strip(d.currentClouds{178});
    history=[]; oracleHistory=[];
    for k=174:178
        [reconstructed,history,~,confirmed]=updateLocalizationSourceWindow(d.currentClouds{k},calls.timeSeconds(k),od.motion(k,:),history);
        [oracle,oracleHistory]=updateLocalizationSourceWindow(d.currentClouds{k},calls.timeSeconds(k),calls{k,{'referenceX','referenceY','referencePsi'}},oracleHistory);
    end
    assert(isequaln(reconstructed.components,source.components));
    confirmed=strip(confirmed); oracle=strip(oracle);
    names=["production","reference_seed_frozen","single_scan","single_scan_without_pole", ...
        "pool_add_pole_weak","pool_add_pole_full","confirmed_current","reference_motion_pool", ...
        "current_sign_centers","current_sign_geometry","current_curb_centers","current_curb_geometry", ...
        "pool_curb_current_sign","current_curb_pool_sign","merge_sign_05","merge_sign_10", ...
        "drop_sign_21","drop_sign_22","drop_sign_30","only_sign_21","only_sign_22","only_sign_30", ...
        "without_direction","no_pyramid_source_merge","reference_seed_dynamic", ...
        "sign_center_reference_oracle"];
    rows=cell(numel(names),13); results=cell(numel(names),1); inputs=cell(numel(names),1);
    for n=1:numel(names)
        m=source; c=cfg; seed=initial; f=fixed;
        switch names(n)
            case "reference_seed_frozen", seed=ref;
            case "single_scan", m=current;
            case "single_scan_without_pole", m=subset(current,current.components.semanticName~="pole");
            case "pool_add_pole_weak", m=append(source,subset(current,current.components.semanticName=="pole"),.1);
            case "pool_add_pole_full", m=append(source,subset(current,current.components.semanticName=="pole"),1);
            case "confirmed_current", m=confirmed;
            case "reference_motion_pool", m=oracle;
            case {"current_sign_centers","current_sign_geometry","current_curb_centers","current_curb_geometry"}
                e=d.sources{178}.heightEvidence; name="trafficSign"; if contains(names(n),"curb"),name="curb";end
                keep=e.available & source.components.semanticName==name;
                m.components.mean(keep,:)=e.mean(keep,1:2);
                if contains(names(n),"geometry"),m.components.covariance(:,:,keep)=e.covariance(1:2,1:2,keep);end
            case "pool_curb_current_sign",m=append(subset(source,source.components.semanticName=="curb"),subset(current,current.components.semanticName=="trafficSign"),1);
            case "current_curb_pool_sign",m=append(subset(current,current.components.semanticName=="curb"),subset(source,source.components.semanticName=="trafficSign"),NaN);
            case "merge_sign_05",m=canonicalizeSemanticCloud(source,.5,"trafficSign");
            case "merge_sign_10",m=canonicalizeSemanticCloud(source,1,"trafficSign");
            case {"drop_sign_21","drop_sign_22","drop_sign_30"},keep=true(source.components.numComponents,1);keep(str2double(extractAfter(names(n),"drop_sign_")))=false;m=subset(source,keep);
            case {"only_sign_21","only_sign_22","only_sign_30"},keep=source.components.semanticName~="trafficSign";keep(str2double(extractAfter(names(n),"only_sign_")))=true;m=subset(source,keep);
            case "without_direction",c.lineDirection.enabled=false;
            case "no_pyramid_source_merge",c.pyramid.sourceMergeRadius=0;
            case "reference_seed_dynamic",seed=ref;f=map.cloud;
            case "sign_center_reference_oracle"
                % Diagnostic intervention: give each partial sign the full target center.
                keep=m.components.semanticName=="trafficSign";
                target=fixed.components.mean(1074,:);m.components.mean(keep,:)=repmat((target-ref(1:2))*rot(ref(3)),nnz(keep),1);
        end
        r=matchLocalProbabilityCloud(f,m,seed,c); results{n}=r;inputs{n}=m;
        e=r.poseXYTheta-ref; body=e(1:2)*rot(ref(3));
        rows(n,:)={names(n),norm(e(1:2)),body(1),body(2),rad2deg(atan2(sin(e(3)),cos(e(3)))),r.accepted,r.directionalAccepted,r.reason,r.iterations,r.similarity,r.observableRank,r.pyramid.refinementShiftM,r.pyramid.coarseRetained};
    end
    controls=cell2table(rows,VariableNames={'variant','errorM','longitudinalM','lateralM','yawErrorDeg','accepted','directional','reason','iterations','similarity','rank','refinementShiftM','coarseRetained'});
    assert(norm(results{1}.poseXYTheta-b.replay{178,{'x','y','psi'}})<1e-7);
    writetable(controls,fullfile(dest,'controls.csv'));disp(controls);
    local=selectLocalProbabilityCloud(fixed,initial,cfg.localMapRadius);
    model=prepareSemanticRegistrationGeometry(local,source,initial,cfg);scale=[1;1;.1];
    poses=[initial;ref;results{1}.poseXYTheta;results{2}.poseXYTheta;results{3}.poseXYTheta];
    labels=["prediction","reference","production","reference_seed_frozen","single_scan"];
    objective=cell(numel(labels),10);systems=cell(numel(labels),1);
    for n=1:numel(labels)
        pose=[poses(n,1:2)-initial(1:2),poses(n,3)];s=model.linearize(pose,scale);systems{n}=s;
        [v,l]=eig(s.H,'vector');[l,order]=sort(l);v=v(:,order);body=v(1:2,1).'*rot(ref(3));
        objective(n,:)={labels(n),s.cost,s.similarity,norm(s.gradient),l(1),l(2),l(3),body(1),body(2),v(3,1)};
        pairs=struct2table(s.pairs);pairs.targetMeanXY=pairs.targetMeanXY+initial(1:2);
        pairs.targetBody=(pairs.targetMeanXY-ref(1:2))*rot(ref(3));
        pairs.referenceDelta=pairs.sourceMeanXY-pairs.targetBody;
        writetable(pairs,fullfile(dest,labels(n)+"_pairs.csv"));
    end
    objectives=cell2table(objective,VariableNames={'pose','cost','similarity','gradientNorm','lambda1','lambda2','lambda3','weakLongitudinal','weakLateral','weakScaledYaw'});
    writetable(objectives,fullfile(dest,'objectives.csv'));disp(objectives);
    c=source.components;e=d.sources{178}.heightEvidence;
    summary=table((1:c.numComponents).',c.semanticName,c.mean,c.detectionFrameCount,c.detectionFrameMask,e.available,e.mean(:,1:2),VariableNames={'id','class','mean','frames','support','currentAvailable','currentMean'});
    writetable(summary,fullfile(dest,'source_tracks.csv'));
    neighbor=b.replay(170:185,:);refs=calls{170:185,{'referenceX','referenceY','referencePsi'}};
    for k=1:height(neighbor),body=(neighbor{k,{'x','y'}}-refs(k,1:2))*rot(refs(k,3));neighbor.longitudinalM(k)=body(1);neighbor.lateralM(k)=body(2);end
    writetable(neighbor,fullfile(dest,'neighbor_frames.csv'));
    save(fullfile(out,'diagnostic.mat'),'initial','ref','cfg','fixed','source','current','confirmed','oracle','results','inputs','controls','objectives','systems','history','oracleHistory','-v7.3');
end
function cloud=strip(cloud)
    if isfield(cloud,'heightEvidence'),cloud=rmfield(cloud,'heightEvidence');end
end
function cloud=subset(cloud,keep)
    c=cloud.components;n=c.numComponents;
    for field=string(fieldnames(c)).'
        value=c.(field);
        if ndims(value)==3 && size(value,3)==n,c.(field)=value(:,:,keep);
        elseif size(value,1)==n,c.(field)=value(keep,:);end
    end
    c.numComponents=nnz(keep);cloud.components=c;
end
function cloud=append(a,b,stability)
    % Only public registration fields are needed for these diagnostic mixtures.
    ca=a.components;cb=b.components;cloud=a;cloud.components=struct();
    fields=["mean","semanticName","semanticProbability","occupancyProbability"];
    for f=fields,cloud.components.(f)=[ca.(f);cb.(f)];end
    cloud.components.covariance=cat(3,ca.covariance,cb.covariance);
    sa=ones(ca.numComponents,1);sb=ones(cb.numComponents,1);
    if isfield(ca,'temporalStability'),sa=ca.temporalStability;end
    if isfield(cb,'temporalStability'),sb=cb.temporalStability;end
    if isfinite(stability),sb(:)=stability;end
    cloud.components.temporalStability=[sa;sb];cloud.components.numComponents=ca.numComponents+cb.numComponents;
    mass=cloud.components.semanticProbability.*cloud.components.occupancyProbability.*[sa;sb];
    cloud.components.mixtureWeight=mass/sum(mass);
    for f=["detectionFrameCount","detectionFrameMask"]
        if isfield(cloud.components,f),cloud.components=rmfield(cloud.components,f);end
    end
end
function r=rot(a)
    r=[cos(a) -sin(a);sin(a) cos(a)];
end
