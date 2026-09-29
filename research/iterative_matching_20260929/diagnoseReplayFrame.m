function diagnoseReplayFrame(label,frame,sourceFile)
% diagnoseReplayFrame Reproduce any causal frame and isolate matching factors.
    root=setupVehicleLocalization();dest=fullfile(fileparts(mfilename('fullpath')),label+"_frame"+frame);if ~isfolder(dest),mkdir(dest);end
    b=load(fullfile('output/iterative_matching_20260929',label+".mat"));data=load(sourceFile,'sources','currentClouds');
    o=load('output/line_direction_matching_20260928/sources.mat','motion');
    p=load('output/line_direction_matching_20260928/production/report.mat','report');calls=p.report.calls;
    before=b.replay{frame-1,{'x','y','psi'}};a=o.motion(frame-1,:);c=o.motion(frame,:);R=@(z)[cos(z) -sin(z);sin(z) cos(z)];
    initial=[before(1:2)+(c(1:2)-a(1:2))*R(a(3))*R(before(3)).',before(3)+c(3)-a(3)];ref=calls{frame,{'referenceX','referenceY','referencePsi'}};
    mc=featureMapBuildConfig();map=load(mc.probabilityCloudPath,'cloud');fixed=registrationSupport.projectSemanticProbabilityCloud(map.cloud,2);
    [fixed,ids]=selectLocalProbabilityCloud(fixed,initial,b.cfg.localMapRadius);source=data.sources{frame};cfg=b.cfg;
    names=["production","reference_seed","unlimited_refinement","no_merge","single_scan","without_direction","only_curb","without_sign","without_pole","relative_height","uniform_priors"];
    results=cell(numel(names),1);rows=cell(numel(names),9);
    for k=1:numel(names)
        reg=cfg;f=fixed;m=source;seed=initial;
        switch names(k)
            case "reference_seed",seed=ref;
            case "unlimited_refinement",reg.pyramid.trustRadius=1e6;
            case "no_merge",reg.pyramid.mapMergeRadius=0;reg.pyramid.sourceMergeRadius=0;
            case "single_scan",m=data.currentClouds{frame};
            case "without_direction",reg.lineDirection.enabled=false;
            case "only_curb",m=subset(m,m.components.semanticName=="curb");
            case "without_sign",m=subset(m,m.components.semanticName~="trafficSign");
            case "without_pole",m=subset(m,m.components.semanticName~="pole");
            case "relative_height",reg.relativeHeight.enabled=true;
            case "uniform_priors",f.components.mixtureWeight(:)=1/f.components.numComponents;
        end
        r=registerSemanticProbabilityCloud(f,m,seed,reg);results{k}=r;e=r.poseXYTheta-ref;
        rows(k,:)={names(k),r.accepted,r.directionalAccepted,r.reason,norm(e(1:2)),rad2deg(atan2(sin(e(3)),cos(e(3)))),r.similarity,r.pyramid.coarseRetained,r.pyramid.refinementShiftM};
        if k==1,assert(max(abs(r.poseXYTheta-b.replay{frame,{'x','y','psi'}}))<1e-7);end
        if k==1||k==3
            pairs=r.correspondences;pairs.sourceBody=pairs.sourceMeanXY;pairs.targetReferenceBody=(pairs.targetMeanXY-ref(1:2))*R(ref(3));pairs.globalTarget=ids(pairs.target);
            writetable(pairs,fullfile(dest,names(k)+"_pairs.csv"));writetable(r.classDiagnostics,fullfile(dest,names(k)+"_classes.csv"));
        end
    end
    controls=cell2table(rows,VariableNames={'variant','accepted','directional','reason','errorM','yawErrorDeg','similarity','coarseRetained','fineShiftM'});writetable(controls,fullfile(dest,'controls.csv'));disp(controls);
    save(fullfile(root,'output/iterative_matching_20260929',label+"_frame"+frame+".mat"),'fixed','ids','source','initial','ref','results','controls','cfg');
end
function cloud=subset(cloud,keep)
    c=cloud.components;n=c.numComponents;
    for field=string(fieldnames(c)).'
        value=c.(field);
        if ndims(value)==3 && size(value,3)==n,c.(field)=value(:,:,keep);
        elseif size(value,1)==n,c.(field)=value(keep,:);end
    end
    c.numComponents=nnz(keep);cloud.components=c;
    if isfield(cloud,'heightEvidence'),cloud=rmfield(cloud,'heightEvidence');end
end
