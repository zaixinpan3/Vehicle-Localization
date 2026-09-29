function diagnoseViewFrame(label,frame)
% diagnoseViewFrame Reproduce a conditional-map frame and isolate its factors.
    setupVehicleLocalization();out='output/matching_objective_20260929';dest=fullfile(fileparts(mfilename('fullpath')),label+"_frame"+frame);if ~isfolder(dest),mkdir(dest);end
    b=load(fullfile(out,label+".mat"));data=load('output/root_cause_matching_20260929/finalSurface_sources.mat','sources','currentClouds');
    o=load('output/line_direction_matching_20260928/sources.mat','motion');p=load('output/line_direction_matching_20260928/production/report.mat','report');calls=p.report.calls;
    before=b.replay{frame-1,{'x','y','psi'}};a=o.motion(frame-1,:);c=o.motion(frame,:);R=@(z)[cos(z) -sin(z);sin(z) cos(z)];
    initial=[before(1:2)+(c(1:2)-a(1:2))*R(a(3))*R(before(3)).',before(3)+c(3)-a(3)];ref=calls{frame,{'referenceX','referenceY','referencePsi'}};
    map=load(b.mapFile,'cloud','viewModel');model=map.viewModel;if label=="viewMap3",model.bandwidth=3;end
    source=data.sources{frame};cfg=b.cfg;names=["production","reference_seed","single_scan","without_direction","only_curb","without_sign","without_pole","hard_fine","uniform_priors","seed_x_minus1","seed_x_plus1","seed_y_minus1","seed_y_plus1"];
    results=cell(numel(names),1);rows=cell(numel(names),8);
    for k=1:numel(names)
        reg=cfg;m=source;seed=initial;
        switch names(k)
            case "hard_fine",reg=rmfield(reg,'softPointAssociation');
            case "reference_seed",seed=ref;
            case "single_scan",m=data.currentClouds{frame};
            case "without_direction",reg.lineDirection.enabled=false;
            case "only_curb",m=subset(m,m.components.semanticName=="curb");
            case "without_sign",m=subset(m,m.components.semanticName~="trafficSign");
            case "without_pole",m=subset(m,m.components.semanticName~="pole");
            case "seed_x_minus1",seed(1)=seed(1)-1;
            case "seed_x_plus1",seed(1)=seed(1)+1;
            case "seed_y_minus1",seed(2)=seed(2)-1;
            case "seed_y_plus1",seed(2)=seed(2)+1;
        end
        fixed=conditionMapOnAcquisition(map.cloud,seed,model);[fixed,ids]=selectLocalProbabilityCloud(fixed,seed,cfg.localMapRadius);
        if names(k)=="uniform_priors",fixed.components.mixtureWeight(:)=1/fixed.components.numComponents;end
        r=registerSemanticProbabilityCloud(fixed,m,seed,reg);results{k}=r;e=r.poseXYTheta-ref;
        rows(k,:)={names(k),r.accepted,r.directionalAccepted,r.reason,norm(e(1:2)),rad2deg(atan2(sin(e(3)),cos(e(3)))),r.similarity,r.pyramid.coarseRetained};
        if k==1
            assert(max(abs(r.poseXYTheta-b.replay{frame,{'x','y','psi'}}))<1e-7);
            pairs=r.correspondences;pairs.targetReferenceBody=(pairs.targetMeanXY-ref(1:2))*R(ref(3));pairs.globalTarget=ids(pairs.target);
            writetable(pairs,fullfile(dest,'pairs.csv'));writetable(r.classDiagnostics,fullfile(dest,'classes.csv'));
        end
    end
    controls=cell2table(rows,VariableNames={'variant','accepted','directional','reason','errorM','yawErrorDeg','similarity','coarseRetained'});writetable(controls,fullfile(dest,'controls.csv'));disp(controls);
    save(fullfile(out,label+"_frame"+frame+".mat"),'initial','ref','results','controls','cfg');
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
