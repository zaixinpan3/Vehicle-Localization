function diagnoseNewMaximum(frame)
% diagnoseProductionFrame Isolate factors at any final causal replay frame.
    setupVehicleLocalization();out='output/partial_sign_matching_20260929';dest=fullfile(fileparts(mfilename('fullpath')),"production_frame"+frame);if ~isfolder(dest),mkdir(dest);end
    b=load(fullfile(out,'production.mat'));data=load('output/root_cause_matching_20260929/finalSurface_sources.mat','sources','currentClouds');
    o=load('output/line_direction_matching_20260928/sources.mat','motion');p=load('output/line_direction_matching_20260928/production/report.mat','report');calls=p.report.calls;
    before=b.replay{frame-1,{'x','y','psi'}};a=o.motion(frame-1,:);c=o.motion(frame,:);R=@(z)[cos(z) -sin(z);sin(z) cos(z)];
    initial=[before(1:2)+(c(1:2)-a(1:2))*R(a(3))*R(before(3)).',before(3)+c(3)-a(3)];ref=calls{frame,{'referenceX','referenceY','referencePsi'}};
    map=load(b.mapFile,'cloud');source=data.sources{frame};cfg=b.cfg;
    names=["production","reference_seed","single_scan","without_direction","only_curb","without_sign","without_pole","static_map","seed_x_minus1","seed_x_plus1","seed_y_minus1","seed_y_plus1"];
    results=cell(numel(names),1);rows=cell(numel(names),8);
    for k=1:numel(names)
        reg=cfg;m=source;seed=initial;fixed=map.cloud;
        switch names(k)
            case "reference_seed",seed=ref;
            case "single_scan",m=data.currentClouds{frame};
            case "without_direction",reg.lineDirection.enabled=false;
            case "only_curb",m=subset(m,m.components.semanticName=="curb");
            case "without_sign",m=subset(m,m.components.semanticName~="trafficSign");
            case "without_pole",m=subset(m,m.components.semanticName~="pole");
            case "static_map",fixed=rmfield(fixed,'landmarkViews');reg.pyramid.mapMergeRadius=0;
            case "seed_x_minus1",seed(1)=seed(1)-1;
            case "seed_x_plus1",seed(1)=seed(1)+1;
            case "seed_y_minus1",seed(2)=seed(2)-1;
            case "seed_y_plus1",seed(2)=seed(2)+1;
        end
        r=matchLocalProbabilityCloud(fixed,m,seed,reg);results{k}=r;e=r.poseXYTheta-ref;
        rows(k,:)={names(k),r.accepted,r.directionalAccepted,r.reason,norm(e(1:2)),rad2deg(atan2(sin(e(3)),cos(e(3)))),r.similarity,r.observableRank};
        if k==1
            assert(max(abs(r.poseXYTheta-b.replay{frame,{'x','y','psi'}}))<1e-7);
            pairs=r.correspondences;pairs.targetReferenceBody=(pairs.targetMeanXY-ref(1:2))*R(ref(3));
            conditioned=conditionSemanticMapOnView(map.cloud,initial);pairs.mapViewReliability=conditioned.components.viewReliability(pairs.globalTarget);
            writetable(pairs,fullfile(dest,'pairs.csv'));writetable(r.classDiagnostics,fullfile(dest,'classes.csv'));
        end
    end
    controls=cell2table(rows,VariableNames={'variant','accepted','directional','reason','errorM','yawErrorDeg','similarity','rank'});writetable(controls,fullfile(dest,'controls.csv'));disp(controls);
    save(fullfile(out,"production_frame"+frame+".mat"),'initial','ref','results','controls','cfg');
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
