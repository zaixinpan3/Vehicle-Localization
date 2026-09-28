function inspectGeometry601()
% inspectGeometry601 Compare frozen fine point geometry and class forces.
    setupVehicleLocalization();addpath('research/fine_matching_20260919');
    dest=fileparts(mfilename('fullpath'));out='output/frame601_matching_diagnosis_20260928';s=load(fullfile(out,'diagnostic.mat'));
    mapcfg=featureMapBuildConfig();frames=597:601;poses=readFramePoseTable(fullfile('data',mapcfg.poseMatchCsvPath),frames);
    store=matfile(fullfile('data',mapcfg.pointCloudMatPath));raw=store.pointClouds(1,frames);history=[];fineClouds=cell(5,1);
    for worker=1:4
        p=load(sprintf('output/fine_matching_20260919/inputs_%d.mat',worker),'frames','selectedIndices','cfg');
        if all(ismember(frames,p.frames)),break;end
    end
    assert(all(ismember(frames,p.frames)));fineIndices=cell(5,3);classes=["curb","pole","trafficSign"];
    for k=1:5
        at=p.frames==frames(k);assert(nnz(at)==1);masks=struct();
        for j=1:3
            ids=p.selectedIndices{at,string(p.cfg.featureNames)==classes(j)};fineIndices{k,j}=ids;
            masks.(classes(j))=false(numel(raw(k).x),1);masks.(classes(j))(ids)=true;
        end
        cfg=s.cfg.perception;[~,tilt]=poseRowToPlanarPose(poses(k,:));cfg.coarseProbabilityCloud.projectionRotation=tilt;
        fineClouds{k}=buildFineMatchingCloud(raw(k),masks,cfg);
        d=s.baseline.report.deadReckoning(frames(k),:);c=s.baseline.report.calls(frames(k),:);
        [fine,history]=updateLocalizationSourceWindow(fineClouds{k},c.timeSeconds,[d.x d.y d.psi],history,s.cfg.sourceWindow);
    end
    names=["fine_window_all","fine_window_curb_only_replacement","fine_window_pole_only_replacement", ...
        "fine_single_scan","curb_only_exact_seed","pole_center_at_map_oracle","without_semantic_precision"];
    fineCurb=replaceClass(s.source,fine,"curb");finePole=replaceClass(s.source,fine,"pole");
    oracle=s.source;id=find(oracle.components.semanticName=="pole");assert(isscalar(id));
    target=s.results{1}.correspondences.target(s.results{1}.correspondences.semanticName=="pole");
    R=[cos(s.ref(3)) -sin(s.ref(3));sin(s.ref(3)) cos(s.ref(3))];oracle.components.mean(id,:)=(s.fixed.components.mean(target,:)-s.ref(1:2))*R;
    configs=s.cfg.perception;configs.semanticPrecision.enabled=false;history=[];
    for k=1:5
        [~,tilt]=poseRowToPlanarPose(poses(k,:));configs.coarseProbabilityCloud.projectionRotation=tilt;
        cloud=perceiveCoarseProbabilityCloud(raw(k),configs);d=s.baseline.report.deadReckoning(frames(k),:);c=s.baseline.report.calls(frames(k),:);
        [unfiltered,history]=updateLocalizationSourceWindow(cloud,c.timeSeconds,[d.x d.y d.psi],history,s.cfg.sourceWindow);
    end
    inputs={fine,fineCurb,finePole,fineClouds{5},subset(s.source,s.source.components.semanticName=="curb"),oracle,unfiltered};
    rows=cell(numel(names),10);results=cell(numel(names),1);
    for k=1:numel(names)
        seed=s.seed;if k==5,seed=s.ref;end
        r=registerSemanticProbabilityCloud(s.fixed,inputs{k},seed,s.cfg.registration);results{k}=r;
        delta=r.poseXYTheta-s.ref;body=delta(1:2)*R;
        rows(k,:)={names(k),r.accepted,r.directionalAccepted,r.reason,norm(delta(1:2)),body(1),body(2), ...
            rad2deg(atan2(sin(delta(3)),cos(delta(3)))),r.observableRank,inputs{k}.components.numComponents};
    end
    controls=cell2table(rows,VariableNames={'variant','accepted','directional','reason','errorM','forwardErrorM','leftErrorM','yawErrorDeg','rank','sourceCount'});
    writetable(controls,fullfile(dest,'geometry_controls.csv'));disp(controls);
    % Evaluate each class under identical reference and fitted poses.
    model=prepareSemanticRegistrationGeometry(s.fixed,s.source,s.ref,s.cfg.registration);sc=[1;1;1/s.cfg.registration.yawLeverArm];rows=cell(0,9);systems=cell(2,1);
    for k=1:2
        pose=s.ref;if k==2,pose=s.results{1}.poseXYTheta;end
        sys=model.linearize([pose(1:2)-s.ref(1:2) pose(3)],sc);systems{k}=sys;
        for name=unique(sys.pairs.semanticName).'
            selected=find(sys.pairs.semanticName==name);h=zeros(3);g=zeros(3,1);
            for j=selected.'
                a=sys.J(:,:,j);w=sys.weights(j)*sys.robust(j);h=h+w*(a.'*a);g=g+w*a.'*sys.residual(:,j);
            end
            u=s.cfg.registration.geometric.robustStandardizedDistance;
            rows(end+1,:)={k,name,numel(selected),sum(sys.weights(selected)),sum(sys.weights(selected).*sys.robust(selected)), ...
                sum(sys.weights(selected)*u^2.*log1p(sys.pairs.squaredStandardizedResidual(selected)/u^2)),norm(g),min(eig(h)),max(eig(h))}; %#ok<AGROW>
        end
    end
    forces=cell2table(rows,VariableNames={'poseReference1Solution2','class','pairs','weight','robustWeight','cost','gradientNorm','minimumEigenvalue','maximumEigenvalue'});
    writetable(forces,fullfile(dest,'class_costs.csv'));disp(forces);
    % Nearest fine pole cluster to the one coarse pole, without changing labels.
    rawXYZ=double([raw(5).x(:) raw(5).y(:) raw(5).z(:)]);cfg=s.cfg.perception;[~,tilt]=poseRowToPlanarPose(poses(5,:));
    xyz=(rawXYZ*cfg.frameCalibration.rotation.'+cfg.frameCalibration.translation)*tilt.';
    poleIds=fineIndices{5,2};finePoints=xyz(poleIds,:);targetBody=(s.fixed.components.mean(target,:)-s.ref(1:2))*R;
    nearby=vecnorm(finePoints(:,1:2)-targetBody,2,2)<1.5;
    currentPole=s.current.components.mean(s.current.components.semanticName=="pole",:);
    poleSummary=struct('coarseWindowCenter',s.source.components.mean(id,:),'coarseCurrentCenters',currentPole,'mapCenterInReferenceBody',targetBody, ...
        'finePoleTotalPoints',numel(poleIds),'nearbyFinePoleCount',nnz(nearby),'nearbyFinePoleCenter',mean(finePoints(nearby,:),1), ...
        'nearbyFinePoleMin',min(finePoints(nearby,:),[],1),'nearbyFinePoleMax',max(finePoints(nearby,:),[],1));
    h=s.results{1}.scaledCurvature;[v,d]=eig(h,'vector');[~,small]=min(d);weak=v(:,small);weakBody=[weak(1:2).'*R weak(3)];
    poleSummary.scaledEigenRatio=min(d)/max(d);poleSummary.minimumRequiredRatio=s.cfg.registration.geometric.minimumObservabilityRatio;poleSummary.weakScaledBodyDirection=weakBody;
    fid=fopen(fullfile(dest,'pole_geometry.json'),'w');fprintf(fid,'%s\n',jsonencode(poleSummary,PrettyPrint=true));fclose(fid);disp(poleSummary);
    save(fullfile(out,'geometry.mat'),'fine','fineClouds','fineCurb','finePole','unfiltered','oracle','results','inputs','controls','systems','forces','poleSummary','fineIndices','xyz','-v7.3');
end
function out=replaceClass(a,b,name)
    a=subset(a,a.components.semanticName~=name);b=subset(b,b.components.semanticName==name);out=a;
    fields=["mean","semanticName","mixtureWeight","semanticProbability","occupancyProbability","temporalStability"];
    c=struct();for field=fields,c.(field)=[a.components.(field);b.components.(field)];end
    c.covariance=cat(3,a.components.covariance,b.components.covariance);c.numComponents=size(c.mean,1);out.components=c;
    if isfield(out,'heightEvidence'),out=rmfield(out,'heightEvidence');end
end
function cloud=subset(cloud,keep)
    c=cloud.components;n=c.numComponents;
    for field=string(fieldnames(c)).'
        a=c.(field);if field=="covariance",c.(field)=a(:,:,keep);elseif size(a,1)==n,c.(field)=a(keep,:);end
    end
    c.numComponents=nnz(keep);cloud.components=c;
    if isfield(cloud,'heightEvidence'),cloud=rmfield(cloud,'heightEvidence');end
end
