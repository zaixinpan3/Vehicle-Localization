function runFrame307Diagnostics(repoRoot)
% runFrame307Diagnostics Audit the historical worst accepted XY match.
% Uses the unmodified historical runtime; all ablations are offline controls.
    setupVehicleLocalization();
    assert(contains(which('registerSemanticProbabilityCloud'), ...
        'reference_free_785_20260920/historical_runtime'));
    out=fullfile(repoRoot,'output/frame307_matching_diagnosis_20260921');
    if ~isfolder(out), mkdir(out); end
    saved=load(fullfile(repoRoot,'output/saved_perception_inspva_20260915/experiment.mat'), ...
        'clouds','reference','time','cfg');
    fixed=load(fullfile(repoRoot,'output/mississippi_mapping_inspva_20260915/probability_cloud.mat'));
    features=load(fullfile(repoRoot,'output/mississippi_mapping_inspva_20260915/feature_observations.mat'));
    cfg=saved.cfg; frame=307; pose=saved.reference(frame,:); source=saved.clouds{frame};
    rotation=[cos(pose(3)) -sin(pose(3));sin(pose(3)) cos(pose(3))];
    baseline=registerSemanticProbabilityCloud(fixed.cloud,source,pose,cfg);
    calls=readtable(fullfile(repoRoot,'output/saved_perception_inspva_20260915/calls.csv'));
    zero=calls(string(calls.mode)=="per_frame_zero" & calls.fullPose==1,:);
    [maximum,index]=max(zero.mappingErrorM);
    assert(zero.frame(index)==frame && abs(maximum-0.575064925868327)<1e-9);
    recorded=zero{index,{'outputX','outputY','outputPsi'}};
    assert(max(abs(baseline.poseXYTheta-recorded))<1e-7);
    variants=["baseline","without_curb","without_pole","without_sign", ...
        "curb_only","pole_only","without_pole_949","without_pole_931"];
    rows=cell(numel(variants),12); results=cell(numel(variants),1);
    for k=1:numel(variants)
        keep=true(source.components.numComponents,1);
        names=source.components.semanticName;
        switch variants(k)
            case "without_curb", keep=names~="curb";
            case "without_pole", keep=names~="pole";
            case "without_sign", keep=names~="trafficSign";
            case "curb_only", keep=names=="curb";
            case "pole_only", keep=names=="pole";
            case "without_pole_949", keep(33)=false;
            case "without_pole_931", keep(34:35)=false;
        end
        r=registerSemanticProbabilityCloud(fixed.cloud,subsetCloud(source,keep),pose,cfg);
        error=r.poseXYTheta-pose; body=error(1:2)*rotation;
        rows(k,:)={variants(k),norm(error(1:2)),body(1),body(2),rad2deg(error(3)), ...
            r.accepted,r.reason,r.observableRank,r.similarity,r.poseXYTheta(1),r.poseXYTheta(2),r.poseXYTheta(3)};
        results{k}=r;
    end
    controls=cell2table(rows,VariableNames={'variant','positionErrorM','forwardErrorM','leftErrorM', ...
        'yawErrorDeg','accepted','reason','rank','similarity','x','y','yaw'});
    writetable(controls,fullfile(out,'controls.csv'));
    held=cfg; held.stepTolerance=1e6;
    pairRows=cell(0,13); classRows=cell(0,8);
    for mode=["reference","estimate"]
        p=pose; if mode=="estimate",p=baseline.poseXYTheta;end
        r=registerSemanticProbabilityCloud(fixed.cloud,source,p,held);
        pairs=r.correspondences;
        for j=1:height(pairs)
            s=source.components.mean(pairs.source(j),:);
            target=(fixed.cloud.components.mean(pairs.target(j),:)-pose(1:2))*rotation;
            pairRows(end+1,:)={mode,pairs.source(j),pairs.target(j),pairs.semanticName(j), ...
                s(1),s(2),target(1),target(2),target(1)-s(1),target(2)-s(2), ...
                pairs.weight(j),pairs.robustWeight(j),pairs.squaredStandardizedResidual(j)}; %#ok<AGROW>
        end
        scale=cfg.geometric.robustStandardizedDistance^2;
        for name=unique(pairs.semanticName).'
            selected=pairs.semanticName==name;
            d=r.classDiagnostics(r.classDiagnostics.semanticName==name,:);
            cost=sum(pairs.weight(selected).*scale.*log1p(pairs.squaredStandardizedResidual(selected)/scale));
            classRows(end+1,:)={mode,name,nnz(selected),numel(unique(pairs.target(selected))), ...
                sum(pairs.weight(selected)),cost,d.observableCorrection,d.observableRank}; %#ok<AGROW>
        end
    end
    pairs=cell2table(pairRows,VariableNames={'poseMode','source','target','semanticName', ...
        'sourceForwardM','sourceLeftM','targetForwardM','targetLeftM','pullForwardM','pullLeftM', ...
        'weight','robustWeight','squaredStandardizedResidual'});
    classes=cell2table(classRows,VariableNames={'poseMode','semanticName','pairs','uniqueTargets','weight', ...
        'robustCost','remainingCorrectionM','rank'});
    writetable(pairs,fullfile(out,'pairs.csv'));writetable(classes,fullfile(out,'classes.csv'));
    % Export nearby observations in a fixed frame-307 coordinate system.
    % These raw per-frame centroids are diagnostics, not map GMM estimates.
    landmarkRows=cell(0,7);
    for target=[949 931 1203]
        name=fixed.cloud.components.semanticName(target);
        classIndex=features.featureData.featureNames==name;
        center=fixed.cloud.components.mean(target,:);
        for k=295:315
            xyz=features.featureData.pointsByFeatureFrame{classIndex,k};
            selected=vecnorm(xyz(:,1:2)-center,2,2)<1;
            if nnz(selected)<2,continue;end
            points=(xyz(selected,1:2)-center)*rotation;
            landmarkRows(end+1,:)={k,target,name,nnz(selected),mean(points(:,1)), ...
                mean(points(:,2)),std(points(:,1))}; %#ok<AGROW>
        end
    end
    observations=cell2table(landmarkRows,VariableNames={'frame','target','semanticName', ...
        'points','forwardOffsetFromMapM','leftOffsetFromMapM','forwardSpreadM'});
    writetable(observations,fullfile(out,'landmark_observations.csv'));
    writetable(features.featureData.framePoseTable(295:315,:),fullfile(out,'pose_neighborhood.csv'));
    verification=struct('frame',frame,'timeSeconds',saved.time(frame),'acceptedFrames',height(zero), ...
        'positionErrorM',maximum,'maximumReproductionDifference',max(abs(baseline.poseXYTheta-recorded)), ...
        'referencePose',pose,'matchingPose',baseline.poseXYTheta,'information',baseline.information, ...
        'scaledCurvatureEigenvalues',eig(baseline.scaledCurvature).', ...
        'runtimeRevision','d6502080b392e6d368696eeb0b7fd755f556752b');
    fid=fopen(fullfile(out,'verification.json'),'w');cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(verification,PrettyPrint=true));
    save(fullfile(out,'diagnostics.mat'),'baseline','results','verification','cfg');
    disp(controls);disp(classes);
end

function cloud=subsetCloud(cloud,keep)
% Preserve component covariance pages and metadata while selecting classes.
    c=cloud.components;n=c.numComponents;
    for field=string(fieldnames(c)).'
        a=c.(field);
        if field=="numComponents",continue;end
        if contains(lower(field),"covariance") && size(a,3)==n
            c.(field)=a(:,:,keep);
        elseif size(a,1)==n
            c.(field)=a(keep,:);
        end
    end
    c.numComponents=nnz(keep);cloud.components=c;
end
