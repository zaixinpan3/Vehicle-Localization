function traceFrame932()
% traceFrame932 Separate motion transport, local curb shape, and pole rejection.
    root=setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));out='output/frame932_diagnosis_20260929';
    s=load(fullfile(out,'diagnostic.mat'));b=load('output/source_shape_matching_20260929/final_raw.mat','wc');
    d=load('output/root_cause_matching_20260929/finalSurface_sources.mat','currentClouds');
    od=load('output/line_direction_matching_20260928/sources.mat','motion');
    prior=load('output/line_direction_matching_20260928/production/report.mat','report');calls=prior.report.calls;
    disp(string(fieldnames(prior.report)));disp(prior.report.summary);
    disp(prior.report.metadata.motionSource);
    translationHistory=[];yawHistory=[];rows=cell(0,10);
    origin=od.motion(932,:);
    for k=928:932
        truth=calls{k,{'referenceX','referenceY','referencePsi'}};dt=calls.timeSeconds(932)-calls.timeSeconds(k);
        referenceShift=(truth(1:2)-s.ref(1:2))*rot(s.ref(3));motionShift=(od.motion(k,1:2)-origin(1:2))*rot(origin(3));
        surrogate=od.motion(k,:);surrogate(1:2)=origin(1:2)+referenceShift*rot(origin(3)).';
        [translationOracle,translationHistory]=updateLocalizationSourceWindow(d.currentClouds{k},calls.timeSeconds(k),surrogate,translationHistory,b.wc);
        surrogate=od.motion(k,:);surrogate(3)=origin(3)+truth(3)-s.ref(3);
        [yawOracle,yawHistory]=updateLocalizationSourceWindow(d.currentClouds{k},calls.timeSeconds(k),surrogate,yawHistory,b.wc);
        rows(end+1,:)={k,dt,motionShift(1),motionShift(2),referenceShift(1),referenceShift(2),rad2deg(od.motion(k,3)-origin(3)),rad2deg(truth(3)-s.ref(3)),referenceShift(1)-motionShift(1),referenceShift(2)-motionShift(2)}; %#ok<AGROW>
    end
    motion=cell2table(rows,VariableNames={'frame','ageSeconds','motionX','motionY','referenceX','referenceY','motionYawDeg','referenceYawDeg','translationDeltaX','translationDeltaY'});
    writetable(motion,fullfile(dest,'relative_motion.csv'));disp(motion);
    variants=["translation_transport_oracle","yaw_transport_oracle"];
    inputs={translationOracle,yawOracle};rows=cell(0,7);results=cell(2,1);
    for k=1:2
        r=matchLocalProbabilityCloud(s.fixed,inputs{k},s.initial,s.cfg);results{k}=r;e=r.poseXYTheta-s.ref;body=e(1:2)*rot(s.ref(3));
        rows(end+1,:)={variants(k),norm(e(1:2)),body(1),body(2),rad2deg(wrap(e(3))),r.accepted,r.directionalAccepted}; %#ok<AGROW>
    end
    controls=cell2table(rows,VariableNames={'variant','errorM','longitudinalM','lateralM','yawErrorDeg','accepted','directional'});
    writetable(controls,fullfile(dest,'transport_controls.csv'));disp(controls);
    fd=load('output/mississippi_mapping_calibrated/feature_observations.mat','featureData');mc=featureMapBuildConfig();
    store=matfile(fullfile(root,'data',mc.pointCloudMatPath));raw=store.pointClouds(1,932);
    cfg=perceptionConfig('Mississippi');cfg.coarseProbabilityCloud.storeDiagnostics=true;
    [~,tilt]=poseRowToPlanarPose(fd.featureData.framePoseTable(932,:));cfg.coarseProbabilityCloud.projectionRotation=tilt;
    p=perceiveFrame(raw,cfg);assert(isequaln(p.probabilityCloud,s.current));
    maps=p.diagnostics.offGround.columnMaps;context=p.diagnostics.offGroundPointContext;
    intensity=context.pointAttributes.intensity;
    structural=~(isfinite(intensity) & intensity>cfg.offGroundFeatures.trafficSignIntensityThreshold);
    pointScore=maps.pointScore;lineScore=maps.lineScore;g=context.pillarGeometry;
    if isfield(context,'sourcePillarGeometry')
        original=context.sourcePillarGeometry;
        offset=round((g.origin-original.origin)./g.cellSize);
        rr=(1:g.mapSize(1))+offset(2);cc=(1:g.mapSize(2))+offset(1);
        support=zeros(original.mapSize,'single');occupied=false(original.mapSize);
        support(rr,cc)=maps.supportEvidence;occupied(rr,cc)=maps.occupiedMask;
        [fullPoint,fullLine]=buildPillarShapeScores(support,occupied,cfg.offGroundFeatures,g.cellSize(1),g.cellSize(2));
        pointScore=fullPoint(rr,cc);lineScore=fullLine(rr,cc);
    end
    % Recompute diagnostic scores through the exact production classifier.
    [evidence,diagnostic]=classifyPillarPoleSupport(context.points,context.pointPillarLinIdx, ...
        g,maps.poleValidationProposals,cfg.offGroundFeatures.pole.validation,structural, ...
        pointScore,lineScore,cfg.offGroundFeatures.pole.distributionValidation);
    assert(isequaln(evidence,maps.poleValidation));
    model=loadPillarPoleModel(cfg.offGroundFeatures.pole.distributionValidation.modelFile);
    poleClass=string(fd.featureData.featureNames)=="pole";curbClass=string(fd.featureData.featureNames)=="curb";
    assert(nnz(poleClass)==1 && nnz(curbClass)==1);
    finePole=(double(fd.featureData.pointsByFeatureFrame{poleClass,932}(:,1:2))-s.ref(1:2))*rot(s.ref(3));
    fineCurb=(double(fd.featureData.pointsByFeatureFrame{curbClass,932}(:,1:2))-s.ref(1:2))*rot(s.ref(3));
    rows=cell(0,10);
    for k=1:numel(diagnostic.scores)
        h=diagnostic.hypotheses(diagnostic.hypothesisIds(k));
        body=[h.axisXY,h.axisZ]*p.probabilityCloud.projectionRotation.'+p.probabilityCloud.projectionTranslation;body=body(1:2);
        distance=min(vecnorm(finePole-body,2,2));
        if distance>1,continue;end
        rows(end+1,:)={diagnostic.owners(k),diagnostic.hypothesisIds(k),diagnostic.scores(k),cfg.offGroundFeatures.pole.distributionValidation.minimumScore,distance,h.supportHeight,h.radialRms,h.isolation,h.acceptedCount,body}; %#ok<AGROW>
    end
    poleScores=cell2table(rows,VariableNames={'ownerPillar','hypothesis','score','threshold','nearestFinePoleM','supportHeightM','radialRmsM','isolation','acceptedPoints','axisBody'});
    writetable(poleScores,fullfile(dest,'pole_rejection.csv'));disp(poleScores);
    fprintf('Current fine pole: %d points; body mean [%.5f, %.5f].\n',size(finePole,1),mean(finePole,1));
    features=array2table(diagnostic.features,VariableNames=cellstr(model.featureNames));features.ownerPillar=diagnostic.owners;features.score=diagnostic.scores;
    writetable(features(ismember(features.ownerPillar,poleScores.ownerPillar),:),fullfile(dest,'pole_features.csv'));
    % Per-frame threshold intervention, never a proposed global deployment.
    relaxedCfg=cfg;relaxedCfg.offGroundFeatures.pole.distributionValidation.minimumScore=.87;
    relaxed=perceiveFrame(raw,relaxedCfg);h=[];
    for k=928:931
        [~,h]=updateLocalizationSourceWindow(d.currentClouds{k},calls.timeSeconds(k),od.motion(k,:),h,b.wc);
    end
    [relaxedSource,h]=updateLocalizationSourceWindow(relaxed.probabilityCloud,calls.timeSeconds(932),od.motion(932,:),h,b.wc); %#ok<ASGLU>
    poleResult=matchLocalProbabilityCloud(s.fixed,relaxedSource,s.initial,s.cfg);error=poleResult.poseXYTheta-s.ref;
    poleControl=table(.87,nnz(relaxed.probabilityCloud.components.semanticName=="pole"),nnz(relaxedSource.components.semanticName=="pole"), ...
        norm(error(1:2)),rad2deg(wrap(error(3))),poleResult.accepted,poleResult.directionalAccepted, ...
        VariableNames={'frame932Threshold','currentPoles','confirmedPoles','errorM','yawErrorDeg','accepted','directional'});
    writetable(poleControl,fullfile(dest,'pole_detection_control.csv'));disp(poleControl);
    rows=cell(0,11);clouds={s.source,s.oracle};names=["odometry_pool","reference_motion_pool"];
    for k=1:2
        c=clouds{k}.components;[tangent,valid]=sourceLineDirections(c,s.cfg.lineDirection);
        for target=unique(s.results{1}.correspondences.globalTarget(s.results{1}.correspondences.semanticName=="curb")).'
            [v,e]=eig(s.fixed.components.covariance(:,:,target),'vector');[~,major]=max(e);axis=v(:,major).'*rot(s.ref(3));normal=[-axis(2),axis(1)];
            center=(s.fixed.components.mean(target,:)-s.ref(1:2))*rot(s.ref(3));
            ids=find(c.semanticName=="curb" & abs((c.mean-center)*axis.')<=3 & abs((c.mean-center)*normal.')<.5);
            for id=ids.'
                q=fineCurb(vecnorm(fineCurb-c.mean(id,:),2,2)<1.5,:);
                if size(q,1)<3,continue;end
                covariance=cov(q);[w,l]=eig(covariance,'vector');[~,j]=max(l);localTangent=w(:,j).';
                localNormal=[-localTangent(2),localTangent(1)];
                mapAngle=rad2deg(asin(dot(localTangent,normal)));
                coarseAngle=rad2deg(asin(dot(tangent(id,:),normal)));
                rows(end+1,:)={names(k),target,id,size(q,1),c.mean(id,:),mean(q,1),dot(c.mean(id,:)-mean(q,1),localNormal),dot(mean(q,1)-center,normal),mapAngle,coarseAngle,valid(id)}; %#ok<AGROW>
            end
        end
    end
    curbs=cell2table(rows,VariableNames={'pool','target','source','fineNeighbors','coarseMean','fineMean','coarseFineNormalM','fineMapNormalM','fineMapAngleDeg','coarseMapAngleDeg','directionValid'});
    writetable(curbs,fullfile(dest,'curb_geometry.csv'));disp(curbs);
    save(fullfile(out,'trace.mat'),'motion','translationOracle','yawOracle','controls','results','poleScores','finePole','fineCurb','curbs','diagnostic','poleControl','poleResult','relaxedSource','-v7.3');
end
function r=rot(a)
    r=[cos(a) -sin(a);sin(a) cos(a)];
end
function a=wrap(a)
    a=atan2(sin(a),cos(a));
end
