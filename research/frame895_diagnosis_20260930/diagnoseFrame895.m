function diagnoseFrame895()
% diagnoseFrame895 Isolate prediction, support transport and geometry bias.
% Reference seed/motion interventions are offline diagnostic oracles only.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    out='output/frame895_diagnosis_20260930';if ~isfolder(out),mkdir(out);end
    b=load('output/support_matching_repair_20260930/final_default.mat');s=b.maximum;
    assert(s.frame==895);initial=s.predicted;ref=s.reference;source=s.source;cfg=b.cfg;
    a=load('output/source_shape_matching_20260929/final_raw.mat','wc');wc=a.wc;
    a=load('output/root_cause_matching_20260929/finalSurface_sources.mat','currentClouds');currentClouds=a.currentClouds;
    a=load('output/line_direction_matching_20260928/sources.mat','motion');motion=a.motion;
    a=load('output/line_direction_matching_20260928/production/report.mat','report');calls=a.report.calls;
    a=load(featureMapBuildConfig().probabilityCloudPath,'cloud');map=a.cloud;
    fixed=conditionSemanticMapOnView(map,initial);fixed=rmfield(fixed,'landmarkViews');cfg.pyramid.mapMergeRadius=0;
    history=[];oracleHistory=[];translationHistory=[];yawHistory=[];motionRows=cell(5,8);
    for k=891:895
        truth=calls{k,{'referenceX','referenceY','referencePsi'}};
        [reconstructed,history,~,confirmed]=updateLocalizationSourceWindow(currentClouds{k},calls.timeSeconds(k),motion(k,:),history,wc);
        [oracle,oracleHistory]=updateLocalizationSourceWindow(currentClouds{k},calls.timeSeconds(k),truth,oracleHistory,wc);
        shift=(truth(1:2)-ref(1:2))*rot(ref(3));surrogate=motion(k,:);
        surrogate(1:2)=motion(895,1:2)+shift*rot(motion(895,3)).';
        [translationOracle,translationHistory]=updateLocalizationSourceWindow(currentClouds{k},calls.timeSeconds(k),surrogate,translationHistory,wc);
        surrogate=motion(k,:);surrogate(3)=motion(895,3)+wrap(truth(3)-ref(3));
        [yawOracle,yawHistory]=updateLocalizationSourceWindow(currentClouds{k},calls.timeSeconds(k),surrogate,yawHistory,wc);
        estimate=(motion(k,1:2)-motion(895,1:2))*rot(motion(895,3));
        motionRows(k-890,:)={k,calls.timeSeconds(895)-calls.timeSeconds(k),estimate(1),estimate(2),shift(1),shift(2), ...
            rad2deg(wrap(motion(k,3)-motion(895,3))),rad2deg(wrap(truth(3)-ref(3)))};
    end
    assert(isequaln(reconstructed,source));
    current=currentClouds{895};targets=unique(s.result.correspondences.globalTarget);
    names=["production","reference_seed_frozen","reference_seed_dynamic","iterations_400", ...
        "no_source_merge","single_scan","confirmed_current","reference_motion_pool", ...
        "translation_transport_oracle","yaw_transport_oracle","current_curb_centers","current_curb_geometry", ...
        "without_pole","without_curb","without_sign","negligible_direction"];
    for id=targets.',names(end+1)="drop_target_"+id;end %#ok<AGROW>
    rows=cell(numel(names),12);results=cell(size(names));
    for k=1:numel(names)
        m=source;c=cfg;seed=initial;f=fixed;
        switch names(k)
            case "reference_seed_frozen",seed=ref;
            case "reference_seed_dynamic",seed=ref;f=map;
            case "iterations_400",c.maximumIterationsPerScale=400;
            case "no_source_merge",c.pyramid.sourceMergeRadius=0;
            case "single_scan",m=current;
            case "confirmed_current",m=confirmed;
            case "reference_motion_pool",m=oracle;
            case "translation_transport_oracle",m=translationOracle;
            case "yaw_transport_oracle",m=yawOracle;
            case {"current_curb_centers","current_curb_geometry"}
                keep=source.heightEvidence.available & source.components.semanticName=="curb";
                m.components.mean(keep,:)=source.heightEvidence.mean(keep,1:2);
                if names(k)=="current_curb_geometry",m.components.covariance(:,:,keep)=source.heightEvidence.covariance(1:2,1:2,keep);end
            case "without_pole",m=suppress(m,m.components.semanticName=="pole");
            case "without_curb",m=suppress(m,m.components.semanticName=="curb");
            case "without_sign",m=suppress(m,m.components.semanticName=="trafficSign");
            case "negligible_direction",c.support.scatterScale=1e6;
            otherwise
                if startsWith(names(k),"drop_target_")
                    target=str2double(extractAfter(names(k),"drop_target_"));
                    drop=ismember((1:m.components.numComponents).',s.result.correspondences.source(s.result.correspondences.globalTarget==target));
                    m=suppress(m,drop);
                end
        end
        r=matchLocalProbabilityCloud(f,m,seed,c);results{k}=r;e=r.poseXYTheta-ref;body=e(1:2)*rot(ref(3));
        rows(k,:)={names(k),norm(e(1:2)),body(1),body(2),rad2deg(wrap(e(3))),r.accepted,r.directionalAccepted,r.reason, ...
            r.iterations,r.observableRank,r.similarity,r.pyramid.coarseRetained};
    end
    assert(max(abs(results{1}.poseXYTheta-s.result.poseXYTheta))<1e-7);
    controls=cell2table(rows,VariableNames={'variant','errorM','longitudinalM','lateralM','yawErrorDeg','accepted','directional','reason','iterations','rank','similarity','coarseRetained'});
    writetable(controls,fullfile(dest,'controls.csv'));disp(controls);
    [local,globalIds]=selectLocalProbabilityCloud(fixed,initial,cfg.localMapRadius);
    fineCfg=cfg;if ~s.result.pyramid.softFineAssociation,fineCfg=rmfield(fineCfg,'softPointAssociation');end
    model=prepareSemanticRegistrationGeometry(local,source,initial,fineCfg);scale=[1;1;1/cfg.yawLeverArm];
    poses=[initial;ref;s.result.poseXYTheta];labels=["prediction","reference","production"];
    objective=cell(3,7);forces=cell(0,11);pairRows=cell(0,13);
    for k=1:3
        pose=[poses(k,1:2)-initial(1:2),poses(k,3)];sys=model.linearize(pose,scale);
        [~,~,rank,eigenvalues]=registrationObservableStep(sys.H,sys.gradient,cfg.geometric.minimumObservabilityRatio,true);
        objective(k,:)={labels(k),sys.cost,norm(sys.gradient),rank,eigenvalues(1),eigenvalues(2),eigenvalues(3)};
        for target=unique(sys.pairs.target).'
            ids=find(sys.pairs.target==target);g=zeros(3,1);angular=0;cost=0;
            for j=ids.'
                w=sys.weights(j)*sys.robust(j);g=g+w*sys.J(:,:,j).'*sys.residual(:,j);
                angular=angular+w*sys.J(3,3,j)*sys.residual(3,j);
                cost=cost+sys.weights(j)*cfg.geometric.robustStandardizedDistance^2*log1p(sys.pairs.squaredStandardizedResidual(j)/cfg.geometric.robustStandardizedDistance^2);
            end
            body=g(1:2).'*rot(ref(3));
            forces(end+1,:)={labels(k),globalIds(target),sys.pairs.semanticName(ids(1)),numel(ids),cost,body(1),body(2),g(3),angular,sum(sys.weights(ids)),sum(sys.weights(ids).*sys.robust(ids))}; %#ok<AGROW>
        end
        if k==2
            for j=1:sys.numPairs
                id=sys.pairs.source(j);target=sys.pairs.target(j);normal=model.fixed.orientationNormal(target,:)*rot(ref(3));
                targetBody=(sys.targetMean(j,:)+initial(1:2)-ref(1:2))*rot(ref(3));
                delta=source.components.mean(id,:)-targetBody;tangent=model.moving.supportTangent(id,:);
                angle=asin(max(-1,min(1,dot(tangent,normal))));
                pairRows(end+1,:)={id,globalIds(target),sys.pairs.semanticName(j),source.components.mean(id,:),targetBody,delta, ...
                    dot(delta,normal),rad2deg(angle),sys.directionScale(j),sys.pairs.slidingFraction(j),sys.pairs.weight(j),source.components.detectionFrameCount(id),source.heightEvidence.available(id)}; %#ok<AGROW>
            end
        end
        pairs=struct2table(sys.pairs);pairs.globalTarget=globalIds(pairs.target);pairs.targetMeanXY=pairs.targetMeanXY+initial(1:2);
        writetable(pairs,fullfile(dest,labels(k)+"_pairs.csv"));
    end
    objectives=cell2table(objective,VariableNames={'pose','cost','gradientNorm','rank','lambda1','lambda2','lambda3'});
    forceTable=cell2table(forces,VariableNames={'pose','target','class','pairs','cost','gradientLongitudinal','gradientLateral','gradientScaledYaw','angularGradientScaledYaw','weight','robustWeight'});
    geometry=cell2table(pairRows,VariableNames={'source','target','class','sourceBody','targetBody','deltaBody','normalOffsetM','directionMismatchDeg','directionScale','slidingFraction','weight','detections','currentAvailable'});
    relativeMotion=cell2table(motionRows,VariableNames={'frame','ageSeconds','motionLongitudinalM','motionLateralM','referenceLongitudinalM','referenceLateralM','motionYawDeg','referenceYawDeg'});
    writetable(objectives,fullfile(dest,'objectives.csv'));writetable(forceTable,fullfile(dest,'objective_forces.csv'));writetable(geometry,fullfile(dest,'reference_geometry.csv'));writetable(relativeMotion,fullfile(dest,'relative_motion.csv'));
    e=s.result.poseXYTheta-ref;pred=initial-ref;
    summary=struct('frame',895,'positionErrorM',norm(e(1:2)),'bodyErrorM',e(1:2)*rot(ref(3)), ...
        'yawErrorDeg',rad2deg(wrap(e(3))),'predictionErrorM',norm(pred(1:2)), ...
        'predictionBodyErrorM',pred(1:2)*rot(ref(3)),'predictionYawErrorDeg',rad2deg(wrap(pred(3))), ...
        'updateBodyM',(s.result.poseXYTheta(1:2)-initial(1:2))*rot(ref(3)), ...
        'currentPoles',nnz(current.components.semanticName=="pole"),'currentCurbs',nnz(current.components.semanticName=="curb"), ...
        'currentSigns',nnz(current.components.semanticName=="trafficSign"), ...
        'confirmedPoles',nnz(source.components.semanticName=="pole"),'confirmedCurbs',nnz(source.components.semanticName=="curb"), ...
        'confirmedSigns',nnz(source.components.semanticName=="trafficSign"), ...
        'sourceReconstructionExact',true,'productionReproduced',true);
    fid=fopen(fullfile(dest,'summary.json'),'w');fprintf(fid,'%s\n',jsonencode(summary,PrettyPrint=true));fclose(fid);
    save(fullfile(out,'diagnostic.mat'),'initial','ref','source','current','confirmed','oracle','translationOracle','yawOracle', ...
        'cfg','fixed','local','globalIds','results','controls','objectives','geometry','summary','wc','-v7.3');
    disp(summary);disp(objectives);disp(forceTable);disp(geometry);disp(relativeMotion);
end
function m=suppress(m,drop)
% Keep other class-normalized weights fixed; neighborhood membership may change.
    c=m.components;q=c.semanticProbability.*c.occupancyProbability;
    for name=unique(c.semanticName(drop)).'
        all=c.semanticName==name;remaining=all & ~drop;
        c.temporalStability(remaining)=c.temporalStability(remaining)*sum(q(remaining))/sum(q(all));
    end
    c.temporalStability(drop)=0;m.components=c;
end
function r=rot(a),r=[cos(a) -sin(a);sin(a) cos(a)];end
function a=wrap(a),a=atan2(sin(a),cos(a));end
