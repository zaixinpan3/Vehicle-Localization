function diagnoseFrame932()
% diagnoseFrame932 Reproduce the live maximum and isolate geometric bias.
% Reference-seeded and reference-motion controls are offline diagnostic
% oracles. No production algorithm or map is modified by this experiment.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    out='output/frame932_diagnosis_20260929';if ~isfolder(out),mkdir(out);end
    b=load('output/source_shape_matching_20260929/final_raw.mat');
    assert(b.maximum.frame==932);s=b.maximum;initial=s.predicted;ref=s.reference;
    cfg=b.cfg;source=s.source;current=s.current;map=load(b.mapFile,'cloud');
    fixed=conditionSemanticMapOnView(map.cloud,initial);fixed=rmfield(fixed,'landmarkViews');
    cfg.pyramid.mapMergeRadius=0;
    data=load('output/root_cause_matching_20260929/finalSurface_sources.mat','currentClouds');
    od=load('output/line_direction_matching_20260928/sources.mat','motion');
    saved=load('output/line_direction_matching_20260928/production/report.mat','report');calls=saved.report.calls;
    history=[];oracleHistory=[];
    for k=928:932
        [reconstructed,history,~,confirmed]=updateLocalizationSourceWindow(data.currentClouds{k},calls.timeSeconds(k),od.motion(k,:),history,b.wc);
        [oracle,oracleHistory]=updateLocalizationSourceWindow(data.currentClouds{k},calls.timeSeconds(k),calls{k,{'referenceX','referenceY','referencePsi'}},oracleHistory,b.wc);
    end
    assert(isequaln(reconstructed,source));
    names=["production","reference_seed_frozen","reference_seed_dynamic","iterations_400", ...
        "no_source_merge","single_scan","confirmed_current","reference_motion_pool", ...
        "current_curb_centers","current_curb_geometry","without_direction","direction_sigma_2deg", ...
        "direction_sigma_4deg","without_pole","pole_center_oracle"];
    targetIds=unique(s.result.correspondences.globalTarget(s.result.correspondences.semanticName=="curb"));
    for id=targetIds.',names(end+1)="drop_target_"+id;end %#ok<AGROW>
    rows=cell(numel(names),13);results=cell(numel(names),1);
    poleIds=find(source.components.semanticName=="pole");
    for n=1:numel(names)
        m=source;c=cfg;seed=initial;f=fixed;
        switch names(n)
            case "reference_seed_frozen",seed=ref;
            case "reference_seed_dynamic",seed=ref;f=map.cloud;
            case "iterations_400",c.maximumIterationsPerScale=400;
            case "no_source_merge",c.pyramid.sourceMergeRadius=0;
            case "single_scan",m=current;
            case "confirmed_current",m=confirmed;
            case "reference_motion_pool",m=oracle;
            case {"current_curb_centers","current_curb_geometry"}
                keep=source.heightEvidence.available & source.components.semanticName=="curb";
                m.components.mean(keep,:)=source.heightEvidence.mean(keep,1:2);
                if names(n)=="current_curb_geometry",m.components.covariance(:,:,keep)=source.heightEvidence.covariance(1:2,1:2,keep);end
            case "without_direction",c.lineDirection.enabled=false;
            case "direction_sigma_2deg",c.lineDirection.standardDeviation=deg2rad(2);
            case "direction_sigma_4deg",c.lineDirection.standardDeviation=deg2rad(4);
            case "without_pole",m=subset(m,m.components.semanticName~="pole");
            case "pole_center_oracle"
                pairs=s.result.correspondences;
                for id=poleIds.'
                    at=find(pairs.source==id);assert(isscalar(at));
                    m.components.mean(id,:)=(pairs.targetMeanXY(at,:)-ref(1:2))*rot(ref(3));
                end
            otherwise
                target=str2double(extractAfter(names(n),"drop_target_"));
                drop=s.result.correspondences.source(s.result.correspondences.globalTarget==target);
                m=removePreservingClassWeights(m,drop);
        end
        r=matchLocalProbabilityCloud(f,m,seed,c);results{n}=r;e=r.poseXYTheta-ref;body=e(1:2)*rot(ref(3));
        rows(n,:)={names(n),norm(e(1:2)),body(1),body(2),rad2deg(wrap(e(3))),r.accepted,r.directionalAccepted,r.reason,r.iterations,r.similarity,r.observableRank,r.pyramid.refinementShiftM,r.pyramid.coarseRetained};
    end
    assert(max(abs(results{1}.poseXYTheta-s.result.poseXYTheta))<1e-7);
    controls=cell2table(rows,VariableNames={'variant','errorM','longitudinalM','lateralM','yawErrorDeg','accepted','directional','reason','iterations','similarity','rank','refinementShiftM','coarseRetained'});
    writetable(controls,fullfile(dest,'controls.csv'));disp(controls);
    [local,globalIds]=selectLocalProbabilityCloud(fixed,initial,cfg.localMapRadius);
    model=prepareSemanticRegistrationGeometry(local,source,initial,cfg);scale=[1;1;.1];
    poses=[initial;ref;results{1}.poseXYTheta;results{2}.poseXYTheta];labels=["prediction","reference","production","reference_seed_frozen"];
    objective=cell(numel(labels),8);forceRows=cell(0,10);pairRows=cell(0,12);
    [tangent,valid,scatter]=sourceLineDirections(source.components,cfg.lineDirection);
    for n=1:numel(labels)
        sys=model.linearize([poses(n,1:2)-initial(1:2),poses(n,3)],scale);
        eigenvalues=sort(eig(sys.H));
        objective(n,:)={labels(n),sys.cost,sys.similarity,norm(sys.gradient),eigenvalues(1),eigenvalues(2),eigenvalues(3),sys.numPairs};
        for target=unique(globalIds(sys.pairs.target)).'
            ids=find(globalIds(sys.pairs.target)==target);gradient=zeros(3,1);directionGradient=zeros(3,1);cost=0;
            for j=ids.'
                gradient=gradient+sys.weights(j)*sys.robust(j)*sys.J(:,:,j).'*sys.residual(:,j);
                if size(sys.residual,1)==3,directionGradient=directionGradient+sys.weights(j)*sys.robust(j)*sys.J(3,:,j).'*sys.residual(3,j);end
                cost=cost+sys.weights(j)*cfg.geometric.robustStandardizedDistance^2*log1p(sys.pairs.squaredStandardizedResidual(j)/cfg.geometric.robustStandardizedDistance^2);
            end
            bodyGradient=gradient(1:2).'*rot(ref(3));
            forceRows(end+1,:)={labels(n),target,sys.pairs.semanticName(ids(1)),numel(ids),cost,bodyGradient(1),bodyGradient(2),gradient(3),directionGradient(3),sum(sys.weights(ids).*sys.robust(ids))}; %#ok<AGROW>
        end
        if labels(n)=="reference"
            for j=1:sys.numPairs
                id=sys.pairs.source(j);target=globalIds(sys.pairs.target(j));
                normal=sys.directionNormal(j,:)*rot(ref(3));targetBody=(sys.targetMean(j,:)+initial(1:2)-ref(1:2))*rot(ref(3));
                delta=source.components.mean(id,:)-targetBody;
                angle=asin(max(-1,min(1,dot(tangent(id,:),normal))));
                pairRows(end+1,:)={id,target,sys.pairs.semanticName(j),source.components.mean(id,:),targetBody,delta,dot(delta,normal),valid(id),rad2deg(angle),rad2deg(hypot(cfg.lineDirection.standardDeviation,.25*scatter(id))),sys.pairs.lineDirectionResidual(j),sys.pairs.weight(j)}; %#ok<AGROW>
            end
        end
        pairs=struct2table(sys.pairs);pairs.globalTarget=globalIds(pairs.target);pairs.targetMeanXY=pairs.targetMeanXY+initial(1:2);
        writetable(pairs,fullfile(dest,labels(n)+"_pairs.csv"));
    end
    objectives=cell2table(objective,VariableNames={'pose','cost','similarity','gradientNorm','lambda1','lambda2','lambda3','pairs'});
    forces=cell2table(forceRows,VariableNames={'pose','target','class','pairs','cost','gradientLongitudinal','gradientLateral','gradientScaledYaw','directionGradientScaledYaw','robustWeightSum'});
    geometry=cell2table(pairRows,VariableNames={'source','globalTarget','class','sourceBody','targetBody','deltaBody','normalOffsetM','directionUsed','directionMismatchDeg','directionSigmaDeg','directionResidual','weight'});
    writetable(objectives,fullfile(dest,'objectives.csv'));writetable(forces,fullfile(dest,'objective_forces.csv'));writetable(geometry,fullfile(dest,'reference_geometry.csv'));
    c=source.components;e=source.heightEvidence;
    tracks=table((1:c.numComponents).',c.semanticName,c.mean,c.detectionFrameCount,c.detectionFrameMask,e.available,e.mean(:,1:2),VariableNames={'id','class','mean','frames','support','currentAvailable','currentMean'});
    writetable(tracks,fullfile(dest,'source_tracks.csv'));
    rows=cell(0,7);transportRows=cell(0,8);
    for k=924:939
        c=data.currentClouds{k}.components;
        rows(end+1,:)={k,nnz(c.semanticName=="pole"),nnz(c.semanticName=="curb"),nnz(c.semanticName=="trafficSign"),b.replay.errorM(k),b.replay.yawErrorDeg(k),b.replay.reason(k)}; %#ok<AGROW>
        if k<928||k>932,continue;end
        reference=calls{k,{'referenceX','referenceY','referencePsi'}};
        delta=od.motion(k,:)-od.motion(932,:);truth=reference-ref;
        motionMean=c.mean(:,1:2)*rot(delta(3)).'+delta(1:2)*rot(od.motion(932,3));
        truthMean=c.mean(:,1:2)*rot(truth(3)).'+truth(1:2)*rot(ref(3));
        for j=1:c.numComponents
            transportRows(end+1,:)={k,j,c.semanticName(j),motionMean(j,:),truthMean(j,:),norm(motionMean(j,:)-truthMean(j,:)),truthMean(j,1)-motionMean(j,1),truthMean(j,2)-motionMean(j,2)}; %#ok<AGROW>
        end
    end
    neighbors=cell2table(rows,VariableNames={'frame','currentPoles','currentCurbs','currentSigns','errorM','yawErrorDeg','reason'});
    transport=cell2table(transportRows,VariableNames={'frame','id','class','odometryBody','referenceBody','transportErrorM','deltaX','deltaY'});
    writetable(neighbors,fullfile(dest,'neighbor_frames.csv'));writetable(transport,fullfile(dest,'motion_transport.csv'));
    error=results{1}.poseXYTheta-ref;prediction=initial-ref;
    summary=struct('frame',932,'positionErrorM',norm(error(1:2)),'bodyErrorM',error(1:2)*rot(ref(3)), ...
        'yawErrorDeg',rad2deg(wrap(error(3))),'predictionErrorM',norm(prediction(1:2)), ...
        'predictionYawErrorDeg',rad2deg(wrap(prediction(3))),'currentPoles',nnz(current.components.semanticName=="pole"), ...
        'confirmedPoles',numel(poleIds),'confirmedCurbs',nnz(source.components.semanticName=="curb"), ...
        'maximumTransportErrorM',max(transport.transportErrorM),'sourceReconstructionExact',true, ...
        'productionPoseReproduced',true,'sourceCodeCommit',string(getenv('FRAME932_SOURCE_COMMIT')));
    fid=fopen(fullfile(dest,'summary.json'),'w');fprintf(fid,'%s\n',jsonencode(summary,PrettyPrint=true));fclose(fid);
    save(fullfile(out,'diagnostic.mat'),'initial','ref','cfg','fixed','source','current','confirmed','oracle','results','controls','objectives','forces','geometry','history','summary','-v7.3');
    disp(objectives);disp(forces);disp(summary);
end
function m=removePreservingClassWeights(m,drop)
    c=m.components;quality=c.semanticProbability.*c.occupancyProbability;
    for name=unique(c.semanticName(drop)).'
        all=c.semanticName==name;remaining=all;remaining(drop)=false;
        c.temporalStability(remaining)=c.temporalStability(remaining)*sum(quality(remaining))/sum(quality(all));
    end
    c.temporalStability(drop)=0;m.components=c;
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
function r=rot(a)
    r=[cos(a) -sin(a);sin(a) cos(a)];
end
function a=wrap(a)
    a=atan2(sin(a),cos(a));
end
