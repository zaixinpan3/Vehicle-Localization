function inspect_poles()
% inspect_poles Quantify individual association influence and source semantics.
    dest=fileparts(mfilename('fullpath'));out=fullfile(pwd,'output/frame959_matching_diagnosis_20260922');
    s=load(fullfile(out,'diagnostic.mat'));it=load(fullfile(out,'iteration_trace.mat'));
    exclusions={100,142,[100,142],[36,105,141]};names=["without_source100","without_source142", ...
        "without_two_distant_poles","without_sign_colocated_poles"];
    controls=cell(4,6);results=cell(4,1);
    for k=1:4
        cloud=subset(s.source,~ismember((1:s.source.components.numComponents).',exclusions{k}));
        r=registerSemanticProbabilityCloud(s.fixed,cloud,s.seed,s.cfg.registration);results{k}=r;
        controls(k,:)={names(k),r.accepted,r.reason,norm(r.poseXYTheta(1:2)-s.reference(1:2)), ...
            rad2deg(wrap(r.poseXYTheta(3)-s.reference(3))),r.similarity};
    end
    controls=cell2table(controls,VariableNames={'variant','accepted','reason','errorM','yawErrorDeg','similarity'});
    writetable(controls,fullfile(dest,'pole_ablation.csv'));disp(controls);
    % Linear contributions sum to the first unconstrained Gauss-Newton step.
    sys=it.trace{1}.system;contribution=zeros(sys.numPairs,3);
    for k=1:sys.numPairs
        force=sys.J(:,:,k).'*sys.residual(:,k)*sys.weights(k)*sys.robust(k);
        contribution(k,:)=(-sys.H\force).';
    end
    p=struct2table(sys.pairs);p.stepX=contribution(:,1);p.stepY=contribution(:,2);
    p.stepYawDeg=rad2deg(contribution(:,3)/s.cfg.registration.yawLeverArm);
    writetable(p,fullfile(dest,'first_step_contributions.csv'));
    counts=cellfun(@(c)c.components.numComponents,s.history.clouds);boundaries=[0;cumsum(counts)];
    queries=[36,100,105,141,142];audit=cell(numel(queries),13);rawNeighborhoods=cell(numel(queries),1);
    cfg=s.cfg.perception;mapCfg=featureMapBuildConfig();
    for frame=[957,958,959]
        frameSlot=frame-956;input=loadPointCloudFrame(fullfile(pwd,'data',mapCfg.pointCloudMatPath),frame);
        row=readFramePoseTable(fullfile(pwd,'data',mapCfg.poseMatchCsvPath),frame);[~,tilt]=poseRowToPlanarPose(row);
        cfg.coarseProbabilityCloud.projectionRotation=tilt;
        [current,~]=perceiveCoarseProbabilityCloud(input,cfg);
        cfg.executionMode="offline";fine=perceiveFrame(input,cfg);
        raw=[double(input.x(:)),double(input.y(:)),double(input.z(:))];
        calibrated=raw*current.projectionRotation.'+current.projectionTranslation;
        for k=find(queries>boundaries(frameSlot)&queries<=boundaries(frameSlot+1))
            q=queries(k);local=q-boundaries(frameSlot);meanXYZ=current.components.meanXYZ(local,:);
            nearby=all(isfinite(calibrated),2)&vecnorm(calibrated(:,1:2)-meanXYZ(1:2),2,2)<=.7;
            rawNeighborhoods{k}=struct('frame',frame,'points',calibrated(nearby,:), ...
                'originalIndex',find(nearby),'finePole',fine.featureMasks.pole(nearby), ...
                'fineSign',fine.featureMasks.trafficSign(nearby),'meanXYZ',meanXYZ);
            audit(k,:)={q,frame,local,meanXYZ(1),meanXYZ(2),meanXYZ(3),current.components.count(local), ...
                current.components.semanticProbability(local),current.components.occupancyProbability(local), ...
                nnz(fine.featureMasks.pole),nnz(fine.featureMasks.pole(nearby)),nnz(fine.featureMasks.trafficSign(nearby)),nnz(nearby)};
        end
    end
    audit=cell2table(audit,VariableNames={'source','frame','localComponent','meanX','meanY','meanZ','coarseHits', ...
        'semanticProbability','occupancyProbability','frameFinePolePoints','nearbyFinePolePoints','nearbyFineSignPoints','nearbyRawPoints'});
    writetable(audit,fullfile(dest,'pole_source_audit.csv'));disp(audit);
    save(fullfile(out,'pole_audit.mat'),'audit','rawNeighborhoods','controls','results');
end
function cloud=subset(cloud,keep)
    c=cloud.components;c.mean=c.mean(keep,:);c.covariance=c.covariance(:,:,keep);
    for field=["semanticName","mixtureWeight","repeatability","semanticProbability","occupancyProbability","supportAmplitude"]
        if isfield(c,field),c.(field)=c.(field)(keep,:);end
    end
    c.numComponents=nnz(keep);cloud.components=c;
end
function value=wrap(value)
    value=atan2(sin(value),cos(value));
end
