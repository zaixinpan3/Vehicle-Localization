function inspectMechanisms()
% inspectMechanisms Check weight-preserving ablation and objective force sources.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    s=load('output/frame178_diagnosis_20260929/diagnostic.mat');
    d=load('output/root_cause_matching_20260929/finalSurface_sources.mat','sources','currentClouds');
    local=selectLocalProbabilityCloud(s.fixed,s.initial,s.cfg.localMapRadius);
    base=prepareSemanticRegistrationGeometry(local,s.source,s.initial,s.cfg);
    scale=[1;1;.1];referencePose=[s.ref(1:2)-s.initial(1:2),s.ref(3)];
    baseline=base.linearize(referencePose,scale);
    rows=cell(0,10);names=["drop22_preserve_other_weights","sign22_fine_center_oracle","sign22_half_weight","sign22_tenth_weight","iterations_400"];
    acquisition=readtable(fullfile(dest,'sign_acquisitions.csv'));track=acquisition(acquisition.pooledSource==22,:);
    results=cell(numel(names),1);
    for k=1:numel(names)
        m=s.source;cfg=s.cfg;
        switch names(k)
            case "drop22_preserve_other_weights"
                quality=m.components.semanticProbability.*m.components.occupancyProbability;
                sign=m.components.semanticName=="trafficSign";remaining=sign;remaining(22)=false;
                fraction=sum(quality(remaining))/sum(quality(sign));
                m.components.temporalStability(remaining)=m.components.temporalStability(remaining)*fraction;
                m.components.temporalStability(22)=0;
                model=prepareSemanticRegistrationGeometry(local,m,s.initial,cfg);sys=model.linearize(referencePose,scale);
                [~,index]=ismember(sys.pairs.source,baseline.pairs.source);
                assert(max(abs(sys.weights-baseline.weights(index)))<1e-12);
            case "sign22_fine_center_oracle",m.components.mean(22,:)=mean(track{:,{'fineX','fineY'}},1);
            case "sign22_half_weight",m.components.temporalStability(22)=.5;
            case "sign22_tenth_weight",m.components.temporalStability(22)=.1;
            case "iterations_400",cfg.maximumIterationsPerScale=400;
        end
        r=matchLocalProbabilityCloud(s.fixed,m,s.initial,cfg);results{k}=r;
        e=r.poseXYTheta-s.ref;body=e(1:2)*rot(s.ref(3));
        rows(end+1,:)={names(k),norm(e(1:2)),body(1),body(2),rad2deg(atan2(sin(e(3)),cos(e(3)))),r.accepted,r.directionalAccepted,r.reason,r.iterations,r.similarity}; %#ok<AGROW>
    end
    controls=cell2table(rows,VariableNames={'variant','errorM','longitudinalM','lateralM','yawErrorDeg','accepted','directional','reason','iterations','similarity'});
    writetable(controls,fullfile(dest,'mechanism_controls.csv'));disp(controls);
    labels=["reference","production","reference_seed_frozen","drop22_preserve_other_weights"];
    poses=[s.ref;s.results{1}.poseXYTheta;s.results{2}.poseXYTheta;results{1}.poseXYTheta];
    rows=cell(0,11);
    for k=1:numel(labels)
        sys=base.linearize([poses(k,1:2)-s.initial(1:2),poses(k,3)],scale);
        groups={sys.pairs.semanticName=="curb",sys.pairs.source==21,sys.pairs.source==22,sys.pairs.source==30};
        groupNames=["all_curb","sign21","sign22","sign30"];
        for j=1:4
            ids=find(groups{j});h=zeros(3);gradient=zeros(3,1);
            for i=ids.'
                weight=sys.weights(i)*sys.robust(i);jac=sys.J(:,:,i);
                h=h+weight*(jac.'*jac);gradient=gradient+weight*jac.'*sys.residual(:,i);
            end
            eigenvalues=sort(eig(h));bodyGradient=gradient(1:2).'*rot(s.ref(3));
            cost=sum(sys.weights(ids)*s.cfg.geometric.robustStandardizedDistance^2.*log1p(sys.pairs.squaredStandardizedResidual(ids)/s.cfg.geometric.robustStandardizedDistance^2));
            rows(end+1,:)={labels(k),groupNames(j),numel(ids),cost,bodyGradient(1),bodyGradient(2),gradient(3),eigenvalues(1),eigenvalues(2),eigenvalues(3),sum(sys.weights(ids).*sys.robust(ids))}; %#ok<AGROW>
        end
    end
    forces=cell2table(rows,VariableNames={'pose','group','pairs','cost','gradientLongitudinal','gradientLateral','gradientScaledYaw','lambda1','lambda2','lambda3','robustWeightSum'});
    writetable(forces,fullfile(dest,'objective_forces.csv'));
    rows=cell(0,6);
    for k=170:185
        current=d.currentClouds{k}.components;pooled=d.sources{k}.components;
        rows(end+1,:)={k,nnz(current.semanticName=="pole"),nnz(pooled.semanticName=="pole"),nnz(current.semanticName=="trafficSign"),nnz(pooled.semanticName=="trafficSign"),nnz(pooled.semanticName=="curb")}; %#ok<AGROW>
    end
    counts=cell2table(rows,VariableNames={'frame','currentPoles','confirmedPoles','currentSigns','confirmedSigns','confirmedCurbs'});
    writetable(counts,fullfile(dest,'neighbor_support.csv'));disp(counts);
    od=load('output/line_direction_matching_20260928/sources.mat','motion');rows=cell(0,7);
    for k=170:179
        c=d.currentClouds{k}.components;delta=od.motion(k,:)-od.motion(178,:);
        for id=find(c.semanticName=="pole").'
            mu=c.mean(id,:)*rot(delta(3)).'+delta(1:2)*rot(od.motion(178,3));
            rows(end+1,:)={k,id,mu(1),mu(2),c.semanticProbability(id),c.count(id),c.meanXYZ(id,3)}; %#ok<AGROW>
        end
    end
    poles=cell2table(rows,VariableNames={'frame','component','x178','y178','semanticProbability','points','meanZ'});
    writetable(poles,fullfile(dest,'pole_acquisitions.csv'));
    pair=s.results{6}.correspondences;writetable(pair,fullfile(dest,'added_pole_pairs.csv'));
    e=s.initial-s.ref;body=e(1:2)*rot(s.ref(3));
    summary=struct('predictionErrorM',norm(e(1:2)),'predictionBodyErrorM',body,'predictionYawErrorDeg',rad2deg(atan2(sin(e(3)),cos(e(3)))), ...
        'weightPreservingAblationVerified',true,'sign22FineCenterOracle',mean(track{:,{'fineX','fineY'}},1), ...
        'maximumPoseChangeWith400Iterations',max(abs(results{5}.poseXYTheta-s.results{1}.poseXYTheta)));
    fid=fopen(fullfile(dest,'mechanism_summary.json'),'w');fprintf(fid,'%s\n',jsonencode(summary,PrettyPrint=true));fclose(fid);
    save('output/frame178_diagnosis_20260929/mechanisms.mat','results','controls','forces','counts','summary');
end
function r=rot(a)
    r=[cos(a) -sin(a);sin(a) cos(a)];
end
