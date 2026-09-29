function checkDiagnosis()
% checkDiagnosis Verify the recorded controls and their intervention scope.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    s=load('output/frame932_diagnosis_20260929/diagnostic.mat');
    t=load('output/frame932_diagnosis_20260929/trace.mat');
    row=@(name)s.controls(s.controls.variant==name,:);
    base=row("production");seed=row("reference_seed_frozen");iterations=row("iterations_400");
    assert(abs(base.errorM-.159861269865301)<1e-10);
    assert(base.accepted && base.rank==3);
    assert(abs(seed.errorM-base.errorM)<2e-6);
    assert(iterations.errorM==base.errorM);
    assert(nnz(s.current.components.semanticName=="pole")==0);
    pole=find(s.source.components.semanticName=="pole");assert(isscalar(pole));
    assert(s.source.components.detectionFrameCount(pole)==4 && ~s.source.heightEvidence.available(pole));
    assert(isequal(s.source.components.detectionFrameMask(pole,:),logical([1 1 1 1 0])));
    assert(row("reference_motion_pool").errorM<.06);
    assert(t.controls.errorM(t.controls.variant=="translation_transport_oracle")<.065);
    assert(t.controls.errorM(t.controls.variant=="yaw_transport_oracle")>.15);
    assert(abs(t.motion.translationDeltaY(1)+.0945959)<1e-5);
    assert(abs(t.motion.referenceYawDeg(1)-t.motion.motionYawDeg(1))<.04);
    assert(t.poleControl.currentPoles==1 && t.poleControl.confirmedPoles==1);
    assert(t.poleControl.errorM>base.errorM && t.poleControl.errorM<.161);
    near=t.poleScores(t.poleScores.ownerPillar==5945,:);assert(height(near)==1);
    assert(near.score<near.threshold && near.score>.87 && near.nearestFinePoleM<.02);
    local=selectLocalProbabilityCloud(s.fixed,s.initial,s.cfg.localMapRadius);
    model=prepareSemanticRegistrationGeometry(local,s.source,s.initial,s.cfg);
    pose=[s.ref(1:2)-s.initial(1:2),s.ref(3)];scale=[1;1;.1];system=model.linearize(pose,scale);
    removed=s.results{1}.correspondences.source(s.results{1}.correspondences.globalTarget==711);
    m=s.source;c=m.components;q=c.semanticProbability.*c.occupancyProbability;
    curb=c.semanticName=="curb";remaining=curb;remaining(removed)=false;
    c.temporalStability(remaining)=c.temporalStability(remaining)*sum(q(remaining))/sum(q(curb));c.temporalStability(removed)=0;
    m.components=c;ablated=prepareSemanticRegistrationGeometry(local,m,s.initial,s.cfg);alternative=ablated.linearize(pose,scale);
    [present,ids]=ismember(alternative.pairs.source,system.pairs.source);assert(all(present));
    assert(max(abs(alternative.weights-system.weights(ids)))<1e-12);
    assert(max(abs(alternative.residual-system.residual(:,ids)),[],'all')<1e-12);
    assert(row("drop_target_711").errorM<.107);
    assert(s.objectives.cost(s.objectives.pose=="reference")>s.objectives.cost(s.objectives.pose=="production"));
    files=["diagnoseFrame932.m","traceFrame932.m","showFrame932Matching.m","checkDiagnosis.m"];
    analyzerRows=cell(numel(files),2);
    for k=1:numel(files)
        issues=checkcode(fullfile(dest,files(k)),'-id','-config=factory');analyzerRows(k,:)={files(k),numel(issues)};
        for issue=issues.'
            fprintf('%s:%d [%s] %s\n',files(k),issue.line(1),issue.id,issue.message);
        end
    end
    analyzer=cell2table(analyzerRows,VariableNames={'file','findings'});writetable(analyzer,fullfile(dest,'code_analysis.csv'));
    validation=struct('recordedMaximumReproduced',true,'sourceHistoryReconstructedExactly',true, ...
        'referenceSeedConvergesBackToBiasedPose',true,'translationAndYawControlsSeparated',true, ...
        'curb711OtherWeightsAndResidualsPreserved',true,'currentPoleReinstatementDoesNotImprovePose',true, ...
        'productionCodeChanged',false,'validationScope',"Saved-data assertions and diagnostic code analysis; no new full-route regression");
    fid=fopen(fullfile(dest,'validation.json'),'w');fprintf(fid,'%s\n',jsonencode(validation,PrettyPrint=true));fclose(fid);
    disp(validation);disp(analyzer);
end
