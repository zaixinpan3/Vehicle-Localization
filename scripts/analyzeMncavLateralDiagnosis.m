function report=analyzeMncavLateralDiagnosis(outputFolder)
% analyzeMncavLateralDiagnosis Separate model, gain and input inconsistency.
% All perturbations are diagnostic; no production configuration is changed.
% INSPVA planar body velocity is the stipulated benchmark. Reference-fed
% controls are oracles, not deployable localization performance claims.
    arguments
        outputFolder (1,1) string="output/mncav_lateral_diagnosis_20260916"
    end
    setupVehicleLocalization();if ~isfolder(outputFolder),mkdir(outputFolder);end
    source="output/mncav_inspva_observer_20260915/experiment.mat";
    z=load(source,'high','lateralCfg','lateralDesign','lateral');
    h=z.high;t=h.time;cfg=z.lateralCfg;design=z.lateralDesign;n=numel(t);
    audit=readtable('output/mncav_inspva_median_20260915/motion_audit.csv');
    reference=interp1(audit.time,audit{:,{'insVx','insVy','insDerivedAx','insDerivedAy','insDerivedYawRate'}},t);
    assert(all(isfinite(reference),'all'));
    moving=h.longitudinalSpeed>=5;straight=moving&abs(h.yawRate)<.03;
    groups={true(n,1),moving,straight,moving&~straight,moving&t<=60,moving&t>60};
    groupNames=["all","moving","straight","turning","moving_first60","moving_later"];
    names=["actual","gain_0","gain_025","gain_05","gain_2","gain_4","gain_10", ...
        "master_gain_3","master_gain_48","master_bias_gain_0","no_dynamic_correction", ...
        "oracle_ay","oracle_rate","oracle_ay_rate","oracle_motion", ...
        "front_07","front_13","rear_07","rear_13","tires_07","tires_13","inertia_07","inertia_13", ...
        "steering_scale_07","steering_scale_13","tires_07_resynth","tires_13_resynth","weight_vy_100_resynth"];
    estimates=cell(numel(names),1);designs=cell(numel(names),1);rows=cell(0,12);
    baselineDifference=NaN;failures=struct('variant',{},'stage',{},'message',{});
    for j=1:numel(names)
        name=names(j);c=cfg;d=design;u=h;resynth=false;label="Diagnostic perturbation; certificate not transferred";
        switch name
            case "actual",label="Original certified design; grid certificate covers LPV branch only";
            case {"gain_0","gain_025","gain_05","gain_2","gain_4","gain_10"}
                factors=containers.Map({'gain_0','gain_025','gain_05','gain_2','gain_4','gain_10'},[0,.25,.5,2,4,10]);
                d.gains=factors(char(name))*d.gains;
            case "master_gain_3",c.hybrid.correction.dynamic.velocityGain=3;
            case "master_gain_48",c.hybrid.correction.dynamic.velocityGain=48;
            case "master_bias_gain_0",c.hybrid.correction.dynamic.biasGain=0;
            case "no_dynamic_correction"
                c.hybrid.correction.dynamic.velocityGain=0;c.hybrid.correction.dynamic.biasGain=0;
            case "oracle_ay",u.lateralAcceleration=reference(:,4);
            case "oracle_rate",u.yawRate=reference(:,5);
            case "oracle_ay_rate",u.lateralAcceleration=reference(:,4);u.yawRate=reference(:,5);
            case "oracle_motion"
                u.longitudinalSpeed=max(reference(:,1),0);u.longitudinalAcceleration=reference(:,3);
                u.lateralAcceleration=reference(:,4);u.yawRate=reference(:,5);
            case {"front_07","front_13","rear_07","rear_13","tires_07","tires_13","inertia_07","inertia_13","tires_07_resynth","tires_13_resynth"}
                factor=1.3;if contains(name,"07"),factor=.7;end
                if startsWith(name,"front")||startsWith(name,"tires"),c.vehicle.frontCorneringStiffness=factor*c.vehicle.frontCorneringStiffness;end
                if startsWith(name,"rear")||startsWith(name,"tires"),c.vehicle.rearCorneringStiffness=factor*c.vehicle.rearCorneringStiffness;end
                if startsWith(name,"inertia"),c.vehicle.yawInertia=factor*c.vehicle.yawInertia;end
                d.model=lateralBicycleModel(c.vehicle);resynth=endsWith(name,"resynth");
            case "steering_scale_07",u.steeringAngle=.7*h.steeringAngle;
            case "steering_scale_13",u.steeringAngle=1.3*h.steeringAngle;
            case "weight_vy_100_resynth",c.synthesis.errorWeight=diag([100,1]);resynth=true;
        end
        if resynth
            try
                d=designLateralObserverGains(c);assert(d.certified);
                label="Re-synthesized and grid-verified LPV design; hybrid not certified";
            catch failure
                failures(end+1)=struct('variant',name,'stage',"synthesis",'message',string(failure.message)); %#ok<AGROW>
                fprintf('%s: synthesis failed, retained without a replay claim: %s\n',name,failure.message);
                save(fullfile(outputFolder,'partial.mat'),'rows','estimates','designs','failures','names','j','baselineDifference','-v7.3');
                continue;
            end
        end
        timer=tic;est=runLateralVelocityObserver(u,d,c);runtime=toc(timer);
        if name=="actual"
            baselineDifference=max(abs(est.state-z.lateral.state),[],'all');assert(baselineDifference==0);
            assert(isequaln(est.dynamicState,z.lateral.dynamicState));
        end
        estimates{j}=est;designs{j}=d;
        for k=1:numel(groups)
            mask=groups{k};e=est.lateralVelocity(mask)-reference(mask,2);
            rows(end+1,:)={name,groupNames(k),nnz(mask),rms(e),median(abs(e)),mean(e), ...
                mean(est.lateralVelocity(mask)),mean(reference(mask,2)), ...
                rms(est.dynamicState(mask,1)-reference(mask,2)), ...
                mean(est.diagnostics.dynamicParticipation(mask)),runtime,label}; %#ok<AGROW>
        end
        fprintf('%s: moving RMS %.6f; straight bias %.6f m/s\n',name, ...
            rms(est.lateralVelocity(moving)-reference(moving,2)),mean(est.lateralVelocity(straight)-reference(straight,2)));
        save(fullfile(outputFolder,'partial.mat'),'rows','estimates','designs','failures','names','j','baselineDifference','-v7.3');
    end
    controls=cell2table(rows,VariableNames={'variant','population','samples','rmseMps','medianAbsMps','biasMps', ...
        'estimatedMeanMps','referenceMeanMps','dynamicBranchRmseMps','meanDynamicParticipation','runtimeSeconds','designStatus'});
    % Algebraic output inversion and reference substitution into the model.
    v=cfg.vehicle;sumC=v.frontCorneringStiffness+v.rearCorneringStiffness;
    moment=v.lr*v.rearCorneringStiffness-v.lf*v.frontCorneringStiffness;
    speed=max(h.longitudinalSpeed,5);r=h.yawRate;delta=h.steeringAngle;ay=h.lateralAcceleration;
    algebraic=(moment*r+v.frontCorneringStiffness*speed.*delta-v.mass*speed.*ay)/sumC;
    predictedAy=-sumC./(v.mass*speed).*reference(:,2)+moment./(v.mass*speed).*r+v.frontCorneringStiffness/v.mass*delta;
    neededSteering=(v.mass*ay+sumC./speed.*reference(:,2)-moment./speed.*r)/v.frontCorneringStiffness;
    referenceDerivative=[gradient(reference(:,2),t),gradient(reference(:,5),t)];
    referenceFlow=zeros(n,2);baselineFlow=zeros(n,2);referenceInnovation=zeros(n,2);
    nominalMismatch=zeros(n,2);nominalOutputMismatch=zeros(n,2);identityResidual=zeros(n,2);
    eigenMax=NaN(n,1);gainEntries=NaN(n,4);masterDifference=z.lateral.lateralVelocity-z.lateral.dynamicState(:,1);
    for i=find(moving).'
        [A,C]=evaluateLateralModel(design.model,[h.longitudinalSpeed(i);1/h.longitudinalSpeed(i)]);
        L=scheduleLateralObserverGain(design,h.longitudinalSpeed(i));
        x=reference(i,[2,5]).';y=[ay(i);r(i)];
        f=A*x+design.model.B*delta(i);pred=C*x+design.model.D*delta(i);
        referenceFlow(i,:)=(f+L*(y-pred)).';referenceInnovation(i,:)=(y-pred).';
        nominalMismatch(i,:)=f.'-referenceDerivative(i,:);
        nominalOutputMismatch(i,:)=pred.'-y.';
        identityResidual(i,:)=referenceFlow(i,:)-referenceDerivative(i,:)- ...
            (nominalMismatch(i,:)-(L*nominalOutputMismatch(i,:).').');
        xx=z.lateral.dynamicState(i,:).';baselineFlow(i,:)=(A*xx+design.model.B*delta(i)+L*(y-C*xx-design.model.D*delta(i))).';
        eigenMax(i)=max(real(eig(A-L*C)));gainEntries(i,:)=L(:).';
    end
    assert(max(abs(identityResidual(moving,:)),[],'all')<1e-10);
    % Check whether a correct initial vy survives the existing dynamics.
    start=find(t>=20,1);fields=fieldnames(h);segment=struct();
    for i=1:numel(fields),segment.(fields{i})=h.(fields{i})(start:end);end
    initialCfg=cfg;initialCfg.observer.initialState=reference(start,[2,5]).';
    initialCfg.hybrid.initialMasterState=[reference(start,2);0];
    initialized=runLateralVelocityObserver(segment,design,initialCfg);
    durations=[0,.25,.5,1,2,5];initialRows=zeros(numel(durations),4);
    for i=1:numel(durations)
        [~,q]=min(abs(segment.time-(segment.time(1)+durations(i))));
        initialRows(i,:)=[segment.time(q)-segment.time(1),initialized.lateralVelocity(q),reference(start+q-1,2), ...
            z.lateral.lateralVelocity(start+q-1)];
    end
    initialControl=array2table(initialRows,VariableNames={'elapsedSeconds','referenceInitializedVy','referenceVy','originalVy'});
    % Matched-model controls distinguish convergence from real model mismatch.
    syntheticCfg=cfg;syntheticCfg.simulation.lateralAccelerationNoiseStd=0;syntheticCfg.simulation.yawRateNoiseStd=0;
    synthetic=simulateLateralObserverScenario(design,syntheticCfg);post=synthetic.truth.time>=5;
    syntheticNoiseless=rms(synthetic.estimate.lateralVelocity(post)-synthetic.truth.lateralVelocity(post));
    assert(syntheticNoiseless<.01);
    syntheticNoisy=simulateLateralObserverScenario(design,cfg);post=syntheticNoisy.truth.time>=5;
    syntheticNoiseRms=rms(syntheticNoisy.estimate.lateralVelocity(post)-syntheticNoisy.truth.lateralVelocity(post));
    signal=array2table([t,h.longitudinalSpeed,reference(:,2),z.lateral.lateralVelocity,z.lateral.dynamicState(:,1), ...
        ay,reference(:,4),r,reference(:,5),delta,algebraic,predictedAy,neededSteering, ...
        z.lateral.lateralAccelerationBias,z.lateral.diagnostics.dynamicParticipation,referenceFlow,referenceDerivative, ...
        nominalMismatch,nominalOutputMismatch,eigenMax,gainEntries], ...
        VariableNames={'time','vx','referenceVy','estimatedVy','dynamicVy','measuredAy','referenceAy','measuredR','referenceR', ...
        'steering','algebraicVy','predictedAyAtReferenceVy','neededSteering','estimatedAyBias','dynamicParticipation', ...
        'observerFlowAtReferenceVy','observerFlowAtReferenceR','referenceVyDot','referenceRDot', ...
        'plantVyMismatch','plantRMismatch','outputAyMismatch','outputRMismatch','maxFrozenEigenvalue', ...
        'L11','L21','L12','L22'});
    report=struct('source',source,'samples',n,'durationSeconds',t(end),'controls',controls,'failures',failures, ...
        'baselineStateDifference',baselineDifference,'syntheticNoiselessPost5RmseMps',syntheticNoiseless, ...
        'syntheticNoisyPost5RmseMps',syntheticNoiseRms,'syntheticSeed',cfg.simulation.randomSeed, ...
        'nominalGridCertificatePassed',design.certified,'maxVisitedFrozenEigenvalue',max(eigenMax(moving)), ...
        'straightMasterMinusDynamicRmsMps',rms(masterDifference(straight)), ...
        'straightReferenceVyMeanMps',mean(reference(straight,2)), ...
        'straightAlgebraicVyMeanMps',mean(algebraic(straight)), ...
        'straightMeasuredAyMeanMps2',mean(ay(straight)), ...
        'straightModelAyAtReferenceVyMeanMps2',mean(predictedAy(straight)), ...
        'straightAyModelResidualRmsMps2',rms(predictedAy(straight)-ay(straight)), ...
        'straightOracleAyResidualRmsMps2',rms(ay(straight)-reference(straight,4)), ...
        'straightReferenceFlowMinusDerivativeMean',mean(referenceFlow(straight,:)-referenceDerivative(straight,:)), ...
        'straightMeanRequiredSteeringCorrectionDeg',rad2deg(mean(neededSteering(straight)-delta(straight))), ...
        'errorEquationIdentityResidual',max(abs(identityResidual(moving,:)),[],'all'), ...
        'factoryAnalyzerFindings',numel(checkcode(mfilename('fullpath'),'-config=factory','-id')), ...
        'scope',"Diagnostic controls only; benchmark INSPVA stipulated; no physical parameter identification or production changes");
    save(fullfile(outputFolder,'experiment.mat'),'report','estimates','designs','signal','initialControl', ...
        'synthetic','syntheticNoisy','initialized','reference','groups','groupNames','names','-v7.3');
    writetable(controls,fullfile(outputFolder,'controls.csv'));writetable(signal,fullfile(outputFolder,'signals.csv'));
    writetable(initialControl,fullfile(outputFolder,'initial_state_control.csv'));
    fid=fopen(fullfile(outputFolder,'summary.json'),'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));
    disp(controls(controls.population=="straight",[1,4,5,6,7,10]));disp(initialControl);
end
