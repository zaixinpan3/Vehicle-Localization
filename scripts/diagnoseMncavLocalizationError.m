function report=diagnoseMncavLocalizationError(outputFolder)
% diagnoseMncavLocalizationError Controlled downstream error attribution.
% Matching outputs are frozen. Reference-pose replacements are diagnostic
% counterfactuals, never claimed as deployable measurement or performance.
    arguments
        outputFolder (1,1) string="output/mncav_error_diagnosis_20260914"
    end
    root=setupVehicleLocalization();if ~isfolder(outputFolder),mkdir(outputFolder);end
    baselineFolder=fullfile(root,'output','mncav_full_localization_20260914');
    f=load(fullfile(baselineFolder,'full_experiment.mat'));
    assertMncavReplayCurrent(f.report.metadata.parameters);
    calls=readtable(fullfile(baselineFolder,'matching','calls.csv'));a=calls.accepted==1;
    poseTime=calls.timeSeconds(a);pose=[calls.x(a),calls.y(a),calls.psi(a)];
    c=calls(a,:);information=zeros(3,3,height(c));
    for k=1:height(c)
        information(:,:,k)=[c.informationXX(k),c.informationXY(k),c.informationXPsi(k); ...
            c.informationXY(k),c.informationYY(k),c.informationYPsi(k); ...
            c.informationXPsi(k),c.informationYPsi(k),c.informationPsiPsi(k)];
    end
    parameters=fullfile(root,'output','mississippi_20240607_120931_20260907','vehicle_parameters.json');
    [~,reference]=prepareMncavObserverReplay(fullfile(fileparts(parameters),'sensors'),parameters);
    referenceRaw=[reference.x,reference.y,unwrap(reference.psi)];
    t=f.data.highRate.time;[~,idx]=ismember(t,f.lateral.time);
    lateral=struct('time',t,'lateralVelocity',f.lateral.lateralVelocity(idx), ...
        'sideSlipAngle',f.lateral.sideSlipAngle(idx),'sideSlipAngleRate',f.lateral.sideSlipAngleRate(idx));
    names=["baseline","reference_pose","zero_auxiliary","zero_lateral", ...
        "delay_zero","delay_075ms","theta_15","reference_pose_delay_zero"];
    runs=cell(numel(names),1);rows=cell(numel(names),1);configs=cell(numel(names),1);
    for trial=1:numel(names)
        name=names(trial);cfg=f.cfg;design=f.design;data=f.data;li=lateral;
        if name=="zero_auxiliary",design.N=zeros(7,4);end
        if name=="zero_lateral"
            li.lateralVelocity(:)=0;li.sideSlipAngle(:)=0;li.sideSlipAngleRate(:)=0;
        end
        if any(name==["delay_zero","reference_pose_delay_zero"]),cfg.measurement.fixedLidarDelay=0;end
        if name=="delay_075ms",cfg.measurement.fixedLidarDelay=.075;end
        if name=="theta_15"
            cfg.observer.theta=1.5;design=designImprovedObserverGains(cfg);
        end
        if any(name==["delay_zero","delay_075ms","reference_pose_delay_zero"])
            data=reconstructContinuousObserverSignals(f.data.highRate,poseTime,pose,information,cfg,MaximumGap=1);
        end
        if any(name==["reference_pose","reference_pose_delay_zero"])
            data.lidar.pose=interp1(reference.time,referenceRaw,t-cfg.measurement.fixedLidarDelay,'linear');
        end
        verification=verifyImprovedObserverDesign(design,cfg);assert(verification.certified);
        initial=cfg.observer.initialState;timer=tic;
        e=runImprovedVehicleObserver(data,struct(),design,cfg,LateralInputs=li,InitialHistory=@(~) initial);
        duration=toc(timer);error=e.pose-f.referencePose;error(:,3)=atan2(sin(error(:,3)),cos(error(:,3)));
        v=vecnorm(error(:,1:2),2,2);mask=t>=5;
        rows{trial}=struct('scenario',name,'positionRmseM',rms(v),'after5SecondsRmseM',rms(v(mask)), ...
            'positionP95M',prctile(v,95),'maximumPositionM',max(v),'headingRmseDeg',rad2deg(rms(error(:,3))), ...
            'seconds',duration,'certificateMargin',verification.uniformMargin, ...
            'maximumStateDifferenceFromStored',max(abs(e.z-f.estimate.z),[],'all'));
        runs{trial}=struct('z',e.z,'error',error,'innovations',e.innovations);configs{trial}=cfg;
        fprintf('%s: position %.6f m, heading %.6f deg, %.1f s\n',name,rms(v),rad2deg(rms(error(:,3))),duration);
        writetable(struct2table(vertcat(rows{1:trial})),fullfile(outputFolder,'ablation_metrics.csv'));
    end
    metrics=struct2table(vertcat(rows{:}));
    signal=runs{2}.error(:,1:2);measurement=runs{1}.error(:,1:2)-signal;
    decomposition=struct('baselineMse',mean(sum(runs{1}.error(:,1:2).^2,2)), ...
        'referencePoseResidualMse',mean(sum(signal.^2,2)), ...
        'measurementInducedMse',mean(sum(measurement.^2,2)), ...
        'crossTerm',2*mean(sum(signal.*measurement,2)));
    report=struct('metrics',table2struct(metrics),'decomposition',decomposition, ...
        'scope',"Controlled offline downstream ablations on frozen real matching; reference substitutions are diagnostic only.");
    fid=fopen(fullfile(outputFolder,'summary.json'),'w');assert(fid>=0);cleanup=onCleanup(@() fclose(fid));
    fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));
    save(fullfile(outputFolder,'ablations.mat'),'report','runs','configs','t','f','-v7.3');
    disp(metrics);disp(decomposition);
end
