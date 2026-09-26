function report=diagnoseMncavReferenceResponse(outputFolder)
% diagnoseMncavReferenceResponse Separate reference-source and DDE effects.
% Pure INSPVA position is a diagnostic same-receiver reference, not truth.
    arguments
        outputFolder (1,1) string="output/mncav_error_diagnosis_20260914"
    end
    root=setupVehicleLocalization();s=load(fullfile(outputFolder,'ablations.mat'));
    assertMncavReplayCurrent(s.f.report.metadata.parameters);
    pva=readtable(fullfile(outputFolder,'pva_reference.csv'));
    parameters=fullfile(root,'output','mississippi_20240607_120931_20260907','vehicle_parameters.json');
    [~,reference]=prepareMncavObserverReplay(fullfile(fileparts(parameters),'sensors'),parameters);
    t=s.t;f=s.f;referencePose=f.referencePose;
    referencePose(:,1:2)=interp1(pva.time,[pva.x,pva.y],t,'linear');
    [~,idx]=ismember(t,f.lateral.time);
    li=struct('time',t,'lateralVelocity',f.lateral.lateralVelocity(idx), ...
        'sideSlipAngle',f.lateral.sideSlipAngle(idx),'sideSlipAngleRate',f.lateral.sideSlipAngleRate(idx));
    rows=s.report.metrics;runs=s.runs;
    names=["pva_pose","pva_pose_delay_zero","half_integration_step"];
    for k=1:numel(names)
        cfg=f.cfg;data=f.data;
        if names(k)=="pva_pose_delay_zero"
            cfg.measurement.fixedLidarDelay=0;data.lidar.delay=0;
        end
        if startsWith(names(k),"pva_pose")
            query=t-cfg.measurement.fixedLidarDelay;
            data.lidar.pose=[interp1(pva.time,[pva.x,pva.y],query,'linear'), ...
                interp1(reference.time,unwrap(reference.psi),query,'linear')];
        else
            cfg.measurement.maximumIntegrationStep=cfg.measurement.maximumIntegrationStep/2;
        end
        initial=cfg.observer.initialState;timer=tic;
        e=runImprovedVehicleObserver(data,struct(),f.design,cfg,LateralInputs=li,InitialHistory=@(~) initial);
        duration=toc(timer);error=e.pose-f.referencePose;error(:,3)=atan2(sin(error(:,3)),cos(error(:,3)));
        v=vecnorm(error(:,1:2),2,2);verification=verifyImprovedObserverDesign(f.design,cfg);
        row=struct('scenario',names(k),'positionRmseM',rms(v),'after5SecondsRmseM',rms(v(t>=5)), ...
            'positionP95M',prctile(v,95),'maximumPositionM',max(v),'headingRmseDeg',rad2deg(rms(error(:,3))), ...
            'seconds',duration,'certificateMargin',verification.uniformMargin, ...
            'maximumStateDifferenceFromStored',max(abs(e.z-f.estimate.z),[],'all'));
        rows(end+1)=row;runs{end+1}=struct('z',e.z,'error',error,'innovations',e.innovations); %#ok<AGROW>
        fprintf('%s: original reference %.6f m; PVA reference %.6f m\n',names(k),rms(v), ...
            sqrt(mean(sum((e.position-referencePose(:,1:2)).^2,2))));
    end
    for k=1:numel(rows)
        error=runs{k}.z(:,[1,4])-referencePose(:,1:2);
        rows(k).pvaPositionRmseM=sqrt(mean(sum(error.^2,2)));
        rows(k).pvaMaximumPositionM=max(vecnorm(error,2,2));
    end
    % Independent sinusoidal input validates the local analytic transfer gain.
    cfg=f.cfg;cfg.observer.initialState=zeros(7,1);design=f.design;design.N=zeros(7,4);
    high=struct('time',(0:.01:30).');
    for field=["longitudinalSpeed","steeringAngle","longitudinalAcceleration","lateralAcceleration","yawRate"]
        high.(field)=zeros(size(high.time));
    end
    li=struct('time',high.time,'lateralVelocity',zeros(size(high.time)), ...
        'sideSlipAngle',zeros(size(high.time)),'sideSlipAngleRate',zeros(size(high.time)));
    frequency=1.0953784;omega=2*pi*frequency;amplitude=.01;delay=cfg.measurement.fixedLidarDelay;
    lidar=struct('delay',delay,'headingConvention',"unwrapped", ...
        'evaluate',@(u) struct('pose',[amplitude*sin(omega*(u-delay));0;0],'information',1e6*eye(3)));
    e=runImprovedVehicleObserver(struct('highRate',high,'lidar',lidar),struct(),design,cfg, ...
        LateralInputs=li,InitialHistory=@(~) zeros(7,1));
    mask=high.time>=20;fit=[sin(omega*high.time(mask)),cos(omega*high.time(mask)),ones(nnz(mask),1)]\e.position(mask,1);
    theta=cfg.observer.theta;z=1i*omega;polynomial=3*theta*z^2+3*theta^2*z+1.5*theta^3;
    analytic=abs(polynomial*exp(-delay*z)/(z^3+polynomial*exp(-delay*z)));
    harmonic=struct('frequencyHz',frequency,'amplitudeM',amplitude,'durationSeconds',30, ...
        'fitStartSeconds',20,'analyticGain',analytic,'measuredGain',hypot(fit(1),fit(2))/amplitude, ...
        'relativeGainError',abs(hypot(fit(1),fit(2))/amplitude/analytic-1));
    assert(harmonic.relativeGainError<.002,'The independent harmonic check disagrees with the analytic gain.');
    writetable(struct2table(rows),fullfile(outputFolder,'complete_ablation_metrics.csv'));
    writetable(table(high.time,amplitude*sin(omega*high.time),e.position(:,1), ...
        VariableNames={'time','captureNoise','observerX'}),fullfile(outputFolder,'harmonic_response.csv'));
    report=struct('metrics',rows,'harmonic',harmonic,'referenceScope', ...
        "Same receiver INSPVA projected without fitted alignment; neither independent truth nor a rebuilt map.");
    fid=fopen(fullfile(outputFolder,'reference_response.json'),'w');assert(fid>=0);cleanup=onCleanup(@() fclose(fid));
    fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));
    save(fullfile(outputFolder,'reference_response.mat'),'report','runs','t','referencePose','-v7.3');
    disp(struct2table(rows));disp(harmonic);
end
