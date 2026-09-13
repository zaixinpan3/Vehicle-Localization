function report=diagnoseStandaloneObserverConvergence(sourceFolder,outputFolder)
% diagnoseStandaloneObserverConvergence Separate initialization, noise and gain effects.
% Production ablations retain verified gains. Gain changes occur only in an
% explicitly local, zero-delay, straight-motion linearization, not the runtime.
    arguments
        sourceFolder (1,1) string="output/standalone_observer_20260913"
        outputFolder (1,1) string="output/observer_convergence_diagnosis_20260913"
    end
    setupVehicleLocalization;
    if ~isfolder(outputFolder),mkdir(outputFolder);end
    saved=load(fullfile(sourceFolder,'traces.mat'),'traces','design');
    base=saved.traces{6};exact=saved.traces{4};straight=saved.traces{1};
    names=["straight_biased","straight_exact_initial","maneuver_biased_no_noise", ...
        "maneuver_exact_initial_no_noise","maneuver_noisy_biased", ...
        "maneuver_noisy_exact_initial","maneuver_noisy_position_initial_corrected", ...
        "maneuver_pose_noise_only","maneuver_input_noise_only"];
    runs=cell(numel(names),1);rows=cell(numel(names),1);
    for k=1:numel(names)
        x=base;
        if k<=2
            x=straight;
        elseif k<=4 || k==8 || k==9
            x.sensorData.highRate=exact.sensorData.highRate;
            x.lateralInputs=exact.lateralInputs;
            if k~=8
                idx=round(x.sensorData.lidar.timestamp/.01)+1;
                x.sensorData.lidar.pose=x.truth.z(idx,[1,4,7]);
            end
            if k==9
                x.sensorData.highRate=base.sensorData.highRate;
                x.lateralInputs=base.lateralInputs;
            end
        end
        if any(k==[2,4,6]),x.cfg.observer.initialState=x.truth.z(1,:).';end
        if k==7,x.cfg.observer.initialState([1,4])=x.truth.z(1,[1,4]).';end
        e=runImprovedVehicleObserver(x.sensorData,struct(),saved.design,x.cfg,LateralInputs=x.lateralInputs);
        d=e.z-x.truth.z;d(:,7)=atan2(sin(d(:,7)),cos(d(:,7)));
        errors=[vecnorm(d(:,[1,4]),2,2),abs(rad2deg(d(:,7))), ...
            vecnorm(d(:,[2,5]),2,2),vecnorm(d(:,[3,6]),2,2)];
        rms=sqrt(mean(errors(x.truth.time>=10,:).^2));
        rows{k}=struct('name',names(k),'positionRmseM',rms(1),'yawRmseDeg',rms(2), ...
            'velocityRmseMps',rms(3),'accelerationRmseMps2',rms(4), ...
            'velocityErrorPeakMps',max(errors(:,3)),'accelerationErrorPeakMps2',max(errors(:,4)), ...
            'finalAccelerationErrorMps2',errors(end,4));
        runs{k}=struct('name',names(k),'estimate',e,'errors',errors);
    end
    % Analytic h Jacobian at true straight motion; measured course rate q=0.
    z=straight.truth.z(1,:).';N=diag(straight.cfg.observer.theta.^straight.cfg.observer.scalingExponents) ...
        *saved.design.N/straight.cfg.observer.theta^3;
    H=zeros(4,7);H(1,[2,5])=2*z([2,5]).';
    H(2,[2,3,5,6])=[z(3),z(2),z(6),z(5)];
    H(3,[2,3,5,6])=[z(6),-z(5),-z(3),z(2)];
    H(4,[2,5,7])=[-sin(z(7)),cos(z(7)),-z(2)*cos(z(7))-z(5)*sin(z(7))];
    A=blkdiag([0,1,0;0,0,1;0,0,0],[0,1,0;0,0,1;0,0,0],0);
    L=diag(straight.cfg.observer.theta.^straight.cfg.observer.scalingExponents)*saved.design.K;
    C=zeros(3,7);C(:,[1,4,7])=eye(3);
    J=straight.sensorData.lidar.information(:,:,1);W=(5*eye(3)+J)\J;
    scales=[.5,1,2,5,10];gainRows=cell(numel(scales),1);
    for k=1:numel(scales)
        candidate=L;candidate([3,6],:)=scales(k)*candidate([3,6],:);
        B=candidate*W*C;
        [~,phi]=ode45(@(t,p) reshape((A-N*H-expm(A*t)*B/expm(A*t)) ...
            *reshape(p,7,7),49,1),[0,.03],reshape(eye(7),49,1), ...
            odeset('RelTol',1e-10,'AbsTol',1e-12));
        periodMap=expm((A-N*H)*.07)*reshape(phi(end,:),7,7);
        radius=max(abs(eig(periodMap)));rate=log(radius)/.1;
        gainRows{k}=struct('accelerationRowScale',scales(k),'spectralRadius',radius, ...
            'dominantDecayRatePerSecond',rate,'dominantTimeConstantSeconds',-1/rate);
    end
    ablations=struct2table(vertcat(rows{:}));linearGainScreen=struct2table(vertcat(gainRows{:}));
    writetable(ablations,fullfile(outputFolder,'ablations.csv'));
    writetable(linearGainScreen,fullfile(outputFolder,'local_gain_screen.csv'));
    report=struct('ablations',vertcat(rows{:}),'localGainScreen',vertcat(gainRows{:}), ...
        'physicalPoseGain',L,'physicalInvariantGain',N,'matlabVersion',version, ...
        'gainScreenScope',"Local straight-motion zero-delay transported linearization with 30 ms on / 70 ms off; altered gains are not certified or deployed");
    report.baselineMaximumStateDifference=max(abs(runs{5}.estimate.z-base.estimate.z),[],'all');
    assert(report.baselineMaximumStateDifference<1e-12,'Baseline reproduction failed.');
    fid=fopen(fullfile(outputFolder,'summary.json'),'w');assert(fid>=0);
    cleanup=onCleanup(@() fclose(fid));fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));
    save(fullfile(outputFolder,'diagnosis.mat'),'runs','report','H','A','L','N','W');
    disp(ablations);disp(linearGainScreen);
end
