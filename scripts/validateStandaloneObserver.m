function report = validateStandaloneObserver(outputFolder)
% validateStandaloneObserver Test the production global observer in isolation.
% Analytic motion and independent quadrature supply truth. No lateral observer,
% perception, map, registration, GPS or recorded dataset is used. All synthetic
% pose events retain the production information, pulse and delay processing.
    arguments
        outputFolder (1,1) string = "output/standalone_observer_20260913"
    end
    setupVehicleLocalization;
    if ~isfolder(outputFolder), mkdir(outputFolder); end
    previousRng=rng; cleanup=onCleanup(@() rng(previousRng));
    cfg=improvedObserverConfig;
    design=improvedObserverReferenceDesign(cfg);
    cases=struct('name',{},'motion',{},'noise',{},'delay',{},'outage',{},'weak',{},'seed',{});
    names=["straight_ideal","circle_ideal","stationary_ideal","maneuver_ideal", ...
        "circle_delayed","maneuver_noisy","maneuver_outage","maneuver_weak"];
    motions=["straight","circle","stationary","maneuver","circle", ...
        "maneuver","maneuver","maneuver"];
    for k=1:numel(names)
        cases(k)=struct('name',names(k),'motion',motions(k),'noise',k>=6, ...
            'delay',.15*(k>=5),'outage',k==7,'weak',k==8,'seed',20260913);
    end
    for seed=20260914:20260917
        cases(end+1)=struct('name',"maneuver_seed_"+seed,'motion',"maneuver", ...
            'noise',true,'delay',.15,'outage',false,'weak',false,'seed',seed); %#ok<AGROW>
    end
    rows=cell(numel(cases),1); traces=cell(numel(cases),1);
    for k=1:numel(cases)
        c=cases(k);rng(c.seed,'twister');runCfg=cfg;
        runCfg.measurement.fixedLidarDelay=c.delay;
        [truth,data,lateral]=makeScenario(c);
        runCfg.observer.initialState=truth.z(1,:).'+[2;-1;.5;-1.5;.8;-.3;deg2rad(15)];
        started=tic;
        estimate=runImprovedVehicleObserver(data,struct(),design,runCfg,LateralInputs=lateral);
        elapsed=toc(started);
        row=scoreRun(truth,estimate,c);row.runtimeSeconds=elapsed;
        rows{k}=row;
        traces{k}=struct('truth',truth,'sensorData',data,'lateralInputs',lateral, ...
            'estimate',estimate,'cfg',runCfg,'case',c);
        fprintf('%s: position %.5f m, yaw %.5f deg, pass=%d\n', ...
            c.name,row.positionRmseM,row.headingRmseDeg,row.trackingPass);
    end
    % Same sensor samples and noise: isolate integration-step sensitivity.
    baseline=traces{6};fineCfg=baseline.cfg;fineCfg.measurement.maximumIntegrationStep=.005;
    fine=runImprovedVehicleObserver(baseline.sensorData,struct(),design,fineCfg, ...
        LateralInputs=baseline.lateralInputs);
    delta=fine.z-baseline.estimate.z;delta(:,7)=atan2(sin(delta(:,7)),cos(delta(:,7)));
    refinement=struct('coarseStepSeconds',.01,'fineStepSeconds',.005, ...
        'maximumPositionDifferenceM',max(vecnorm(delta(:,[1,4]),2,2)), ...
        'maximumHeadingDifferenceDeg',rad2deg(max(abs(delta(:,7)))), ...
        'maximumVelocityDifferenceMps',max(vecnorm(delta(:,[2,5]),2,2)), ...
        'maximumAccelerationDifferenceMps2',max(vecnorm(delta(:,[3,6]),2,2)));
    refinement.pass=refinement.maximumPositionDifferenceM<.001 ...
        && refinement.maximumHeadingDifferenceDeg<.01;
    % A negative control exposes the need for absolute position observations.
    openData=baseline.sensorData;openData=rmfield(openData,'lidar');
    open=runImprovedVehicleObserver(openData,struct(),design,baseline.cfg, ...
        LateralInputs=baseline.lateralInputs);
    openCase=baseline.case;openCase.name="maneuver_without_pose_control";
    negativeControl=scoreRun(baseline.truth,open,openCase);
    rows=vertcat(rows{:});summary=struct2table(rows);writetable(summary,fullfile(outputFolder,'metrics.csv'));
    report=struct('matlabVersion',version,'cases',cases,'metrics',rows, ...
        'refinement',refinement,'negativeControl',negativeControl, ...
        'allTrackingPassed',all([rows.trackingPass]),'allFinite',all([rows.finite]), ...
        'certified',false,'designTheta',cfg.observer.theta, ...
        'scope',"Synthetic validation of the production seven-state observer with supplied lateral inputs");
    fid=fopen(fullfile(outputFolder,'summary.json'),'w');assert(fid>=0);
    fileCleanup=onCleanup(@() fclose(fid));fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));
    save(fullfile(outputFolder,'traces.mat'),'traces','fine','open','report','design','-v7.3');
    plotResults(traces,outputFolder);
    disp(summary(:,{'name','positionRmseM','headingRmseDeg','velocityRmseMps','accelerationRmseMps2','trackingPass'}));
    disp(refinement);
end

function [truth,data,lateral]=makeScenario(c)
    time=(0:.01:30).';motion=kinematics(time,c.motion);
    % Tight-tolerance independent integration; never use observer dynamics.
    [~,position]=ode45(@(t,~) velocityAt(t,c.motion),time,[2;-1], ...
        odeset('RelTol',1e-11,'AbsTol',1e-12));
    velocity=motion.speed.*[cos(motion.course),sin(motion.course)];
    acceleration=motion.speedDot.*[cos(motion.course),sin(motion.course)] ...
        +motion.speed.*motion.q.*[-sin(motion.course),cos(motion.course)];
    bodySpeed=motion.speed.*cos(motion.beta);bodyLateral=motion.speed.*sin(motion.beta);
    bodyAx=acceleration(:,1).*cos(motion.psi)+acceleration(:,2).*sin(motion.psi);
    bodyAy=-acceleration(:,1).*sin(motion.psi)+acceleration(:,2).*cos(motion.psi);
    z=[position(:,1),velocity(:,1),acceleration(:,1),position(:,2), ...
        velocity(:,2),acceleration(:,2),motion.psi];
    % Exact omitted derivative terms: v'' e_course + v q' J e_course.
    modelDisturbance=hypot(motion.speedDDot,motion.speed.*motion.qDot);
    truth=struct('time',time,'z',z,'position',position,'modelDisturbance',modelDisturbance);
    n=numel(time);noise=double(c.noise);
    data.highRate=struct('time',time,'steeringAngle',zeros(n,1), ...
        'longitudinalSpeed',max(0,bodySpeed+noise*.02*boundedNoise(n,1)), ...
        'longitudinalAcceleration',bodyAx+noise*.03*boundedNoise(n,1), ...
        'lateralAcceleration',bodyAy+noise*.03*boundedNoise(n,1), ...
        'yawRate',motion.r+noise*.002*boundedNoise(n,1));
    lateral=struct('time',time,'lateralVelocity',bodyLateral+noise*.02*boundedNoise(n,1), ...
        'sideSlipAngle',motion.beta+noise*deg2rad(.1)*boundedNoise(n,1), ...
        'sideSlipAngleRate',motion.betaDot+noise*.001*boundedNoise(n,1));
    idx=(1:10:n).';
    if c.outage,idx=idx(time(idx)<12 | time(idx)>=15);end
    idx=idx(time(idx)+c.delay<=time(end));
    pose=z(idx,[1,4,7])+noise*boundedNoise(numel(idx),3).*[.05,.05,deg2rad(.5)];
    pose(:,3)=atan2(sin(pose(:,3)),cos(pose(:,3)));
    information=repmat(diag([1/.05^2,1/.05^2,1/deg2rad(.5)^2]),1,1,numel(idx));
    if c.weak
        for k=1:numel(idx)
            if time(idx(k))>=12 && time(idx(k))<18
                angle=.7;R=[cos(angle),-sin(angle),0;sin(angle),cos(angle),0;0,0,1];
                information(:,:,k)=R*diag([.2,400,1/deg2rad(.5)^2])*R.';
            end
        end
    end
    data.lidar=struct('timestamp',time(idx),'arrivalTime',time(idx)+c.delay, ...
        'pose',pose,'information',information);
end

function m=kinematics(t,kind)
    one=ones(size(t));zero=zeros(size(t));
    m=struct('speed',8*one,'speedDot',zero,'speedDDot',zero, ...
        'psi',.3*one,'r',zero,'beta',zero,'betaDot',zero,'qDot',zero);
    switch kind
        case "circle"
            m.psi=.3+.15*t;m.r=.15*one;m.beta=.03*one;
        case "stationary"
            m.speed=zero;
        case "maneuver"
            m.speed=8+1.2*sin(.35*t);m.speedDot=.42*cos(.35*t);
            m.speedDDot=-.147*sin(.35*t);
            m.psi=.3+.12*t+.25*sin(.4*t);m.r=.12+.1*cos(.4*t);
            m.beta=.03*sin(.5*t);m.betaDot=.015*cos(.5*t);
            m.qDot=-.04*sin(.4*t)-.0075*sin(.5*t);
    end
    m.course=m.psi+m.beta;m.q=m.r+m.betaDot;
end

function v=velocityAt(t,kind)
    m=kinematics(t,kind);v=m.speed*[cos(m.course);sin(m.course)];
end

function values=boundedNoise(n,m)
    values=min(max(randn(n,m),-3),3);
end

function row=scoreRun(truth,estimate,c)
    error=estimate.z-truth.z;error(:,7)=atan2(sin(error(:,7)),cos(error(:,7)));
    p=vecnorm(error(:,[1,4]),2,2);yaw=abs(rad2deg(error(:,7)));
    v=vecnorm(error(:,[2,5]),2,2);a=vecnorm(error(:,[3,6]),2,2);
    scoring=truth.time>=10;tail=truth.time>=25;
    good=p<.1 & yaw<1;lastBad=find(~good,1,'last');settling=NaN;
    if isempty(lastBad),settling=0;elseif lastBad<numel(p),settling=truth.time(lastBad+1);end
    ideal=~c.noise && c.motion~="maneuver";
    limits=[.5,2,.5,1.5];if ideal,limits=[.01,.01,.01,.01];end
    metrics=[sqrt(mean(p(scoring).^2)),sqrt(mean(yaw(scoring).^2)), ...
        sqrt(mean(v(scoring).^2)),sqrt(mean(a(scoring).^2))];
    row=struct('name',c.name,'seed',c.seed,'finite',all(isfinite(estimate.z),'all'), ...
        'positionRmseM',metrics(1),'headingRmseDeg',metrics(2), ...
        'velocityRmseMps',metrics(3),'accelerationRmseMps2',metrics(4), ...
        'fullPositionPeakM',max(p),'fullHeadingPeakDeg',max(yaw), ...
        'fullVelocityPeakMps',max(v),'fullAccelerationPeakMps2',max(a), ...
        'post10PositionPeakM',max(p(scoring)),'post10HeadingPeakDeg',max(yaw(scoring)), ...
        'tailPositionRmseM',sqrt(mean(p(tail).^2)), ...
        'tailHeadingRmseDeg',sqrt(mean(yaw(tail).^2)), ...
        'tailVelocityRmseMps',sqrt(mean(v(tail).^2)), ...
        'tailAccelerationRmseMps2',sqrt(mean(a(tail).^2)), ...
        'positionRmseLimitM',limits(1),'headingRmseLimitDeg',limits(2), ...
        'velocityRmseLimitMps',limits(3),'accelerationRmseLimitMps2',limits(4), ...
        'settlingToPoint1MAnd1DegS',settling, ...
        'maximumModelDisturbanceMps3',max(truth.modelDisturbance), ...
        'acceptedPoses',nnz(estimate.diagnostics.acceptedLidar), ...
        'rejectedPoses',estimate.diagnostics.rejectedEventCount(end), ...
        'outsideEnvelopeSamples',nnz(estimate.diagnostics.invariantExtensionActive), ...
        'trackingPass',all(metrics<limits),'runtimeSeconds',0);
end

function plotResults(traces,folder)
    f=figure('Visible','off','Color','w','Position',[100,100,1200,850]);
    cleanup=onCleanup(@() close(f));layout=tiledlayout(f,2,2);
    chosen=[2,6,7,8];labels=["Ideal circle","Noisy maneuver","3 s pose outage","Weak pose direction"];
    for j=1:numel(chosen)
        x=traces{chosen(j)};err=x.estimate.z-x.truth.z;
        nexttile(layout);yyaxis left;
        plot(x.truth.time,vecnorm(err(:,[1,4]),2,2),'LineWidth',1.3);ylabel('Position error (m)');
        yyaxis right;plot(x.truth.time,abs(rad2deg(atan2(sin(err(:,7)),cos(err(:,7))))),'LineWidth',1.1);
        ylabel('Absolute yaw error (deg)');xlabel('Time (s)');title(labels(j));grid on;
    end
    title(layout,'Standalone production observer: biased initialization, synthetic inputs');
    exportgraphics(f,fullfile(folder,'errors.png'),'Resolution',160);
    exportgraphics(f,fullfile(folder,'errors.pdf'),'ContentType','vector');
    f2=figure('Visible','off','Color','w','Position',[100,100,1000,800]);
    cleanup2=onCleanup(@() close(f2));x=traces{6};layout2=tiledlayout(f2,2,2);
    nexttile(layout2);plot(x.truth.z(:,1),x.truth.z(:,4),'k-',x.estimate.z(:,1),x.estimate.z(:,4),'--');
    axis equal;grid on;xlabel('X (m)');ylabel('Y (m)');title('Noisy maneuver trajectory');legend('Truth','Estimate');
    nexttile(layout2);plot(x.truth.time,rad2deg(unwrap(x.truth.z(:,7))),'k-', ...
        x.truth.time,rad2deg(unwrap(x.estimate.z(:,7))),'--');grid on;xlabel('Time (s)');ylabel('Yaw (deg)');
    nexttile(layout2);plot(x.truth.time,x.truth.z(:,[2,5]),'-',x.truth.time,x.estimate.z(:,[2,5]),'--');
    grid on;xlabel('Time (s)');ylabel('Global velocity (m/s)');legend('True Vx','True Vy','Estimated Vx','Estimated Vy');
    nexttile(layout2);plot(x.truth.time,x.truth.z(:,[3,6]),'-',x.truth.time,x.estimate.z(:,[3,6]),'--');
    grid on;xlabel('Time (s)');ylabel('Global acceleration (m/s^2)');legend('True Ax','True Ay','Estimated Ax','Estimated Ay');
    exportgraphics(f2,fullfile(folder,'states.png'),'Resolution',160);
    exportgraphics(f2,fullfile(folder,'states.pdf'),'ContentType','vector');
end
