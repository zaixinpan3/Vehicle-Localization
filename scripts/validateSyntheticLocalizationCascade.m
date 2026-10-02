function report=validateSyntheticLocalizationCascade(outputFolder,options)
% validateSyntheticLocalizationCascade Exercise the complete observer cascade.
% A 2-DOF bicycle plant with analytic speed and steering generates the truth.
% Synthetic wheel speed, steering, IMU, 20 Hz receiver positions and jittered
% 10 Hz LiDAR poses then pass through the production chain: hybrid LPV
% lateral observer -> frame-clock synchronization -> synchronous seven-state
% observer (MnCAV configuration, receiver output-point correction, optional
% online matcher callback). GNSS corrects position only; heading is the gyro
% corrected by LiDAR yaw.
% Scenarios separate convention errors (clean signals), noise/bias accuracy,
% source availability, closed-loop matching and lateral-model mismatch.
% All signals are synthetic; no recorded data or reference trajectory is read.
% NoiseOverrides replaces named noise settings for ablations; Scenarios
% selects a subset (pass/fail checks then need the complete default set).
    arguments
        outputFolder (1,1) string="output/synthetic_localization_cascade_20260929"
        options.NoiseOverrides (1,1) struct=struct()
        options.Scenarios (1,:) string=strings(1,0)
    end
    setupVehicleLocalization();if ~isfolder(outputFolder),mkdir(outputFolder);end
    lateralCfg=lateralObserverConfig("mncav");design=designLateralObserverGains(lateralObserverConfig());
    cfg=mncavFullObserverConfig();settings=scenarioSettings(lateralCfg,cfg);
    for name=string(fieldnames(options.NoiseOverrides)).'
        assert(isfield(settings.noise,name),'VehicleLocalization:UnknownNoiseSetting','Unknown noise setting %s.',name);
        settings.noise.(name)=options.NoiseOverrides.(name);
    end
    nominal=simulateTruth(design.model,settings);
    mismatchVehicle=lateralCfg.vehicle;
    mismatchVehicle.frontCorneringStiffness=settings.mismatch.front*mismatchVehicle.frontCorneringStiffness;
    mismatchVehicle.rearCorneringStiffness=settings.mismatch.rear*mismatchVehicle.rearCorneringStiffness;
    mismatch=simulateTruth(lateralBicycleModel(mismatchVehicle),settings);
    definitions={ ...
        "clean_both_truth_init",nominal,"clean","truth",["gnss","lidar"],[]; ...
        "clean_dead_reckoning",nominal,"clean","truth",strings(1,0),[]; ...
        "noisy_both",nominal,"noisy","perturbed",["gnss","lidar"],[]; ...
        "noisy_gnss_only",nominal,"noisy","perturbed","gnss",[]; ...
        "noisy_lidar_only",nominal,"noisy","perturbed","lidar",[]; ...
        "gnss_outage_40_60",nominal,"noisy","perturbed",["gnss","lidar"],struct('gnss',[40,60]); ...
        "lidar_outage_40_60",nominal,"noisy","perturbed",["gnss","lidar"],struct('lidar',[40,60]); ...
        "both_outage_40_50",nominal,"noisy","perturbed",["gnss","lidar"],struct('gnss',[40,50],'lidar',[40,50]); ...
        "alternating_1s",nominal,"noisy","perturbed",["gnss","lidar"],"alternating"; ...
        "online_matcher",nominal,"noisy","perturbed",["gnss","matcher"],[]; ...
        "plant_mismatch_both",mismatch,"noisy","perturbed",["gnss","lidar"],[]};
    if ~isempty(options.Scenarios)
        selected=ismember([definitions{:,1}],options.Scenarios);
        assert(nnz(selected)==numel(unique(options.Scenarios)),'VehicleLocalization:UnknownScenario','Unknown scenario name.');
        definitions=definitions(selected,:);
    end
    runs=cell(size(definitions,1),1);rows=cell(0,17);
    for k=1:size(definitions,1)
        [name,truth,noise,init,sources,mask]=definitions{k,:};
        signals=synthesizeSignals(truth,settings,noise);
        timer=tic;lateral=runLateralVelocityObserver(signals.highRate,design,lateralCfg);lateralSeconds=toc(timer);
        data=struct('highRate',signals.highRate,'gnss',signals.gnss,'lidar',signals.lidar);
        [aligned,alignedLateral,synchronization]=synchronizeLocalizationInputs(data,lateral,cfg);
        t=aligned.highRate.time;frameTruth=truthAt(truth,t);
        current=aligned;
        if ~ismember("gnss",sources),current=rmfield(current,'gnss');end
        if ~ismember("lidar",sources),current=rmfield(current,'lidar');end
        if isstruct(mask)
            for source=string(fieldnames(mask)).'
                window=mask.(source);current.(source).valid(t>=window(1) & t<window(2))=false;
            end
        elseif isstring(mask) && mask=="alternating"
            current.gnss.valid=current.gnss.valid & mod(floor(t),2)==0;
            current.lidar.valid=current.lidar.valid & mod(floor(t),2)==1;
        end
        matcherLog=[];
        if ismember("matcher",sources)
            matcherLog=matcherState(numel(t));
            current.lidarMatcher=@(i,seed,aid) syntheticMatcher(i,seed,aid,aligned.lidar,frameTruth,settings.matcher,matcherLog);
        end
        runCfg=cfg;runCfg.initialState=initialState(frameTruth,aligned.highRate,alignedLateral,settings,init);
        timer=tic;estimate=runFullLocalizationObserver(current,struct(),runCfg,LateralInputs=alignedLateral);seconds=toc(timer);
        metrics=scoreRun(estimate,frameTruth,truth,lateral,current,settings,mask);
        if ~isempty(matcherLog),metrics=addMatcherMetrics(metrics,matcherLog,t,settings);end
        runs{k}=struct('scenario',name,'truthModel',string(ifelse(truth.mismatch,"mismatch","nominal")), ...
            'noise',noise,'initialization',init,'sources',sources,'estimate',estimate,'metrics',metrics, ...
            'synchronization',synchronization,'lateralSeconds',lateralSeconds,'observerSeconds',seconds, ...
            'frameTruth',frameTruth,'lateral',lateral);
        rows(end+1,:)={name,noise,init,strjoin(sources,"+"),numel(t),metrics.positionRmseM, ...
            metrics.settledPositionRmseM,metrics.settledPositionP95M,metrics.positionMaximumAfterSettleM, ...
            metrics.settledHeadingRmseDeg,metrics.settledSpeedRmseMps,metrics.settledLateralVelocityRmseMps, ...
            metrics.convergenceTimeS,metrics.outageMaximumM,metrics.recoveryTimeS, ...
            metrics.rawGnssRmseM,metrics.rawLidarRmseM}; %#ok<AGROW>
        fprintf('%-24s settled position RMSE %.4f m, heading %.3f deg, lateral vy %.4f m/s\n', ...
            name,metrics.settledPositionRmseM,metrics.settledHeadingRmseDeg,metrics.settledLateralVelocityRmseMps);
    end
    table_=cell2table(rows,VariableNames={'scenario','noise','initialization','sources','frames', ...
        'positionRmseM','settledPositionRmseM','settledPositionP95M','settledPositionMaximumM', ...
        'settledHeadingRmseDeg','settledSpeedRmseMps','settledLateralVelocityRmseMps', ...
        'lastTenCmExceedanceS','outageMaximumM','recoveryTimeS', ...
        'rawGnssRmseM','rawLidarRmseM'});
    writetable(table_,fullfile(outputFolder,'metrics.csv'));
    if ~isempty(options.Scenarios)
        report=struct('settings',rmfield(settings,'steering'),'metrics',table_);
        save(fullfile(outputFolder,'experiment.mat'),'report','runs','settings','-v7.3');return;
    end
    checks=evaluateChecks(runs,settings);writetable(checks,fullfile(outputFolder,'checks.csv'));
    plotRuns(runs,outputFolder);
    report=struct('settings',rmfield(settings,'steering'),'metrics',table_,'checks',checks, ...
        'allChecksPassed',all(checks.passed),'lateralDesignCertified',design.certified, ...
        'lateralVertexGainNorms',arrayfun(@(k) norm(design.vertexGains(:,:,k)),1:size(design.vertexGains,3)), ...
        'observerKind',cfg.kind,'scope',"Synthetic signals only; truth plant equals the lateral design model except plant_mismatch_both. Not a recorded-data or hardware validation.");
    save(fullfile(outputFolder,'experiment.mat'),'report','runs','settings','design','-v7.3');
    writeJson(fullfile(outputFolder,'summary.json'),rmfield(report,'metrics'));
    disp(checks);
end

function s=scenarioSettings(lateralCfg,cfg)
    s=struct('duration',90,'plantStep',.001,'motionStep',.01, ...
        'outputForwardOffsetM',lateralCfg.outputPoint.forwardOffsetM, ...
        'receiverBodyOffsetM',cfg.gnss.outputPoint.bodyOffset(:), ...
        'speed',struct('mean',11,'slowAmplitude',4,'slowPeriod',60,'fastAmplitude',1,'fastPeriod',13), ...
        'steering',struct('timeS',[0,5,8,14,17,22,25,27,29,31,33,36,42,45,51,54,60,64,76,80,90], ...
            'roadWheelDeg',[0,0,2.2,2.2,0,0,1.2,-1.2,1.2,-1.2,0,0,0,-2.5,-2.5,0,0,1,1,0,0]), ...
        'mismatch',struct('front',.85,'rear',1.15), ...
        'seed',20260929,'settleS',5,'headingSettleS',10);
    % Noise priors follow mncavSensorParameters.json where one exists; the
    % remaining values are declared engineering stress values.
    sensors=mncavSensorConfig();
    s.noise=struct('wheelScaleError',.005,'wheelSpeedStdMps',.02,'steeringStdRad',5e-4, ...
        'longitudinalAccelerationStdMps2',.05, ...
        'lateralAccelerationStdMps2',sensors.lateralSimulation.lateralAccelerationNoiseStdMps2, ...
        'lateralAccelerationBiasMps2',.1,'yawRateStdRadps',sensors.lateralSimulation.yawRateNoiseStdRadps, ...
        'yawRateBiasRadps',.001,'gnssWhiteStdM',.02,'gnssMarkovStdM',.03,'gnssMarkovTimeS',20, ...
        'lidarAlongStdM',.10,'lidarCrossStdM',.04,'lidarYawStdRad',deg2rad(.3), ...
        'lidarRejectFraction',.05,'lidarJitterS',.003);
    s.gnss=struct('periodS',.05,'offsetS',.013);s.lidar=struct('periodS',.1,'offsetS',.05);
    s.initialError=struct('positionM',[1.5;-1.0],'headingRad',deg2rad(3));
    s.matcher=struct('captureRadiusM',1.0,'captureYawRad',deg2rad(3));
    s.outageRecoveryToleranceM=.15;s.convergenceToleranceM=.10;
end

function truth=simulateTruth(model,s)
    h=s.plantStep;t=(0:h:s.duration).';n=numel(t);x=zeros(n,5);d=s.outputForwardOffsetM;
    % Plant state [vy at the IMU/model origin; r; X; Y; psi], with X,Y at the
    % INSPVA output point that the global observer, GNSS correction and map share.
    f=@(time,z) plant(time,z,model,s,d);
    for k=1:n-1
        k1=f(t(k),x(k,:).');k2=f(t(k)+h/2,x(k,:).'+h/2*k1);
        k3=f(t(k)+h/2,x(k,:).'+h/2*k2);k4=f(t(k+1),x(k,:).'+h*k3);
        x(k+1,:)=x(k,:)+h/6*(k1+2*k2+2*k3+k4).';
    end
    [vx,ax]=speedProfile(t,s);delta=steering(t,s);rates=zeros(n,2);
    for k=1:n
        A=model.A0+vx(k)*model.A1+model.A2/vx(k);rates(k,:)=(A*x(k,1:2).'+model.B*delta(k)).';
    end
    vyOut=x(:,1)-d*x(:,2);vyOutRate=rates(:,1)-d*rates(:,2);
    truth=struct('time',t,'longitudinalSpeed',vx,'speedRate',ax,'steeringAngle',delta, ...
        'imuLateralVelocity',x(:,1),'yawRate',x(:,2),'lateralVelocity',vyOut, ...
        'position',x(:,3:4),'heading',x(:,5), ...
        'longitudinalSpecificForce',ax-x(:,1).*x(:,2), ...
        'lateralSpecificForce',rates(:,1)+vx.*x(:,2), ...
        'outputAcceleration',[ax-vyOut.*x(:,2),vyOutRate+vx.*x(:,2)], ...
        'sideSlipAngleRate',(vyOutRate.*vx-vyOut.*ax)./(vx.^2+vyOut.^2), ...
        'mismatch',~isequal(model.A2,lateralBicycleModel(lateralObserverConfig().vehicle).A2));
end

function dz=plant(t,z,model,s,d)
    [vx,~]=speedProfile(t,s);delta=steering(t,s);
    A=model.A0+vx*model.A1+model.A2/vx;lat=A*z(1:2)+model.B*delta;
    c=cos(z(5));si=sin(z(5));vyOut=z(1)-d*z(2);
    dz=[lat;c*vx-si*vyOut;si*vx+c*vyOut;z(2)];
end

function [v,a]=speedProfile(t,s)
    p=s.speed;w1=2*pi/p.slowPeriod;w2=2*pi/p.fastPeriod;
    v=p.mean-p.slowAmplitude*cos(w1*t)+p.fastAmplitude*sin(w2*t);
    a=p.slowAmplitude*w1*sin(w1*t)+p.fastAmplitude*w2*cos(w2*t);
end

function delta=steering(t,s)
    wt=s.steering.timeS(:);wd=deg2rad(s.steering.roadWheelDeg(:));
    idx=discretize(min(max(t,wt(1)),wt(end)),wt);idx=idx(:);
    fraction=(t(:)-wt(idx))./(wt(idx+1)-wt(idx));fraction=min(max(fraction,0),1);
    delta=wd(idx)+(.5-.5*cos(pi*fraction)).*(wd(idx+1)-wd(idx));delta=reshape(delta,size(t));
end

function signals=synthesizeSignals(truth,s,noise)
    stream=RandStream('mt19937ar','Seed',s.seed);clean=noise=="clean";
    step=round(s.motionStep/s.plantStep);ix=(1:step:numel(truth.time)).';t=truth.time(ix);n=numel(t);
    q=s.noise;if clean,q=structfun(@(~)0,q,UniformOutput=false);end
    white=@(sigma,count) sigma*randn(stream,count,1);
    h=struct('time',t, ...
        'steeringAngle',truth.steeringAngle(ix)+white(q.steeringStdRad,n), ...
        'longitudinalSpeed',(1+q.wheelScaleError)*truth.longitudinalSpeed(ix)+white(q.wheelSpeedStdMps,n), ...
        'longitudinalAcceleration',truth.longitudinalSpecificForce(ix)+white(q.longitudinalAccelerationStdMps2,n), ...
        'lateralAcceleration',truth.lateralSpecificForce(ix)+q.lateralAccelerationBiasMps2+white(q.lateralAccelerationStdMps2,n), ...
        'yawRate',truth.yawRate(ix)+q.yawRateBiasRadps+white(q.yawRateStdRadps,n));
    % Receiver: 20 Hz on its own clock, at the calibrated body offset, with
    % white plus first-order Gauss-Markov error (time-correlated multipath/atmosphere).
    tg=(s.gnss.offsetS:s.gnss.periodS:s.duration).';ng=numel(tg);g=truthAt(truth,tg);
    phi=exp(-s.gnss.periodS/max(q.gnssMarkovTimeS,eps));markov=zeros(ng,2);
    markov(1,:)=q.gnssMarkovStdM*randn(stream,1,2);
    for k=2:ng,markov(k,:)=phi*markov(k-1,:)+sqrt(1-phi^2)*q.gnssMarkovStdM*randn(stream,1,2);end
    receiver=g.position+[cos(g.heading)*s.receiverBodyOffsetM(1)-sin(g.heading)*s.receiverBodyOffsetM(2), ...
        sin(g.heading)*s.receiverBodyOffsetM(1)+cos(g.heading)*s.receiverBodyOffsetM(2)];
    gnssError=markov+q.gnssWhiteStdM*randn(stream,ng,2);
    gnssSigma=sqrt(s.noise.gnssWhiteStdM^2+s.noise.gnssMarkovStdM^2);
    signals.gnss=struct('time',tg,'position',receiver+gnssError,'valid',true(ng,1), ...
        'information',repmat(eye(2)/gnssSigma^2,1,1,ng),'delay',0,'truthError',gnssError);
    % LiDAR: jittered 10 Hz frames at the output point, anisotropic body-frame
    % covariance (weak along track), random rejected matches.
    tl=(s.lidar.offsetS:s.lidar.periodS:s.duration-s.lidar.periodS).';nl=numel(tl);
    tl=tl+q.lidarJitterS*(2*rand(stream,nl,1)-1);l=truthAt(truth,tl);
    pose=zeros(nl,3);information=zeros(3,3,nl);lidarError=zeros(nl,3);
    for k=1:nl
        R=[cos(l.heading(k)),-sin(l.heading(k));sin(l.heading(k)),cos(l.heading(k))];
        C=blkdiag(R*diag([s.noise.lidarAlongStdM,s.noise.lidarCrossStdM].^2)*R.',s.noise.lidarYawStdRad^2);
        information(:,:,k)=inv(C);information(:,:,k)=(information(:,:,k)+information(:,:,k).')/2;
        e=[R*([q.lidarAlongStdM;q.lidarCrossStdM].*randn(stream,2,1));q.lidarYawStdRad*randn(stream)];
        lidarError(k,:)=e.';pose(k,:)=[l.position(k,:),l.heading(k)]+e.';
    end
    valid=rand(stream,nl,1)>=q.lidarRejectFraction;pose(~valid,:)=NaN;
    signals.highRate=h;
    signals.lidar=struct('time',tl,'pose',pose,'valid',valid,'information',information,'delay',0, ...
        'headingConvention',"unwrapped",'truthError',lidarError);
end

function f=truthAt(truth,t)
    names=["longitudinalSpeed","lateralVelocity","yawRate","sideSlipAngleRate"];
    f=struct('time',t,'position',interp1(truth.time,truth.position,t,'spline'), ...
        'heading',interp1(truth.time,truth.heading,t,'spline'), ...
        'outputAcceleration',interp1(truth.time,truth.outputAcceleration,t,'spline'));
    for name=names,f.(name)=interp1(truth.time,truth.(name),t,'spline');end
    c=cos(f.heading);s=sin(f.heading);
    f.velocity=[c.*f.longitudinalSpeed-s.*f.lateralVelocity,s.*f.longitudinalSpeed+c.*f.lateralVelocity];
end

function x=initialState(truth,h,lateral,s,mode)
    p=truth.position(1,:).';yaw=truth.heading(1);
    if mode=="perturbed",p=p+s.initialError.positionM;yaw=yaw+s.initialError.headingRad;end
    % Production initialization: pose plus measured wheel/lateral velocity and
    % IMU acceleration rotated by the initial heading estimate.
    R=[cos(yaw),-sin(yaw);sin(yaw),cos(yaw)];
    v=R*[h.longitudinalSpeed(1);lateral.lateralVelocity(1)];a=R*[h.longitudinalAcceleration(1);h.lateralAcceleration(1)];
    x=[p(1);v(1);a(1);p(2);v(2);a(2);yaw];
end

function log=matcherState(n)
    log=containers.Map('KeyType','char','ValueType','any');
    log('seedError')=nan(n,2);log('accepted')=false(n,1);
end

function r=syntheticMatcher(k,seed,~,lidar,truth,settings,log)
    % Stand-in for localizeLidarFrame: converges to the recorded synthetic
    % measurement only when the fused seed lies inside its capture basin.
    positionError=norm(seed(1:2)-truth.position(k,:));
    yawError=abs(atan2(sin(seed(3)-truth.heading(k)),cos(seed(3)-truth.heading(k))));
    captured=positionError<=settings.captureRadiusM && yawError<=settings.captureYawRad;
    accepted=lidar.valid(k) && captured;
    pose=seed;information=lidar.information(:,:,k);
    if accepted,pose=lidar.pose(k,:);end
    e=log('seedError');e(k,:)=[positionError,yawError];log('seedError')=e;
    a=log('accepted');a(k)=accepted;log('accepted')=a; %#ok<NASGU> containers.Map is a handle
    r=struct('poseXYTheta',pose,'information',information,'accepted',accepted);
end

function m=scoreRun(estimate,truth,fine,lateral,data,s,mask)
    t=estimate.time;e=vecnorm(estimate.position-truth.position,2,2);
    headingError=atan2(sin(estimate.headingUnwrapped-truth.heading),cos(estimate.headingUnwrapped-truth.heading));
    speedError=vecnorm(estimate.velocity,2,2)-vecnorm(truth.velocity,2,2);
    settled=t>=s.settleS;headingSettled=t>=s.headingSettleS;
    m=struct('positionRmseM',rms(e),'settledPositionRmseM',rms(e(settled)), ...
        'settledPositionP95M',prctile(e(settled),95),'positionMaximumAfterSettleM',max(e(settled)), ...
        'finalPositionErrorM',e(end),'settledHeadingRmseDeg',rad2deg(rms(headingError(headingSettled))), ...
        'headingMaximumAfterSettleDeg',rad2deg(max(abs(headingError(headingSettled)))), ...
        'settledSpeedRmseMps',rms(speedError(settled)), ...
        'settledVelocityRmseMps',rms(vecnorm(estimate.velocity(settled,:)-truth.velocity(settled,:),2,2)), ...
        'settledAccelerationRmseMps2',NaN,'positionError',e,'headingError',headingError);
    % Lateral observer on its native 100 Hz grid, at the output point.
    lateralTruth=interp1(fine.time,fine.lateralVelocity,lateral.time);lateralSettled=lateral.time>=s.settleS;
    m.settledLateralVelocityRmseMps=rms(lateral.lateralVelocity(lateralSettled)-lateralTruth(lateralSettled));
    m.lateralVelocityError=lateral.lateralVelocity-lateralTruth;
    betaRateTruth=interp1(fine.time,fine.sideSlipAngleRate,lateral.time);
    m.settledSideSlipRateRmseRadps=rms(lateral.sideSlipAngleRate(lateralSettled)-betaRateTruth(lateralSettled));
    last=find(e>s.convergenceToleranceM,1,'last');m.convergenceTimeS=0;
    if ~isempty(last),m.convergenceTimeS=t(min(last+1,numel(t)))-t(1);if last==numel(t),m.convergenceTimeS=Inf;end,end
    m.outageMaximumM=NaN;m.recoveryTimeS=NaN;
    if isstruct(mask)
        windows=cell2mat(struct2cell(mask));window=[min(windows(:,1)),max(windows(:,2))];
        inside=t>=window(1) & t<window(2);m.outageMaximumM=max(e(inside));
        after=find(t>=window(2));bad=find(e(after)>s.outageRecoveryToleranceM,1,'last');
        m.recoveryTimeS=0;if ~isempty(bad),m.recoveryTimeS=t(after(min(bad+1,numel(after))))-window(2);end
        if ~isempty(bad) && bad==numel(after),m.recoveryTimeS=Inf;end
    end
    m.rawGnssRmseM=NaN;m.rawLidarRmseM=NaN;
    if isfield(data,'gnss')
        ok=data.gnss.valid & settled;
        m.rawGnssRmseM=rms(vecnorm(estimate.diagnostics.gnssPositionAtObserverPoint(ok,:)-truth.position(ok,:),2,2));
    end
    if isfield(data,'lidar')
        ok=data.lidar.valid & settled;m.rawLidarRmseM=rms(vecnorm(data.lidar.pose(ok,1:2)-truth.position(ok,:),2,2));
    end
    m.modeCounts=arrayfun(@(v) nnz(estimate.diagnostics.mode==v),0:3);
    m.headingModeCountsGyroLidar=[nnz(estimate.diagnostics.headingMode==0),nnz(estimate.diagnostics.headingMode==2)];
    m.maximumTrackAngleRate=estimate.diagnostics.maximumTrackAngleRate;
    m.acceleration=vecnorm(estimate.acceleration-rotateBody(truth.outputAcceleration,truth.heading),2,2);
    m.settledAccelerationRmseMps2=rms(m.acceleration(settled));
end

function m=addMatcherMetrics(m,log,t,s)
    a=log('accepted');e=log('seedError');settled=t>=s.settleS;
    m.matcherAcceptanceAfterSettle=mean(a(settled));m.matcherFirstAcceptS=t(find(a,1));
    m.settledSeedErrorRmseM=rms(e(settled,1));m.matcherSeedError=e;m.matcherAccepted=a;
end

function w=rotateBody(b,yaw)
    w=[cos(yaw).*b(:,1)-sin(yaw).*b(:,2),sin(yaw).*b(:,1)+cos(yaw).*b(:,2)];
end

function checks=evaluateChecks(runs,s)
    names=cellfun(@(r) r.scenario,runs);get=@(n) runs{names==n}.metrics;rows=cell(0,5);
    add=@(rows,name,value,limit,passed,statement) [rows;{name,value,limit,passed,statement}];
    c=get("clean_both_truth_init");
    rows=add(rows,"clean_consistency_position",c.positionMaximumAfterSettleM,.05,c.positionMaximumAfterSettleM<=.05, ...
        "Exact signals, truth initialization: maximum position error <= 5 cm (frame/sign conventions)");
    rows=add(rows,"clean_consistency_heading",c.headingMaximumAfterSettleDeg,.1,c.headingMaximumAfterSettleDeg<=.1, ...
        "Exact signals: maximum heading error <= 0.1 deg");
    d=get("clean_dead_reckoning");distance=pathLength(runs{names=="clean_dead_reckoning"}.frameTruth.position);
    rows=add(rows,"clean_dead_reckoning_drift_fraction",d.finalPositionErrorM/distance,.005,d.finalPositionErrorM/distance<=.005, ...
        "No absolute source: final drift <= 0.5% of distance travelled");
    b=get("noisy_both");gOnly=get("noisy_gnss_only");lOnly=get("noisy_lidar_only");
    rows=add(rows,"noisy_both_convergence_time",b.convergenceTimeS,s.settleS,b.convergenceTimeS<=s.settleS, ...
        "From 1.8 m / 3 deg initial error: position error stays <= 10 cm after 5 s");
    rows=add(rows,"noisy_both_heading_rmse",b.settledHeadingRmseDeg,.5,b.settledHeadingRmseDeg<=.5, ...
        "Heading RMSE after 10 s <= 0.5 deg");
    best=min(gOnly.settledPositionRmseM,lOnly.settledPositionRmseM);
    rows=add(rows,"fusion_not_worse_than_best_single_source",b.settledPositionRmseM,best,b.settledPositionRmseM<=best, ...
        "Both-source settled RMSE <= best single-source settled RMSE");
    rows=add(rows,"gnss_only_filters_raw_receiver",gOnly.settledPositionRmseM,gOnly.rawGnssRmseM,gOnly.settledPositionRmseM<gOnly.rawGnssRmseM, ...
        "GNSS-only settled RMSE below raw corrected receiver RMSE");
    rows=add(rows,"lidar_only_filters_raw_lidar",lOnly.settledPositionRmseM,lOnly.rawLidarRmseM,lOnly.settledPositionRmseM<lOnly.rawLidarRmseM, ...
        "LiDAR-only settled RMSE below raw LiDAR pose RMSE");
    for n=["gnss_outage_40_60","lidar_outage_40_60"]
        o=get(n);rows=add(rows,n+"_maximum",o.outageMaximumM,.3,o.outageMaximumM<=.3,"Single-source outage: error <= 30 cm");
        rows=add(rows,n+"_recovery",o.recoveryTimeS,3,o.recoveryTimeS<=3,"Error <= 15 cm within 3 s after outage");
    end
    o=get("both_outage_40_50");
    rows=add(rows,"both_outage_40_50_maximum",o.outageMaximumM,1,o.outageMaximumM<=1,"10 s dead reckoning on supplied motion: error <= 1 m");
    rows=add(rows,"both_outage_40_50_recovery",o.recoveryTimeS,3,o.recoveryTimeS<=3,"Error <= 15 cm within 3 s after outage");
    a=get("alternating_1s");
    rows=add(rows,"alternating_sources",a.settledPositionRmseM,.1,a.settledPositionRmseM<=.1,"Sources alternating each second: settled RMSE <= 10 cm");
    m=get("online_matcher");
    rows=add(rows,"online_matcher_acceptance",m.matcherAcceptanceAfterSettle,.9,m.matcherAcceptanceAfterSettle>=.9, ...
        "Closed-loop matcher with 1 m capture basin: acceptance after 5 s >= 90%");
    rows=add(rows,"online_matcher_accuracy",m.settledPositionRmseM,1.1*b.settledPositionRmseM,m.settledPositionRmseM<=1.1*b.settledPositionRmseM, ...
        "Closed-loop matching RMSE within 10% of recorded-LiDAR fusion");
    rows=add(rows,"lateral_observer_nominal",b.settledLateralVelocityRmseMps,.05,b.settledLateralVelocityRmseMps<=.05, ...
        "Lateral velocity RMSE at output point after 5 s <= 0.05 m/s (nominal plant, noise and ay bias)");
    p=get("plant_mismatch_both");
    rows=add(rows,"plant_mismatch_position",p.settledPositionRmseM,.1,p.settledPositionRmseM<=.1, ...
        "Cornering stiffness -15%/+15%: fused settled RMSE <= 10 cm");
    checks=cell2table(rows,VariableNames={'check','value','limit','passed','criterion'});
end

function L=pathLength(p)
    L=sum(vecnorm(diff(p),2,2));
end

function plotRuns(runs,folder)
    names=cellfun(@(r) r.scenario,runs);get=@(n) runs{names==n};
    b=get("noisy_both");f=figure(Visible="off",Position=[0,0,1200,900]);theme(f,"light");
    tiledlayout(f,3,2,TileSpacing="compact");
    nexttile;plot(b.frameTruth.position(:,1),b.frameTruth.position(:,2),'k',LineWidth=1.5);hold on;
    plot(b.estimate.position(:,1),b.estimate.position(:,2),'--',LineWidth=1);axis equal;grid on;
    legend(["truth","fused (noisy both)"],Location="best");title("Trajectory");xlabel("X (m)");ylabel("Y (m)");
    nexttile;hold on;
    for n=["noisy_both","noisy_gnss_only","noisy_lidar_only","plant_mismatch_both"]
        r=get(n);plot(r.estimate.time,r.metrics.positionError,DisplayName=strrep(n,"_"," "));
    end
    set(gca,YScale="log");grid on;legend(Location="best");title("Position error");xlabel("t (s)");ylabel("m");
    nexttile;hold on;
    for n=["gnss_outage_40_60","lidar_outage_40_60","both_outage_40_50","alternating_1s"]
        r=get(n);plot(r.estimate.time,r.metrics.positionError,DisplayName=strrep(n,"_"," "));
    end
    xregion(40,60,FaceAlpha=.08,HandleVisibility="off");set(gca,YScale="log");grid on;legend(Location="best");
    title("Outage scenarios");xlabel("t (s)");ylabel("m");
    nexttile;hold on;
    for n=["noisy_both","noisy_gnss_only","noisy_lidar_only"]
        r=get(n);plot(r.estimate.time,rad2deg(r.metrics.headingError),DisplayName=strrep(n,"_"," "));
    end
    grid on;legend(Location="best");title("Heading error");xlabel("t (s)");ylabel("deg");ylim([-1,3.5]);
    nexttile;l=b.lateral;plot(l.time,l.lateralVelocity,DisplayName="estimate");hold on;
    plot(l.time,l.lateralVelocity-b.metrics.lateralVelocityError,'k',DisplayName="truth");
    p=get("plant_mismatch_both");plot(p.lateral.time,p.metrics.lateralVelocityError,DisplayName="mismatch error");
    grid on;legend(Location="best");title("Lateral velocity at output point");xlabel("t (s)");ylabel("m/s");
    nexttile;m=get("online_matcher");yyaxis left;plot(m.estimate.time,m.metrics.matcherSeedError(:,1));ylabel("seed error (m)");
    set(gca,YScale="log");yyaxis right;stairs(m.estimate.time,double(m.metrics.matcherAccepted));ylim([-.1,1.1]);ylabel("accepted");
    grid on;title("Online matcher: fused seed error and acceptance");xlabel("t (s)");
    exportgraphics(f,fullfile(folder,'cascade_overview.png'),Resolution=130);close(f);
end

function out=ifelse(condition,a,b)
    out=b;if condition,out=a;end
end

function writeJson(path,value)
    fid=fopen(path,'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(value,PrettyPrint=true));
end
