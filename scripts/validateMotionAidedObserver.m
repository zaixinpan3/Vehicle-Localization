function report=validateMotionAidedObserver(outputFolder,options)
% validateMotionAidedObserver Frozen-gain synthetic cascade and noise checks.
% Thirty runs over ten noise seeds use 0.7/1/1.3 times nominal axle
% stiffness in the true MnCAV plant, while estimation keeps nominal parameters.
% Noise values are declared simulation assumptions, not sensor specifications.
% A separate clean-LiDAR/biased-motion counterexample tests the claim boundary.
    arguments
        outputFolder (1,1) string="output/mncav_motion_aided_20260914/synthetic"
        options.ExperimentFile (1,1) string="output/mncav_motion_aided_20260914/experiment.mat"
    end
    setupVehicleLocalization;if ~isfolder(outputFolder),mkdir(outputFolder);end
    originalRng=rng;cleanup=onCleanup(@() rng(originalRng));
    saved=load(options.ExperimentFile,'lateralDesign','lateralCfg','cfg');cfg=saved.cfg;
    records=zeros(30,12);runs=cell(30,1);count=0;
    for factor=[.7,1,1.3]
        truth=plantTruth(saved.lateralCfg.vehicle,factor);
        for seed=20260915:20260924
            count=count+1;rng(seed,'twister');t=truth.time;n=numel(t);
            high=truth.highRate;
            high.longitudinalSpeed=high.longitudinalSpeed+.05*randn(n,1);
            high.longitudinalAcceleration=high.longitudinalAcceleration+.08*randn(n,1);
            high.lateralAcceleration=high.lateralAcceleration+.08*randn(n,1);
            high.yawRate=high.yawRate+.002*randn(n,1);
            high.steeringAngle=high.steeringAngle+deg2rad(.05)*randn(n,1);
            lateral=runLateralVelocityObserver(high,saved.lateralDesign,saved.lateralCfg);
            knots=1:10:n;lidar=truth.pose(knots,:)+randn(numel(knots),3).*[.1,.1,deg2rad(.3)];
            lidar(:,1)=lidar(:,1)+.15*sin(.7*t(knots));lidar(:,2)=lidar(:,2)+.1*sin(.4*t(knots));
            pose=interp1(t(knots),lidar,t,'linear');data=struct('highRate',high,'lidar',source(t,pose));
            estimate=runMotionAidedVehicleObserver(data,lateral,cfg);
            raw=metrics(pose,truth.pose);new=metrics(estimate.pose,truth.pose);
            records(count,:)=[factor,seed,raw,new,max(abs(estimate.trackAngleRate)),all(new<=raw)];
            runs{count}=struct('truth',truth,'data',data,'lateral',lateral,'estimate',estimate);
        end
        fprintf('Completed synthetic plant stiffness factor %.1f.\n',factor);
    end
    scores=array2table(records,VariableNames={'stiffnessFactor','seed','rawRmseM','rawMaximumM', ...
        'rawP95M','rawHeadingRmseDeg','observerRmseM','observerMaximumM','observerP95M', ...
        'observerHeadingRmseDeg','maximumCourseRate','allMetricsNoWorse'});
    % Constant-noise-frequency check: zero motion and positive LiDAR ripple.
    t=(0:.01:40).';zero=zeros(size(t));high=struct('time',t,'longitudinalSpeed',zero, ...
        'longitudinalAcceleration',zero,'lateralAcceleration',zero,'yawRate',zero);
    lateral=struct('time',t,'lateralVelocity',zero,'sideSlipAngleRate',zero);
    frequency=.3587165;pose=[.01*sin(2*pi*frequency*t),zero,zero];
    harmonic=runMotionAidedVehicleObserver(struct('highRate',high,'lidar',source(t,pose)),lateral,cfg);
    mask=t>=25;basis=[sin(2*pi*frequency*t(mask)),cos(2*pi*frequency*t(mask)),ones(nnz(mask),1)];
    coefficient=basis\harmonic.position(mask,1);amplitude=norm(coefficient(1:2))/.01;
    expected=cfg.gains(1)/hypot(cfg.gains(1),2*pi*frequency);
    % The linear source itself attenuates the ideal sinusoid by O(dt^2).
    harmonicRelativeError=abs(amplitude-expected)/expected;assert(harmonicRelativeError<5e-5);
    high.longitudinalSpeed(:)=1;pose(:)=0;
    counterexample=runMotionAidedVehicleObserver(struct('highRate',high,'lidar',source(t,pose)),lateral,cfg);
    counterexampleRmse=rms(vecnorm(counterexample.position,2,2));
    report=struct('seeds',20260915:20260924,'plantStiffnessFactors',[.7,1,1.3], ...
        'runs',30,'gains',cfg.gains,'positionRmseWins',nnz(scores.observerRmseM<=scores.rawRmseM), ...
        'allMetricWins',nnz(scores.allMetricsNoWorse),'meanRawRmseM',mean(scores.rawRmseM), ...
        'meanObserverRmseM',mean(scores.observerRmseM), ...
        'allCourseRatesInEnvelope',all(scores.maximumCourseRate<=cfg.maximumTrackAngleRate), ...
        'harmonicFrequencyHz',frequency,'harmonicMeasuredGain',amplitude,'harmonicExpectedGain',expected, ...
        'harmonicRelativeError',harmonicRelativeError,'cleanLidarBiasedMotionRmseM',counterexampleRmse, ...
        'cleanLidarBaselineRmseM',0,'universalDominanceClaimed',false, ...
        'noiseAssumptions',"100 Hz independent Gaussian speed .05 m/s, acceleration .08 m/s^2, gyro .002 rad/s, steering .05 deg; 10 Hz LiDAR XY .1 m, yaw .3 deg, plus X .15*sin(.7*t) m and Y .1*sin(.4*t) m.");
    writetable(scores,fullfile(outputFolder,'metrics.csv'));
    save(fullfile(outputFolder,'validation.mat'),'report','scores','runs','harmonic','counterexample','-v7.3');
    fid=fopen(fullfile(outputFolder,'summary.json'),'w');assert(fid>=0);
    fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));fclose(fid);disp(report);
end

function truth=plantTruth(vehicle,factor)
    vehicle.frontCorneringStiffness=vehicle.frontCorneringStiffness*factor;
    vehicle.rearCorneringStiffness=vehicle.rearCorneringStiffness*factor;
    model=lateralBicycleModel(vehicle);t=(0:.01:40).';n=numel(t);
    vx=8+2*(1-cos(.2*t));vxDot=.4*sin(.2*t);
    steering=deg2rad(2)*(sin(.25*t)+.3*sin(.65*t));state=zeros(n,2);
    for k=2:n
        dt=t(k)-t(k-1);v=(vx(k)+vx(k-1))/2;delta=(steering(k)+steering(k-1))/2;
        A=model.A0+v*model.A1+model.A2/v;
        % Exact frozen-midpoint linear plant step; independent of observer RK4.
        transition=expm([A,model.B;zeros(1,3)]*dt);
        state(k,:)=(transition(1:2,1:2)*state(k-1,:).'+transition(1:2,3)*delta).';
    end
    vy=state(:,1);r=state(:,2);ay=zeros(n,1);
    for k=1:n
        C=model.C0+model.C2/vx(k);ay(k)=C(1,:)*state(k,:).'+model.D(1)*steering(k);
    end
    psi=cumtrapz(t,r);velocity=[cos(psi).*vx-sin(psi).*vy,sin(psi).*vx+cos(psi).*vy];
    pose=[cumtrapz(t,velocity),psi];
    high=struct('time',t,'longitudinalSpeed',vx,'steeringAngle',steering, ...
        'longitudinalAcceleration',vxDot-vy.*r,'lateralAcceleration',ay,'yawRate',r);
    truth=struct('time',t,'pose',pose,'velocity',velocity,'lateralVelocity',vy,'highRate',high, ...
        'vehicle',vehicle,'stiffnessFactor',factor);
end

function s=source(t,pose)
    s=struct('time',t,'pose',pose,'information',repmat(1e6*eye(3),1,1,numel(t)), ...
        'delay',0,'representation',"piecewiseLinear",'headingConvention',"unwrapped");
end

function m=metrics(pose,reference)
    e=vecnorm(pose(:,1:2)-reference(:,1:2),2,2);y=atan2(sin(pose(:,3)-reference(:,3)),cos(pose(:,3)-reference(:,3)));
    m=[rms(e),max(e),prctile(e,95),rad2deg(rms(y))];
end
