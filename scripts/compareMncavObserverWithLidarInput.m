function report=compareMncavObserverWithLidarInput(outputFolder)
% compareMncavObserverWithLidarInput Audit the actual input and fixed-gain cases.
% All candidates use the same zero-delay cached input, original time grid,
% actual lateral output, reference and initial state except the named initial
% measurement control. This is an exploratory single-sequence comparison.
    arguments
        outputFolder (1,1) string="output/mncav_input_comparison_20260914"
    end
    setupVehicleLocalization;if ~isfolder(outputFolder),mkdir(outputFolder);end
    baselineFolder="output/mncav_zero_delay_20260914";
    baseline=load(fullfile(baselineFolder,'experiment.mat'));
    calls=readtable(fullfile(baselineFolder,'precomputed_calls.csv'));
    index=baseline.uniformIndices;time=baseline.estimate.time(index);
    inputPose=baseline.data.lidar.pose(index,:);reference=baseline.referencePose(index,:);
    alternate=baseline.pvaPose(index,:);
    inputMetrics=score(inputPose,reference,alternate,time);
    definitions=table(["baseline";"first_measurement_initialization";"theta_25";"theta_3"; ...
        "theta_4";"theta_6";"theta_8";"position_gain_6";"position_gain_9"], ...
        [2;2;2.5;3;4;6;8;2;2],[3;3;3;3;3;3;3;6;9], ...
        [false;true;false;false;false;false;false;false;false], ...
        VariableNames={'name','theta','positionGain','measurementInitialization'});
    runs=cell(height(definitions),1);designs=runs;configs=runs;rows=runs;
    for trial=1:height(definitions)
        c=definitions(trial,:);cfg=baseline.cfg;cfg.observer.theta=c.theta;
        design=baseline.design;design.theta=c.theta;
        design.K(1,1)=c.positionGain;design.K(4,2)=c.positionGain;
        errorMessage="";status="completed";witnessDelay=0;duration=NaN;
        try
            verification=verifyImprovedObserverDesign(design,cfg);
            if ~verification.certified
                % Positive-delay synthesis is only a numerical certificate
                % construction. The actual measurement and runtime delay stay 0.
                witnessDelay=min(.15,.9/(c.theta*c.positionGain));
                witnessCfg=cfg;witnessCfg.measurement.fixedLidarDelay=witnessDelay;
                design=synthesizeFixedGains(design,witnessCfg);
                verification=verifyImprovedObserverDesign(design,cfg);
            end
            assert(verification.certified,'VehicleLocalization:InfeasibleCertificate','Zero-delay matrix check failed.');
            design.verification=verification;design.certified=true;
            if c.measurementInitialization
                p=baseline.data.lidar.pose(1,:);cfg.observer.initialState([1,4,7])=p;
                rotation=[cos(p(3)),-sin(p(3));sin(p(3)),cos(p(3))];
                cfg.observer.initialState([2,5])=rotation*[baseline.data.highRate.longitudinalSpeed(1); ...
                    baseline.lateralInput.lateralVelocity(1)];
            end
            timer=tic;
            estimate=runImprovedVehicleObserver(baseline.data,struct(),design,cfg,LateralInputs=baseline.lateralInput);
            duration=toc(timer);metrics=score(estimate.pose(index,:),reference,alternate,time);
            metrics.maximumVelocityComponent=max(abs(estimate.z(:,[2,5])),[],'all');
            metrics.maximumAccelerationComponent=max(abs(estimate.z(:,[3,6])),[],'all');
            metrics.maximumDifferenceFromStoredState=max(abs(estimate.z-baseline.estimate.z),[],'all');
            runs{trial}=estimate.z;designs{trial}=design;configs{trial}=cfg;
            fprintf('%s: PVA RMSE %.6f, peak %.6f, input RMSE %.6f, %.1f s\n', ...
                c.name,metrics.pvaPositionRmseM,metrics.pvaPositionMaximumM,inputMetrics.pvaPositionRmseM,duration);
        catch exception
            status="failed";errorMessage=string(exception.identifier)+": "+string(exception.message);
            metrics=emptyMetrics(inputMetrics);verification=struct('uniformMargin',NaN);
            fprintf('%s: %s\n',c.name,errorMessage);
        end
        row=struct('name',c.name,'status',status,'theta',c.theta,'normalizedPositionGain',c.positionGain, ...
            'certificateConstructionDelay',witnessDelay,'runtimeDelay',0,'certificateMargin',verification.uniformMargin, ...
            'seconds',duration,'errorMessage',errorMessage);
        for name=string(fieldnames(metrics)).',row.(name)=metrics.(name);end
        rows{trial}=row;
        writetable(struct2table(vertcat(rows{1:trial})),fullfile(outputFolder,'candidate_metrics.csv'));
    end
    metrics=struct2table(vertcat(rows{:}));
    inputError=inputPose(:,1:2)-alternate(:,1:2);
    observerError=baseline.estimate.pose(index,1:2)-alternate(:,1:2);
    residual=observerError-inputError;
    decomposition=struct('inputMse',mean(sum(inputError.^2,2)),'addedStateDifferenceMse',mean(sum(residual.^2,2)), ...
        'crossTerm',2*mean(sum(inputError.*residual,2)),'observerMse',mean(sum(observerError.^2,2)));
    [~,peak]=max(vecnorm(observerError,2,2));
    audit=struct('input',inputMetrics,'baseline',table2struct(metrics(1,:)), ...
        'fractionBaselineWorseThanInput',mean(vecnorm(observerError,2,2)>vecnorm(inputError,2,2)), ...
        'peakTimeSeconds',time(peak),'inputErrorAtObserverPeakM',norm(inputError(peak,:)), ...
        'observerErrorAtSamePeakM',norm(observerError(peak,:)), ...
        'decomposition',decomposition,'uniformSamples',numel(index), ...
        'comparison',"Identical 100 Hz times and references: actual continuous LiDAR input vs observer output. No mixed sampling baselines.");
    report=struct('audit',audit,'candidates',table2struct(metrics),'definitions',table2struct(definitions), ...
        'scope',"Exploratory fixed-input gain sensitivity on the same drive; not independent validation or population inference.");
    fid=fopen(fullfile(outputFolder,'summary.json'),'w');assert(fid>=0);cleanup=onCleanup(@() fclose(fid));
    fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));
    save(fullfile(outputFolder,'comparison.mat'),'report','runs','designs','configs','baseline','calls','index','-v7.3');
    disp(metrics);disp(audit);
end

function value=score(pose,reference,alternate,time)
    a=vecnorm(pose(:,1:2)-alternate(:,1:2),2,2);r=vecnorm(pose(:,1:2)-reference(:,1:2),2,2);
    yaw=atan2(sin(pose(:,3)-reference(:,3)),cos(pose(:,3)-reference(:,3)));
    value=struct('odomPositionRmseM',rms(r),'pvaPositionRmseM',rms(a), ...
        'pvaPositionMaximumM',max(a),'pvaPositionP95M',prctile(a,95), ...
        'pvaPositionAfter5RmseM',rms(a(time>=5)),'headingRmseDeg',rad2deg(rms(yaw)), ...
        'maximumVelocityComponent',NaN,'maximumAccelerationComponent',NaN,'maximumDifferenceFromStoredState',NaN);
end

function value=emptyMetrics(example)
    value=example;for name=string(fieldnames(value)).',value.(name)=NaN;end
end

function design=synthesizeFixedGains(design,cfg)
    data=buildImprovedObserverCertificateData(cfg);
    [vertices,uncertainty]=continuousLidarCertificateVertices(design,cfg);
    Ad=-cfg.observer.theta*design.K*data.C;delay=cfg.measurement.fixedLidarDelay;
    yalmip('clear');P=sdpvar(7,7,'symmetric');Q=sdpvar(7,7,'symmetric');R=sdpvar(7,7,'symmetric');
    g=sdpvar(1);margin=sdpvar(1);pBound=sdpvar(1);rBound=sdpvar(1);
    constraints=[P>=1e-5*eye(7),Q>=1e-5*eye(7),R>=1e-5*eye(7), ...
        P<=pBound*eye(7),R<=rBound*eye(7),trace(P)==7,trace(Q)+trace(R)<=1000, ...
        g>=1e-5,g<=1000,margin>=1e-6];
    for k=1:size(vertices,3)
        block=continuousObserverDelayLmi(vertices(:,:,k),Ad,P,Q,R,g,delay,design.rate);
        constraints=[constraints,block<=-(margin+2*uncertainty*(pBound+delay*rBound))*eye(28)]; %#ok<AGROW>
    end
    result=optimize(constraints,-margin,sdpsettings('solver',char(cfg.synthesis.solver),'verbose',0));
    assert(result.problem==0,'VehicleLocalization:InfeasibleCertificate','Fixed-gain synthesis failed: %s',result.info);
    design.P=double(P);design.Q=double(Q);design.R=double(R);design.g=double(g);
    for name=["P","Q","R"],design.(name)=(design.(name)+design.(name).')/2;end
    verification=verifyImprovedObserverDesign(design,cfg);assert(verification.certified);
end
