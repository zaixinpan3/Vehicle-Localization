function report=validateMncavInputAmplification(outputFolder)
% validateMncavInputAmplification Check local noise transfer independently.
% The scalar transfer is a frozen q=0, N=0, W=I model; it is not asserted
% to describe every sample of the recorded time-varying vehicle trajectory.
    arguments
        outputFolder (1,1) string="output/mncav_input_comparison_20260914"
    end
    setupVehicleLocalization;s=load(fullfile(outputFolder,'comparison.mat'));
    selected=[1,8];frequency=linspace(.001,4,60000);z=2i*pi*frequency;
    gains=zeros(numel(selected),numel(frequency));rows=cell(numel(selected),1);
    for k=1:numel(selected)
        cfg=s.configs{selected(k)};design=s.designs{selected(k)};
        D=buildImprovedObserverCertificateData(cfg);physical=D.T*design.K;
        k1=physical(1,1);k2=physical(2,1);k3=physical(3,1);
        numerator=k1*z.^2+k2*z+k3;response=numerator./(z.^3+numerator);
        gains(k,:)=abs(response);[peak,j]=max(gains(k,:));omega=2*pi*frequency(j);
        % Run the actual observer with an independent analytic source.
        time=(0:.01:45).';zero=zeros(size(time));cfg.observer.initialState=zeros(7,1);design.N=zeros(7,4);
        high=struct('time',time,'longitudinalSpeed',zero,'steeringAngle',zero, ...
            'longitudinalAcceleration',zero,'lateralAcceleration',zero,'yawRate',zero);
        lateral=struct('time',time,'lateralVelocity',zero,'sideSlipAngle',zero,'sideSlipAngleRate',zero);
        amplitude=.01;lidar=struct('delay',0,'headingConvention',"unwrapped", ...
            'evaluate',@(t) struct('pose',[amplitude*sin(omega*t);0;0],'information',1e6*eye(3)));
        result=runImprovedVehicleObserver(struct('highRate',high,'lidar',lidar),struct(),design,cfg,LateralInputs=lateral);
        mask=time>=30;
        coefficients=[sin(omega*time(mask)),cos(omega*time(mask)),ones(nnz(mask),1)]\result.position(mask,1);
        measured=hypot(coefficients(1),coefficients(2))/amplitude;
        rows{k}=struct('name',s.report.candidates(selected(k)).name,'analyticPeakGain',peak, ...
            'peakFrequencyHz',frequency(j),'measuredGain',measured,'relativeError',abs(measured/peak-1), ...
            'k1',k1,'k2',k2,'k3',k3,'durationSeconds',45,'fitStartSeconds',30,'noiseAmplitudeM',amplitude);
        assert(rows{k}.relativeError<1e-5,'Independent harmonic response does not match the local transfer.');
    end
    i=s.index;t=s.baseline.estimate.time(i);reference=s.baseline.pvaPose(i,1:2);
    inputError=vecnorm(s.baseline.data.lidar.pose(i,1:2)-reference,2,2);
    oldError=vecnorm(s.runs{1}(i,[1,4])-reference,2,2);
    candidateError=vecnorm(s.runs{8}(i,[1,4])-reference,2,2);
    % Confirm auxiliary-channel influence on this exact zero-delay replay.
    cfg=s.configs{1};design=s.designs{1};design.N=zeros(7,4);
    auxiliaryEstimate=runImprovedVehicleObserver(s.baseline.data,struct(),design,cfg, ...
        LateralInputs=s.baseline.lateralInput);
    auxiliaryControl=struct('pvaPositionRmseM',sqrt(mean(sum((auxiliaryEstimate.position(i,:)-reference).^2,2))), ...
        'pvaPositionMaximumM',max(vecnorm(auxiliaryEstimate.position(i,:)-reference,2,2)), ...
        'maximumPositionChangeM',max(vecnorm(auxiliaryEstimate.position-s.baseline.estimate.position,2,2)), ...
        'baselinePvaPositionRmseM',rms(oldError));
    save(fullfile(outputFolder,'zero_auxiliary_control.mat'),'auxiliaryEstimate','auxiliaryControl','-v7.3');
    writetable(table(t,inputError,oldError,candidateError),fullfile(outputFolder,'continuous_error_curves.csv'));
    harmonic=struct2table(vertcat(rows{:}));writetable(harmonic,fullfile(outputFolder,'harmonic_checks.csv'));
    fig=figure('Color','w','Name','Actual LiDAR input versus observer','Position',[100,80,1250,800]);
    tiledlayout(fig,2,2,'TileSpacing','compact');
    nexttile;plot(t,[inputError,oldError,candidateError]);grid on;
    xlabel('Receiver time (s)');ylabel('Position discrepancy from INSPVA (m)');title('Same 100 Hz timestamps');
    legend('Continuous LiDAR input','Current observer','Position gain doubled',Location='northwest',FontSize=9);
    nexttile;plot(t,[inputError,oldError,candidateError]);xlim([79,84]);grid on;
    xlabel('Receiver time (s)');ylabel('Position discrepancy (m)');title('Residual peak amplification');
    nexttile;plot(frequency,gains);yline(1,'k--');xlim([0,2]);grid on;
    xlabel('Frequency (Hz)');ylabel('Position noise amplitude gain');title('Local zero-delay transfer; q=0, N=0, W=I');
    legend('Current gains','Position gain doubled',Location='northeast',FontSize=9);
    nexttile;bar([rms(inputError),rms(oldError),rms(candidateError)]);grid on;
    xticks(1:3);xticklabels({'LiDAR input','Current observer','Position gain doubled'});
    ylabel('Whole-sequence position RMSE (m)');title('Improved tuning still does not beat the input');
    drawnow;exportgraphics(fig,fullfile(outputFolder,'input_comparison.png'),'Resolution',170);
    exportgraphics(fig,fullfile(outputFolder,'input_comparison.pdf'),'ContentType','vector');
    report=struct('harmonic',table2struct(harmonic),'zeroAuxiliaryControl',auxiliaryControl,'scope', ...
        "Independent harmonic check of the q=0, N=0 zero-delay scalar response; all recorded errors retain original samples.");
    fid=fopen(fullfile(outputFolder,'harmonic_validation.json'),'w');assert(fid>=0);cleanup=onCleanup(@() fclose(fid));
    fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));disp(harmonic);disp(auxiliaryControl);
end
