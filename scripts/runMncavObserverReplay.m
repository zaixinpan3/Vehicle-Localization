function report = runMncavObserverReplay(replayFolder,designFile,parameterFile,outputFolder,mode,fixedLidarDelay,options)
% runMncavObserverReplay Evaluate a declared continuous reconstruction of data.
% Only separate gnss/lidar modes are supported. This offline adapter rejects
% missing measurements and inadequate LiDAR information; it does not simulate
% measurement arrival, fusion, outages or correction pulses.
    arguments
        replayFolder (1,1) string
        designFile (1,1) string
        parameterFile (1,1) string
        outputFolder (1,1) string
        mode (1,1) string {mustBeMember(mode,["gnss","lidar"])} = "lidar"
        fixedLidarDelay (1,1) double {mustBeNonnegative} = .15
        options.SensorFolder (1,1) string = ""
        options.MaximumReconstructionGap (1,1) double {mustBePositive} = .12
    end
    if ~isfolder(outputFolder),mkdir(outputFolder);end
    folder=options.SensorFolder;
    if strlength(folder)==0,folder=fullfile(fileparts(replayFolder),'sensors');end
    calls=table();if mode=="lidar",calls=readtable(fullfile(replayFolder,'calls.csv'));end
    [raw,reference,metadata]=prepareMncavObserverReplay(folder,parameterFile,calls,fixedLidarDelay);
    loaded=load(designFile,'observerDesign','observerCfg','lateralDesign');cfg=loaded.observerCfg;
    assert(isfield(cfg,'mode') && cfg.mode==mode,'VehicleLocalization:CertificateMismatch', ...
        'Generate a design for the selected continuous mode first.');
    cfg.measurement.fixedLidarDelay=fixedLidarDelay;
    started=tic;
    try
        if mode=="gnss",recorded=raw.gps;information=[];
        else,recorded=raw.lidar;information=recorded.information;end
        [data,reconstruction]=reconstructContinuousObserverSignals(raw.highRate,recorded.timestamp, ...
            recorded.pose,information,cfg,MaximumGap=options.MaximumReconstructionGap);
        t=data.highRate.time;
        referencePose=interp1(reference.time,[reference.x,reference.y,reference.psi],t,'linear');
        yaw=referencePose(1,3)+deg2rad(2);v=data.highRate.longitudinalSpeed(1);
        initial=[referencePose(1,1)+.5;v*cos(yaw);0;referencePose(1,2)-.4;v*sin(yaw);0;yaw];
        cfg.observer.initialState=initial;
        estimate=runImprovedVehicleObserver(data,loaded.lateralDesign,loaded.observerDesign,cfg,InitialHistory=@(~) initial);
        error=estimate.pose-referencePose;error(:,3)=atan2(sin(error(:,3)),cos(error(:,3)));
        summary=struct('mode',mode,'completed',true,'samples',numel(t), ...
            'positionRmseM',sqrt(mean(sum(error(:,1:2).^2,2))), ...
            'yawRmseDeg',rad2deg(sqrt(mean(error(:,3).^2))), ...
            'computationSeconds',toc(started),'matrixCertificateVerified',estimate.observer.certificateVerified, ...
            'coefficientBoundsSatisfied',estimate.diagnostics.certificateConditions.coefficientBoundsSatisfied, ...
            'unconditionalStabilityClaimed',false);
        metadata.reconstruction=reconstruction;
        metadata.initialization="Reference-based biased current pose and constant estimated prehistory; initial error bound unverified.";
        metadata.observer="Continuous ODE/DDE; no physical sensor continuity or online causality established by reconstruction.";
        report=struct('summary',summary,'metadata',metadata,'estimate',estimate);
        writetable(array2table([t,estimate.pose,referencePose], ...
            'VariableNames',{'time','x','y','psi','referenceX','referenceY','referencePsi'}),fullfile(outputFolder,'trajectory.csv'));
        save(fullfile(outputFolder,'report.mat'),'report','cfg','-v7.3');
    catch exception
        summary=struct('mode',mode,'completed',false,'errorIdentifier',string(exception.identifier), ...
            'message',string(exception.message),'computationSeconds',toc(started));
        report=struct('summary',summary,'metadata',metadata);
    end
    file=fopen(fullfile(outputFolder,'summary.json'),'w');assert(file>=0);
    cleanup=onCleanup(@() fclose(file));fprintf(file,'%s\n',jsonencode(summary,PrettyPrint=true));
end
