function summaries = validatePoseObserverImplementation(sequenceFolder,outputFolder,options)
% validatePoseObserverImplementation Reproduce full recorded observer checks.
% Reuse the frozen Mississippi map and sourced/nominal vehicle inputs from a
% prepared sequence. Default: rerun coarse perception and D2D on all 1170 raw
% scans. RerunPerception=false explicitly reuses existing calls in OUTPUTFOLDER.
% Every observer case advances once and scores causal output without state replay.
    arguments
        sequenceFolder (1,1) string
        outputFolder (1,1) string
        options.RerunPerception (1,1) logical = true
    end
    setupVehicleLocalization;
    threads=maxNumCompThreads;
    cleanup=onCleanup(@() maxNumCompThreads(threads));
    maxNumCompThreads(8);
    if ~isfolder(outputFolder), mkdir(outputFolder); end
    sensors=fullfile(sequenceFolder,'sensors');
    parameters=fullfile(sequenceFolder,'vehicle_parameters.json');
    replayFolder=fullfile(outputFolder,'recursive_8threads');
    if options.RerunPerception
        replayMississippiLocalization(fullfile(sequenceFolder,'publishedCloud.mat'),sensors,replayFolder);
    end
    calls=readtable(fullfile(replayFolder,'calls.csv'));
    assert(height(calls)==1170,'The full fresh/reused recorded replay is required.');
    source=load(fullfile(sequenceFolder,'mncavObserverDesign.mat'),'lateralDesign');
    lateralDesign=source.lateralDesign;
    observerCfg=improvedObserverConfig;
    observerCfg.measurement.inputInterpolation="zoh";
    observerDesign=improvedObserverReferenceDesign(observerCfg);
    designFile=fullfile(outputFolder,'final_design.mat');
    save(designFile,'observerDesign','observerCfg','lateralDesign');
    scenarios=["fusion","positionOutage","lidarOnly","gpsOnly","fusion","outageNoLidar"];
    names=["fusion","positionOutage","lidarOnly","gpsOnly","fusion_delay300ms","outageNoLidar"];
    delays=[.15,.15,.15,.15,.30,.15];
    summaryCells=cell(1,numel(scenarios));
    for k=1:numel(scenarios)
        report=runMncavObserverReplay(replayFolder,designFile,parameters, ...
            fullfile(outputFolder,"final_"+names(k)),scenarios(k),delays(k),SensorFolder=sensors);
        summaryCells{k}=report.summary;
        assert(report.summary.completed,'A recorded observer case did not finish.');
    end
    summaries=[summaryCells{:}];
    writetable(struct2table(summaries),fullfile(outputFolder,'final_scenarios.csv'));
    metadata=struct('sequence',sequenceFolder,'reusedExistingPerceptionAndD2D',~options.RerunPerception, ...
        'scans',height(calls),'sourceCalls',fullfile(replayFolder,'calls.csv'), ...
        'fixedDelaySeconds',delays,'scenarioNames',names, ...
        'poseOutput',"causal X,Y,psi",'sameDriveMap',true, ...
        'knownRecordedTiltRetained',true,'nominalVehicleDynamics',true);
    fid=fopen(fullfile(outputFolder,'validation_metadata.json'),'w');assert(fid>=0);
    fileCleanup=onCleanup(@() fclose(fid));
    fprintf(fid,'%s\n',jsonencode(metadata,PrettyPrint=true));
end
