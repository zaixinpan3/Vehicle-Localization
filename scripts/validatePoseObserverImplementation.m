function summaries = validatePoseObserverImplementation(sequenceFolder,outputFolder,options)
% validatePoseObserverImplementation Evaluate the two continuous data modes.
% Recorded signals are explicit offline reconstructions. A failed coverage or
% information contract is retained as a failed result, never filled by a pulse
% or another source. This function does not certify physical sensor continuity.
    arguments
        sequenceFolder (1,1) string
        outputFolder (1,1) string
        options.RerunPerception (1,1) logical = true
    end
    setupVehicleLocalization;
    if ~isfolder(outputFolder),mkdir(outputFolder);end
    sensors=fullfile(sequenceFolder,'sensors');parameters=fullfile(sequenceFolder,'vehicle_parameters.json');
    replayFolder=fullfile(outputFolder,'recursive_8threads');
    if options.RerunPerception
        replayMississippiLocalization(fullfile(sequenceFolder,'publishedCloud.mat'),sensors,replayFolder);
    else
        replayFolder=fullfile(sequenceFolder,'recursive_8threads');
    end
    rows=cell(2,1);index=0;
    for mode=["gnss","lidar"]
        index=index+1;file=fullfile(outputFolder,"continuous_"+mode+"_design.mat");
        designMncavReplayObserver(parameters,file,1,mode);
        report=runMncavObserverReplay(replayFolder,file,parameters,fullfile(outputFolder,"continuous_"+mode), ...
            mode,.15,SensorFolder=sensors);
        rows{index}=report.summary;
    end
    summaries=rows;
    file=fopen(fullfile(outputFolder,'continuous_summaries.json'),'w');assert(file>=0);
    cleanup=onCleanup(@() fclose(file));fprintf(file,'%s\n',jsonencode(summaries,PrettyPrint=true));
end
