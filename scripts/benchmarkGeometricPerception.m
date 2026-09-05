function report = benchmarkGeometricPerception(baselineFolder, outputFolder, calibration)
% benchmarkGeometricPerception: In-memory coarse timing and feature fidelity.
% baselineFolder is an immutable export of the preceding project revision,
% with the SAME native binary available. No frame loading is timed. Restore
% the caller's MATLAB path on completion or failure.
    arguments
        baselineFolder (1,1) string
        outputFolder (1,1) string
        calibration (1,1) struct = lidarFrameCalibrationConfig()
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    frames=[260 550 900 120 350 700 1050]; data=cell(size(frames));
    current=cell(size(frames)); currentFine=cell(size(frames));
    rows=zeros(numel(frames),6); cfg=perceptionConfig(); cfg.executionBackend="native";
    for k=1:numel(frames)
        data{k}=loadPointCloudFrame(fullfile(root,'data','raw','MissisipiPointClouds.mat'),frames(k));
        current{k}=perceiveCoarseProbabilityCloud(data{k},cfg);
        rows(k,1)=frames(k);
        rows(k,2)=1000*timeit(@() perceiveCoarseProbabilityCloud(data{k},cfg));
        fineCfg=cfg; fineCfg.executionMode="offline";
        currentFine{k}=perceiveFrame(data{k},fineCfg);
        fineCfg.frameCalibration=calibration;
        corrected=perceiveFrame(data{k},fineCfg);
        rows(k,5)=isequaln(currentFine{k}.featureMasks,corrected.featureMasks);
        rows(k,6)=isequaln(currentFine{k}.candidates,corrected.candidates);
    end
    originalPath=path; restore=onCleanup(@() path(originalPath));
    addpath(genpath(baselineFolder));
    assert(startsWith(string(which('perceiveFrame')),baselineFolder),'Baseline path did not take precedence.');
    previousCfg=perceptionConfig(); previousCfg.executionBackend="native";
    for k=1:numel(frames)
        previous=perceiveCoarseProbabilityCloud(data{k},previousCfg);
        rows(k,3)=1000*timeit(@() perceiveCoarseProbabilityCloud(data{k},previousCfg));
        rows(k,4)=isequaln(previous.components,current{k}.components);
    end
    clear restore
    report=array2table(rows,'VariableNames',{'frame','currentCoarseMs','previousCoarseMs', ...
        'identityComponentsExact','calibratedFineMasksExact','calibratedCandidatesExact'});
    assert(all(rows(:,4:6)==1,'all'),'Perception fidelity changed.');
    if ~isfolder(outputFolder), mkdir(outputFolder); end
    writetable(report,fullfile(outputFolder,'perception_fidelity_timing.csv'));
    disp(report);
end
