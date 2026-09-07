function results = runMississippiFullSequenceExperiment(outputFolder,parameterFile,rebuildMap)
% runMississippiFullSequenceExperiment Build, perceive, match, and observe.
% First export sensors with extractVehicleReplaySensors.py and produce the
% independent calibration/nominal-parameter JSON with calibrateMncavReplayInputs.py.
% Add YALMIP and SeDuMi before running. Mapping and query share this drive;
% results establish sequence consistency, not independent localization accuracy.
    arguments
        outputFolder (1,1) string
        parameterFile (1,1) string
        rebuildMap (1,1) logical = true
    end
    root=setupVehicleLocalization();
    threads=maxNumCompThreads; cleanup=onCleanup(@() maxNumCompThreads(threads));
    if ~isfolder(outputFolder), mkdir(outputFolder); end
    mapFile=fullfile(outputFolder,'probabilityCloudMap.mat');
    if rebuildMap
        cfg=featureMapBuildConfig(); cfg.mapOutputPath="";
        timer=tic;
        [probabilityCloudMap,featureData]=buildFeatureMap(fullfile(root,'data'),cfg);
        save(mapFile,'probabilityCloudMap','-v7.3');
        mappingSeconds=toc(timer);
        save(fullfile(outputFolder,'featureData.mat'),'featureData','cfg','mappingSeconds','-v7.3');
        writetable(featureData.frameSummaryTable,fullfile(outputFolder,'map_feature_counts.csv'));
    else
        loaded=load(mapFile,'probabilityCloudMap'); probabilityCloudMap=loaded.probabilityCloudMap;
        mappingSeconds=NaN;
    end
    probabilityCloud=temporalMapToProbabilityCloud(probabilityCloudMap);
    compactMap=fullfile(outputFolder,'publishedCloud.mat');
    save(compactMap,'probabilityCloud');
    writetable(probabilityCloudMap.layerSummaryTable,fullfile(outputFolder,'map_layers.csv'));
    summaryPath=fullfile(outputFolder,'map_summary.json');
    if rebuildMap || ~isfile(summaryPath)
        writeJson(summaryPath,struct('mappingSeconds',mappingSeconds, ...
            'frames',numel(probabilityCloudMap.frameIndices), ...
            'publishedComponents',probabilityCloud.components.numComponents,'totalMass',probabilityCloud.totalMass));
    end
    clear probabilityCloudMap probabilityCloud
    maxNumCompThreads(1);
    results.oneThread=replayMississippiLocalization(compactMap,fullfile(outputFolder,'sensors'), ...
        fullfile(outputFolder,'recursive'));
    maxNumCompThreads(8);
    replayFolder=fullfile(outputFolder,'recursive_8threads');
    results.d2d=replayMississippiLocalization(compactMap,fullfile(outputFolder,'sensors'),replayFolder);
    certificates=struct();
    for factor=[1 .7 1.3]
        designFile=fullfile(outputFolder,sprintf('mncavObserverDesign_%.1f.mat',factor));
        if factor==1, designFile=fullfile(outputFolder,'mncavObserverDesign.mat'); end
        designMncavReplayObserver(parameterFile,designFile,factor);
        design=load(designFile);
        name=matlab.lang.makeValidName(sprintf('factor_%.1f',factor));
        certificates.(name)=struct('factor',factor,'vehicle',design.lateralCfg.vehicle, ...
            'lateralCertified',design.lateralDesign.certified, ...
            'lateralWorstMargin',design.lateralDesign.maxCertificateMargin, ...
            'globalVerification',design.observerDesign.verification, ...
            'globalCfg',design.observerCfg,'synthesisSeconds',design.synthesisSeconds);
        if factor~=1
            report=runMncavObserverReplay(replayFolder,designFile,parameterFile, ...
                fullfile(outputFolder,sprintf('observer_fusion_%.1f',factor)),"fusion");
            results.(name)=report.summary;
        end
    end
    writeJson(fullfile(outputFolder,'design_certificates.json'),certificates);
    designFile=fullfile(outputFolder,'mncavObserverDesign.mat');
    for scenario=["gpsOnly","fusion","positionOutage","outageNoLidar"]
        report=runMncavObserverReplay(replayFolder,designFile,parameterFile, ...
            fullfile(outputFolder,"observer_"+scenario),scenario);
        results.(scenario)=report.summary;
    end
    report=runMncavObserverReplay(replayFolder,designFile,parameterFile, ...
        fullfile(outputFolder,'observer_fusion_delay0.30'),"fusion",.30);
    results.delay300ms=report.summary;
    results.outageCertificate=auditMncavOutageCertificate(designFile, ...
        fullfile(outputFolder,'outage_certificate_audit.csv'));
    plotMississippiExperiment(outputFolder);
end

function writeJson(path,value)
    fid=fopen(path,'w'); assert(fid>=0); cleanup=onCleanup(@() fclose(fid));
    fprintf(fid,'%s\n',jsonencode(value,PrettyPrint=true));
end
