function report=runDowntownReferenceExperiment(datasetRoot,outputRoot,options)
% runDowntownReferenceExperiment Build and replay every prepared Downtown bag.
% Add YALMIP and SeDuMi first. Fresh nominal lateral gains are shared within
% this operation; saved historical gain snapshots are never live defaults.
    arguments
        datasetRoot (1,1) string="data/processed/downtown_reference_gnss_free_20261005"
        outputRoot (1,1) string="output/downtown_reference_gnss_free_20261005"
        options.Only (1,1) string=""
    end
    setupVehicleLocalization();maxNumCompThreads(2);
    if ~isfolder(outputRoot),mkdir(outputRoot);end
    cfg=lateralObserverConfig();timer=tic;lateralDesign=designLateralObserverGains(cfg);synthesisSeconds=toc(timer);
    save(fullfile(outputRoot,'lateral_design.mat'),'lateralDesign','cfg','synthesisSeconds','-v7.3');
    folders=dir(fullfile(datasetRoot,'raw_data*'));records=cell(0,1);
    for k=1:numel(folders)
        name=string(folders(k).name);if strlength(options.Only)>0 && ~contains(name,options.Only),continue;end
        dataset=fullfile(datasetRoot,name);target=fullfile(outputRoot,name);mapFolder=fullfile(target,'map');localFolder=fullfile(target,'localization');
        if ~isfile(fullfile(dataset,'metadata.json')),continue;end
        record=struct('sequence',name,'status',"started");
        try
            if ~isfile(fullfile(mapFolder,'map_summary.json')),buildDowntownReferenceFeatureMap(dataset,mapFolder);end
            if ~isfile(fullfile(localFolder,'summary.json')),replayDowntownGnssFreeLocalization(dataset,fullfile(mapFolder,'probability_cloud.mat'),localFolder,lateralDesign);end
            record.metrics=scoreDowntownReferenceLocalization(dataset,localFolder);record.status="completed";
        catch exception
            record.status="failed";record.identifier=string(exception.identifier);record.error=string(exception.message);
            fprintf(2,'%s\n',getReport(exception,'extended','hyperlinks','off'));
        end
        records{end+1}=record; %#ok<AGROW>
        f=fopen(fullfile(outputRoot,'batch_status.json'),'w');fprintf(f,'%s\n',jsonencode(records,PrettyPrint=true));fclose(f);
    end
    report=struct('records',{records},'lateralSynthesisSeconds',synthesisSeconds,'gnssAvailable',false);
end
