function report=runDowntownReferenceExperiment(datasetRoot,outputRoot,options)
% runDowntownReferenceExperiment Build and replay every prepared Downtown bag.
% Add YALMIP and SeDuMi first. Fresh nominal lateral gains are shared within
% this operation; saved historical gain snapshots are never live defaults.
    arguments
        datasetRoot (1,1) string="data/processed/downtown_reference_gnss_free_20261005"
        outputRoot (1,1) string="output/downtown_exact_start_gnss_free_20261005"
        options.Only (1,1) string=""
        options.MapRoot (1,1) string=""
    end
    setupVehicleLocalization();maxNumCompThreads(2);
    if ~isfolder(outputRoot),mkdir(outputRoot);end
    cfg=lateralObserverConfig();timer=tic;lateralDesign=designLateralObserverGains(cfg);synthesisSeconds=toc(timer);
    save(fullfile(outputRoot,'lateral_design.mat'),'lateralDesign','cfg','synthesisSeconds','-v7.3');
    folders=dir(fullfile(datasetRoot,'raw_data*'));records=cell(0,1);
    for k=1:numel(folders)
        name=string(folders(k).name);if strlength(options.Only)>0 && ~contains(name,options.Only),continue;end
        dataset=fullfile(datasetRoot,name);target=fullfile(outputRoot,name);mapFolder=fullfile(target,'map');localFolder=fullfile(target,'localization');
        if strlength(options.MapRoot)>0,mapFolder=fullfile(options.MapRoot,name,'map');end
        if ~isfile(fullfile(dataset,'metadata.json')),continue;end
        record=struct('sequence',name,'status',"started");
        try
            if ~isfile(fullfile(mapFolder,'map_summary.json'))
                assert(strlength(options.MapRoot)==0,'VehicleLocalization:MissingFrozenMap','The supplied frozen map is unavailable.');
                buildDowntownReferenceFeatureMap(dataset,mapFolder);
            end
            inputs=prepareDowntownMotionInputs(dataset,lateralDesign);[ids,native]=downtownReplayFrameSchedule(dataset,inputs);
            packet=readDowntownInitialPose(dataset,ids(1),native(1));
            if isfile(fullfile(localFolder,'summary.json'))
                existing=jsondecode(fileread(fullfile(localFolder,'summary.json')));
                assert(isfield(existing,'initializationMode') && string(existing.initializationMode)=="reference_pose_once" && ...
                    existing.referenceInitializationCount==1 && ~existing.referenceUsedAfterInitialization && ...
                    norm(existing.initialPose(:)-packet.poseXYTheta(:))<1e-10, ...
                    'VehicleLocalization:StaleInitialization','Existing output uses a different initialization; choose a new output root.');
                validateDowntownInitialPose(existing.initialPosePacket,ids(1),native(1));
            else
                replayDowntownGnssFreeLocalization(dataset,fullfile(mapFolder,'probability_cloud.mat'),localFolder,lateralDesign,InitialPosePacket=packet);
            end
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
