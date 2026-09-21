function report=runMississippiMapMatchingExperiment(outputFolder,options)
% runMississippiMapMatchingExperiment Evaluate raw coarse D2D pose measurements.
% No global GNSS/LiDAR observer is run. Wheel/gyro/lateral motion supplies only
% recursive initial guesses; accepted matching poses are scored without fusion.
% A separate reference-seeded control diagnoses local matching convergence.
% Rejected frames have no full-pose measurement and are counted separately.
    arguments
        outputFolder (1,1) string="output/mississippi_matching_only"
        options.RegistrationConfig (1,1) struct=distributionRegistrationConfig()
        options.SourceWindowConfig (1,1) struct=localizationSourceWindowConfig()
    end
    setupVehicleLocalization();
    if ~isfolder(outputFolder),mkdir(outputFolder);end
    mapFile="output/mississippi_mapping_synchronized/probability_cloud.mat";
    sensorFolder="output/mncav_wheel_only_20260916/sensors";
    parameterFile="output/mncav_interface_audit_20260916/vehicle_parameters.json";
    [prepared,~,~]=prepareMncavObserverReplay(sensorFolder,parameterFile,table(),0,IncludeOdom=false);
    saved=load('output/mncav_inspva_observer_20260915/experiment.mat','lateralDesign');
    lateral=runLateralVelocityObserver(prepared.highRate,saved.lateralDesign,saved.lateralDesign.cfg);
    h=prepared.highRate;
    motion=struct('time',h.time,'longitudinalSpeed',h.longitudinalSpeed, ...
        'lateralVelocity',lateral.lateralVelocity,'yawRate',h.yawRate, ...
        'longitudinalVelocitySource',"four_wheel",'clockModelId',prepared.clockModelId);
    modes=["recursive","referenceSeed"];runs=cell(2,1);rows=cell(0,10);
    for k=1:numel(modes)
        result=replayMississippiLocalization(mapFile,sensorFolder,fullfile(outputFolder,modes(k)), ...
            modes(k),[],MotionInputs=motion,RegistrationConfig=options.RegistrationConfig, ...
            SourceWindowConfig=options.SourceWindowConfig);runs{k}=result;
        c=result.calls;
        for population=["accepted_matching_measurements","all_outputs_with_prediction"]
            selected=true(height(c),1);
            if population=="accepted_matching_measurements",selected=c.accepted;end
            e=c.positionErrorM(selected);yaw=c.yawErrorDeg(selected);
            dx=c.x(selected)-c.referenceX(selected);dy=c.y(selected)-c.referenceY(selected);
            rows(end+1,:)={modes(k),population,nnz(selected),rms(e),median(e),prctile(e,95), ...
                max(e),rms(yaw),rms(dx),rms(dy)}; %#ok<AGROW>
        end
    end
    metrics=cell2table(rows,VariableNames={'mode','population','samples','positionRmseM', ...
        'positionMedianM','positionP95M','positionMaximumM','headingRmseDeg','mapXRmseM','mapYRmseM'});
    report=struct('metadata',struct('rawFramesPerMode',height(runs{1}.calls), ...
        'perceptionMode',"coarseProbabilityCloud",'finePerceptionUsed',false, ...
        'registrationMethod',options.RegistrationConfig.method, ...
        'sourceWindow',options.SourceWindowConfig, ...
        'globalFusionObserverUsed',false,'mapRebuilt',false,'mapFile',mapFile, ...
        'primaryMode',"recursive",'reference',runs{1}.metadata.reference, ...
        'referenceControl',"Every frame starts at reference plus [0.5 m,-0.4 m,2 degrees]; diagnostic only", ...
        'evaluation',"Direct map-frame XY and wrapped yaw discrepancies at acquisition time; no trajectory alignment", ...
        'rejectedHandling',"Absent matching measurements; propagated/seeded outputs scored separately", ...
        'mapOverlap',runs{1}.metadata.mapOverlap,'randomness',"None"),'metrics',metrics);
    save(fullfile(outputFolder,'experiment.mat'),'runs','report','-v7.3');
    writetable(metrics,fullfile(outputFolder,'metrics.csv'));
    fid=fopen(fullfile(outputFolder,'summary.json'),'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));disp(metrics);
end
