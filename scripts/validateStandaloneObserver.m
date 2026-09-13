function report = validateStandaloneObserver(outputFolder)
% validateStandaloneObserver Exercise the actual continuous runner in two modes.
% Uses analytic truth and continuous measurement functions. Old outage, pulse,
% asynchronous and directional-pose campaigns are preserved in Git history.
    arguments
        outputFolder (1,1) string = "output/continuous_observer_runtime_20260913"
    end
    setupVehicleLocalization;
    if ~isfolder(outputFolder),mkdir(outputFolder);end
    results=cell(4,1);rows=cell(4,1);index=0;
    for mode=["gnss","lidar"]
        for noisy=[false,true]
            index=index+1;cfg=improvedObserverConfig(mode);
            if ~noisy,cfg.simulation.positionNoiseAmplitude=0;cfg.simulation.headingNoiseAmplitude=0;end
            design=improvedObserverReferenceDesign(cfg);
            started=tic;result=simulateImprovedObserverScenario(design,struct(),cfg);seconds=toc(started);
            row=result.metrics;row.mode=mode;row.noisy=noisy;row.runtimeSeconds=seconds;
            row.matrixCertificateVerified=result.estimate.observer.certificateVerified;
            row.coefficientBoundsSatisfied=result.estimate.diagnostics.certificateConditions.coefficientBoundsSatisfied;
            rows{index}=row;results{index}=result;
        end
    end
    % Refine the same smooth LiDAR scenario without changing its measurements.
    refinedCfg=results{end}.cfg;refinedCfg.measurement.maximumIntegrationStep=.0025;
    refined=simulateImprovedObserverScenario(improvedObserverReferenceDesign(refinedCfg),struct(),refinedCfg);
    refinement=max(abs(refined.estimate.z-results{end}.estimate.z),[],1);
    report=struct('metrics',struct2table([rows{:}]),'maximumStateRefinementDifference',refinement, ...
        'scope',"Continuous ODE/DDE simulation; finite experiments are not an unconditional ISS proof.");
    writetable(report.metrics,fullfile(outputFolder,'metrics.csv'));
    fid=fopen(fullfile(outputFolder,'summary.json'),'w');assert(fid>=0);
    cleanup=onCleanup(@() fclose(fid));
    fprintf(fid,'%s\n',jsonencode(struct('metrics',[rows{:}], ...
        'maximumStateRefinementDifference',refinement,'scope',report.scope),PrettyPrint=true));
    save(fullfile(outputFolder,'traces.mat'),'results','refined','report','-v7.3');
    disp(report.metrics);disp(refinement);
end
