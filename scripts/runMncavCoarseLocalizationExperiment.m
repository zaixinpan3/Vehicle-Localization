function report=runMncavCoarseLocalizationExperiment(outputFolder,options) %#ok<STOUT,INUSD>
% runMncavCoarseLocalizationExperiment Archived registration experiment.
% Historical implementation: Git 2d6dcf52e94c391a9538e9c9be59751d85c08cae.
% Current entry: runMncavFullObserverExperiment (overlapGradient only).
    arguments
        outputFolder (1,1) string="output/mncav_coarse_localization"
        options.MapFile (1,1) string=""
        options.FrameCalibration (1,1) struct=struct()
    end
    error('VehicleLocalization:ArchivedRegistration', ...
        'This registration experiment is archived. Run runMncavFullObserverExperiment with overlapGradient.');
end
