function runViewMapControls()
% runViewMapControls Separate conditional means, scatter, and map refitting.
    setupVehicleLocalization();out='output/matching_objective_20260929';file=fullfile(out,'viewConditioned_map.mat');
    a=load(file,'viewModel');cfg=distributionRegistrationConfig();cfg.pyramid.mapMergeRadius=0;
    source='output/root_cause_matching_20260929/finalSurface_sources.mat';
    for label=["viewStaticCanonical","viewGlobalEqualScan","viewMeanOnly","viewScatterOnly","viewMap3"]
        model=a.viewModel;
        switch label
            case "viewStaticCanonical",replayStudy(source,label,cfg,file);continue;
            case "viewGlobalEqualScan",model.bandwidth=1e6;model.maximumHeadingDifference=pi;
            case "viewMeanOnly",model.updateCovariance=false;
            case "viewScatterOnly",model.updateMean=false;
            case "viewMap3",model.bandwidth=3;
        end
        replayStudy(source,label,cfg,file,model);
    end
    replayStudy('output/pole_boundary_recovery_20260929/replay.mat',"viewEmpiricalCurb",cfg,file,a.viewModel);
end
