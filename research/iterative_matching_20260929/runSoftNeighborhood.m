function runSoftNeighborhood()
% runSoftNeighborhood Check whether the remaining peak has a stable local improvement.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));summaries=table();source='output/pole_boundary_recovery_20260929/replay.mat';
    saved=load('output/iterative_matching_20260929/confidentSoft200.mat','cfg');
    settings=[1.5 1.5 .9;2.5 1.5 .9;3 1.5 .9;2 .75 .9;2 1 .9;2 2 .9;2 1.5 .8;2 1.5 .95];
    for k=1:size(settings,1)
        cfg=saved.cfg;cfg.lineDirection.scatterScale=.25;
        cfg.softPointAssociation.temperature=settings(k,1);cfg.softPointAssociation.radius=settings(k,2);cfg.softPointAssociation.minimumPosterior=settings(k,3);
        row=replayCloudVariant(source,"softNeighborhood"+k,cfg);summaries=[summaries;row]; %#ok<AGROW>
        writetable(summaries,fullfile(dest,'soft_neighborhood_screen.csv'));
    end
end
