function prepare_sources()
% prepare_sources Save freshly recomputed horizons for the paired experiment.
    setupVehicleLocalization;out='output/gnss_aided_matching_20260922';
    if ~isfolder(out),mkdir(out);end
    cache=prepareMississippiLocalizationClouds('output/temporal_perception_20260922/five_frame_matching');
    save(fullfile(out,'sources.mat'),'-struct','cache','-v7.3');
    writetable(table(cache.calls.frame,cache.seconds,cache.counts, ...
        VariableNames={'frame','seconds','confirmedComponents'}), ...
        fullfile(fileparts(mfilename('fullpath')),'source_preparation.csv'));
end
