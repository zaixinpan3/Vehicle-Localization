addpath(pwd);setupVehicleLocalization;
out='output/localization_evaluation_20260923';matching='output/temporal_perception_20260922/five_frame_matching';
t0=tic;cache=prepareMississippiLocalizationClouds(matching);
save(fullfile(out,'sources.mat'),'-struct','cache','-v7.3');fprintf('Cache prepared in %.1f s\n',toc(t0));
report=runMncavFullObserverExperiment(string(out)+"/observer",MatchingFolder=matching, ...
    CoarseSourceFile=string(out)+"/sources.mat");
fprintf('DONE total %.1f s\n',toc(t0));
