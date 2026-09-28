function report = runStudy()
% runStudy Replay all Mississippi scans with the current production defaults.
% External solver dependencies are loaded only for fresh MnCAV gain synthesis.
    root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    cd(root);
    setupVehicleLocalization();
    external = fullfile(fileparts(root),'RobustVehicleLocalization','external');
    addpath(genpath(fullfile(external,'YALMIP')));
    addpath(genpath(fullfile(external,'sedumi')));
    assert(exist('sdpvar','file') == 2 && exist('sedumi','file') == 2);
    set(groot,'defaultFigureVisible','off');
    output = 'output/mississippi_matching_20260928';
    cfg = perceptionConfig('Mississippi');
    assert(all(cfg.voxel.voxelSize == .6));
    assert(~isfile(fullfile(output,'experiment.mat')), 'Do not overwrite a completed run.');
    timer = tic;
    report = runMississippiMapMatchingExperiment(output);
    fprintf('FULL_EXPERIMENT_COMPLETED %.3f seconds\n',toc(timer));
end
