function runSupportFullLocalization()
% runSupportFullLocalization Run the current raw-scan full observer cascade.
% Use current MnCAV configuration and freshly synthesized gains. Preserve
% existing experiment folders and avoid opening visualization windows.
    setupVehicleLocalization();
    folder='output/support_full_localization_20260930';
    if ~isfolder(folder),mkdir(folder);end
    previous=get(groot,'DefaultFigureVisible');
    cleanup=onCleanup(@()set(groot,'DefaultFigureVisible',previous));
    set(groot,'DefaultFigureVisible','off');
    assert(distributionRegistrationConfig().method=="supportD2D");
    report=runMncavCoarseLocalizationExperiment(folder);
    save(fullfile(folder,'report.mat'),'report');
    close all;
end
