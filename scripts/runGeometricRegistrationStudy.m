function reports = runGeometricRegistrationStudy(outputFolder)
% runGeometricRegistrationStudy: Reproduce paired identity/calibrated experiments.
% Fits curb pitch only on frames 260:266, then runs three starts on seven
% query-excluded map windows. Six query windows are outside the fit interval.
% Raw inputs and fitted maps remain in the local, ignored output directory.
    arguments
        outputFolder (1,1) string = "output/geometric_registration"
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    if ~isfolder(outputFolder), mkdir(outputFolder); end
    cfg=featureMapBuildConfig(); cfg.logEnabled=false;
    frames=260:266;
    poses=readFramePoseTable(fullfile(root,'data',cfg.poseMatchCsvPath),frames);
    observations=collectFeatureObservations(fullfile(root,'data',cfg.pointCloudMatPath),frames,poses,perceptionConfig(),cfg);
    [vlGeoCalibration,vlGeoCalibrationFit]=fitLidarPitchCalibration(observations, ...
        Identifier="Mississippi260CurbPitchCandidate");
    save(fullfile(outputFolder,'pitch_calibration.mat'),'vlGeoCalibration','vlGeoCalibrationFit');
    frames=[260 550 900 120 350 700 1050];
    reports.identity=evaluateGeometricRegistration(outputFolder,frames);
    reports.calibrated=evaluateGeometricRegistration(outputFolder,frames,vlGeoCalibration);
    reports.calibrationFit=vlGeoCalibrationFit;
end
