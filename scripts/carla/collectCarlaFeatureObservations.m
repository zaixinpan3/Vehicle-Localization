function featureData=collectCarlaFeatureObservations(datasetFolder,frameIndices,outputFile)
% collectCarlaFeatureObservations Offline CARLA observations for one block of sweeps.
% Runs perceptionConfig("Carla","offline") through collectFeatureObservations:
% curb, pole and facade points are registered with the ground-truth pose of
% the kinematic reference point and the CARLA LiDAR calibration. Blocks of
% one drive can run in separate MATLAB processes; buildCarlaFeatureMap merges
% them. frameIndices are sweep numbers (rows of poses.csv).
    arguments
        datasetFolder (1,1) string
        frameIndices (1,:) double
        outputFile (1,1) string=""
    end
    setupVehicleLocalization();
    perception=perceptionConfig("Carla","offline");
    cfg=carlaMapBuildConfig(perception);
    poses=readFramePoseTable(fullfile(datasetFolder,'poses.csv'),frameIndices);
    featureData=collectFeatureObservations(fullfile(datasetFolder,'pointClouds.mat'),frameIndices,poses,perception,cfg);
    if strlength(outputFile)>0
        folder=fileparts(outputFile);
        if strlength(folder)>0 && ~isfolder(folder),mkdir(folder);end
        save(outputFile,'featureData','-v7.3');
    end
end
