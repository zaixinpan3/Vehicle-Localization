function cfg=carlaMapBuildConfig(perception)
% carlaMapBuildConfig Mapping configuration for CARLA Town10HD drives.
% Identical to featureMapBuildConfig except for the inputs: the semantic
% channels and LiDAR calibration of perceptionConfig("Carla","offline").
% Dataset paths are passed to the CARLA mapping functions directly.
    if nargin<1,perception=perceptionConfig("Carla","offline");end
    cfg=featureMapBuildConfig();
    cfg.pointCloudMatPath="";cfg.poseMatchCsvPath="";cfg.mapOutputPath="";
    cfg.baseProbabilityCloudPath="";cfg.featureObservationPath="";cfg.probabilityCloudPath="";
    cfg.featureNames=perception.featureNames;
    cfg.frameCalibration=perception.frameCalibration;
    cfg.logEnabled=false;
end
