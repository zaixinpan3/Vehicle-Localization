function cfg = featureMapBuildConfig()
% featureMapBuildConfig: Offline mapping run parameters for the Mississippi
% route: dataset files, the covered frame range, the sliding-window batch
% schedule, the semantic feature classes registered into the map, and the
% per-class point thresholds applied before Gaussian support map building.
% Paths are relative to the data root passed to the mapping functions.
%
% Input:
%   none
%
% Output:
%   cfg: struct consumed by buildFeatureMap and collectFeatureObservations
    cfg = struct();

    % Dataset: organized point-cloud frames and the LiDAR-to-GNSS/INS pose table
    cfg.pointCloudMatPath = fullfile("raw", "MissisipiPointClouds.mat");
    cfg.poseMatchCsvPath = fullfile("raw", "Missisipi", "gnss", "raw_data_2024-06-07-12-09-31_0_front_lidar_pose_match_1_1170.csv");
    cfg.mapOutputPath = "missisipiTemporalStabilityProbabilityCloudMap.mat";
    cfg.frameCalibration = lidarFrameCalibrationConfig();

    % Covered frames: empty frameIndices means every frame of the MAT file
    cfg.frameIndices = [];

    % Sliding-window batches: 30-frame windows advancing by 20 frames
    cfg.batchFrameCount = 30;
    cfg.batchFrameStride = 20;

    % Semantic feature classes registered into the map (facades are off on this route)
    cfg.featureNames = ["curb", "roadMarking", "pole"];
    cfg.facadeDetectionEnabled = false;

    % Deterministic representatives, observation blocks, and tile inference.
    % Sparse classes are retained as empty/unconfirmed layers.
    cfg.temporalMap = temporalStabilityMapConfig();
    cfg.logEnabled = true;
end
