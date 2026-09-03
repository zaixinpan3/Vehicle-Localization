% buildMississippiFeatureMap: Build the sliding-window semantic
% temporal-stability probability-cloud map of the Mississippi route from the
% extracted point-cloud frames and the matched GNSS/INS poses. Point dataRoot
% at the folder that holds raw/MissisipiPointClouds.mat and the raw/Missisipi
% GNSS extracts (see scripts/extractGnssFromBag.py); the pose match table is
% built with matchFramePoses when it does not exist yet.
%
% Input:
%   dataRoot: optional workspace variable, defaults to ../data
%
% Output:
%   probabilityCloudMap and featureData in the workspace, and the saved map
%       artifact under dataRoot
setupVehicleLocalization();
if ~exist("dataRoot", "var") || strlength(string(dataRoot)) == 0
    dataRoot = fullfile(fileparts(fileparts(mfilename("fullpath"))), "data");
end
cfg = featureMapBuildConfig();

poseMatchCsvPath = fullfile(dataRoot, cfg.poseMatchCsvPath);
if ~isfile(poseMatchCsvPath)
    gnssDir = fileparts(poseMatchCsvPath);
    bagStem = "raw_data_2024-06-07-12-09-31_0";
    matchFramePoses(fullfile(gnssDir, bagStem + "_front_lidar_points.csv"), ...
        fullfile(gnssDir, bagStem + "_odom.csv"), fullfile(gnssDir, bagStem + "_inspva.csv"), poseMatchCsvPath);
end

[probabilityCloudMap, featureData] = buildFeatureMap(dataRoot, cfg);
disp(probabilityCloudMap.layerSummaryTable);
