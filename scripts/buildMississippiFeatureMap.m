% buildMississippiFeatureMap: Build the sliding-window semantic
% temporal-stability probability-cloud map of the Mississippi route from the
% extracted point-cloud frames and the matched GNSS/INS poses. Point dataRoot
% at the folder that holds raw/MissisipiPointClouds.mat and the raw/Missisipi
% GNSS extracts (see scripts/extractGnssFromBag.py); the pose match table is
% built from INSPVA with prepareInspvaMappingPoses.py when absent.
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
    script=fullfile(fileparts(mfilename('fullpath')),'prepareInspvaMappingPoses.py');
    command=strjoin(["uv run --offline --with numpy --with pyproj python",shellQuote(script), ...
        "--lidar",shellQuote(fullfile(gnssDir,bagStem+"_front_lidar_points.csv")), ...
        "--inspva",shellQuote(fullfile(gnssDir,bagStem+"_inspva.csv")),"--output",shellQuote(poseMatchCsvPath)]);
    [status,message]=system(command);
    assert(status==0,'VehicleLocalization:InspvaPreparation','INSPVA pose preparation failed: %s',message);
end

[probabilityCloudMap, featureData] = buildFeatureMap(dataRoot, cfg);
disp(probabilityCloudMap.layerSummaryTable);

function value=shellQuote(value)
% Quote a POSIX shell argument, including literal apostrophes.
    value="'"+replace(string(value),"'","'""'""'")+"'";
end
