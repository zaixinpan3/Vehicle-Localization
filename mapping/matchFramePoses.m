function poseMatchTable = matchFramePoses(lidarCsvPath, odomCsvPath, inspvaCsvPath, outputCsvPath)
% matchFramePoses: Pair every LiDAR frame timestamp with the nearest
% odometry and INSPVA rows extracted from the mapping-drive bag, producing
% the frame pose table that registers local observations into the global
% odom/UTM-like frame. The odometry provides metric position and a full
% orientation quaternion; INSPVA provides the heading fallback and attitude.
%
% Input:
%   lidarCsvPath: CSV with frame_index and stamp_sec per LiDAR frame
%   odomCsvPath: CSV with index, stamp_sec, x_m, y_m, z_m, qx, qy, qz, qw
%   inspvaCsvPath: CSV with index, stamp_sec, latitude_deg, longitude_deg,
%       height_m, roll_deg, pitch_deg, azimuth_deg, ins_status
%   outputCsvPath: optional destination CSV path; empty skips writing
%
% Output:
%   poseMatchTable: table with one matched pose row per LiDAR frame
    assert(isfile(lidarCsvPath), "LiDAR timestamp CSV not found: %s", lidarCsvPath);
    assert(isfile(odomCsvPath), "Odom CSV not found: %s", odomCsvPath);
    assert(isfile(inspvaCsvPath), "INSPVA CSV not found: %s", inspvaCsvPath);
    if nargin < 4
        outputCsvPath = "";
    end

    lidarTable = readtable(lidarCsvPath, "TextType", "string");
    odomTable = readtable(odomCsvPath, "TextType", "string");
    inspvaTable = readtable(inspvaCsvPath, "TextType", "string");
    validateRequiredColumns(lidarTable, ["frame_index", "stamp_sec"], "LiDAR timestamp CSV");
    validateRequiredColumns(odomTable, ["index", "stamp_sec", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw"], "Odom CSV");
    validateRequiredColumns(inspvaTable, ["index", "stamp_sec", "latitude_deg", "longitude_deg", "height_m", "roll_deg", "pitch_deg", "azimuth_deg", "ins_status"], "INSPVA CSV");

    lidarStamp = double(lidarTable.stamp_sec(:));
    odomStamp = double(odomTable.stamp_sec(:));
    inspvaStamp = double(inspvaTable.stamp_sec(:));
    assert(all(isfinite(lidarStamp)) && all(isfinite(odomStamp)) && all(isfinite(inspvaStamp)), ...
        "LiDAR, odom, and INSPVA timestamp columns must be finite.");

    nearestOdomRows = nearestTimestampRows(odomStamp, lidarStamp);
    nearestInspvaRows = nearestTimestampRows(inspvaStamp, lidarStamp);

    poseMatchTable = table();
    poseMatchTable.frame_index = double(lidarTable.frame_index(:));
    poseMatchTable.lidar_stamp_sec = lidarStamp;
    poseMatchTable.nearest_odom_index = double(odomTable.index(nearestOdomRows));
    poseMatchTable.odom_stamp_sec = odomStamp(nearestOdomRows);
    poseMatchTable.odom_dt_sec = abs(poseMatchTable.odom_stamp_sec - lidarStamp);
    poseMatchTable.odom_x_m = double(odomTable.x_m(nearestOdomRows));
    poseMatchTable.odom_y_m = double(odomTable.y_m(nearestOdomRows));
    poseMatchTable.odom_z_m = double(odomTable.z_m(nearestOdomRows));
    poseMatchTable.odom_qx = double(odomTable.qx(nearestOdomRows));
    poseMatchTable.odom_qy = double(odomTable.qy(nearestOdomRows));
    poseMatchTable.odom_qz = double(odomTable.qz(nearestOdomRows));
    poseMatchTable.odom_qw = double(odomTable.qw(nearestOdomRows));
    poseMatchTable.nearest_inspva_index = double(inspvaTable.index(nearestInspvaRows));
    poseMatchTable.inspva_stamp_sec = inspvaStamp(nearestInspvaRows);
    poseMatchTable.inspva_dt_sec = abs(poseMatchTable.inspva_stamp_sec - lidarStamp);
    poseMatchTable.latitude_deg = double(inspvaTable.latitude_deg(nearestInspvaRows));
    poseMatchTable.longitude_deg = double(inspvaTable.longitude_deg(nearestInspvaRows));
    poseMatchTable.height_m = double(inspvaTable.height_m(nearestInspvaRows));
    poseMatchTable.roll_deg = double(inspvaTable.roll_deg(nearestInspvaRows));
    poseMatchTable.pitch_deg = double(inspvaTable.pitch_deg(nearestInspvaRows));
    poseMatchTable.azimuth_deg = double(inspvaTable.azimuth_deg(nearestInspvaRows));
    poseMatchTable.ins_status = double(inspvaTable.ins_status(nearestInspvaRows));

    if strlength(string(outputCsvPath)) > 0
        outputDir = fileparts(outputCsvPath);
        if strlength(string(outputDir)) > 0 && ~isfolder(outputDir)
            mkdir(outputDir);
        end
        writetable(poseMatchTable, outputCsvPath);
    end
end

function nearestRows = nearestTimestampRows(sourceStamps, queryStamps)
% nearestTimestampRows: Resolve nearest source-row indices for a vector of
% query timestamps using sorted one-dimensional nearest-neighbor lookup.
%
% Input:
%   sourceStamps: [N x 1] finite timestamp vector
%   queryStamps: [M x 1] finite timestamp vector
%
% Output:
%   nearestRows: [M x 1] row indices into the unsorted sourceStamps vector
    [sortedStamps, sortOrder] = sort(double(sourceStamps(:)));
    sortedRows = double((1:numel(sourceStamps)).');
    sortedRows = sortedRows(sortOrder);
    nearestSortedRows = interp1(sortedStamps, (1:numel(sortedStamps)).', double(queryStamps(:)), "nearest", "extrap");
    nearestSortedRows = max(1, min(numel(sortedStamps), round(nearestSortedRows)));
    nearestRows = sortedRows(nearestSortedRows);
end

function validateRequiredColumns(inputTable, requiredColumns, tableDescription)
% validateRequiredColumns: Fail fast when an input table is missing a named
% column required for frame pose matching.
%
% Input:
%   inputTable: MATLAB table to validate
%   requiredColumns: string vector of required variable names
%   tableDescription: string scalar used in assertion messages
%
% Output:
%   none
    variableNames = string(inputTable.Properties.VariableNames);
    for columnName = string(requiredColumns(:)).'
        assert(any(variableNames == columnName), "%s is missing required column: %s.", char(string(tableDescription)), char(columnName));
    end
end
