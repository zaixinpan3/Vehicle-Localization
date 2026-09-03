function framePoseTable = readFramePoseTable(poseMatchCsvPath, frameIndices)
% readFramePoseTable: Load the pre-extracted LiDAR-to-GNSS/INS pose
% match table and validate that it contains one global pose row for every
% requested point-cloud frame. The projection uses odom x/y/z as global
% metric position and INSPVA azimuth as heading, where azimuth is measured
% clockwise from true north to vehicle forward direction.
%
% Input:
%   poseMatchCsvPath: absolute path to the frame pose match CSV
%   frameIndices: [1 x F] requested one-based LiDAR frame indices
%
% Output:
%   framePoseTable: table with frame_index, odom position, and INSPVA
%       roll/pitch/azimuth fields for the requested frames
    assert(isfile(poseMatchCsvPath), "Pose match CSV not found: %s. Run scripts/extractMissisipiGnssFromBag.py first.", poseMatchCsvPath);
    framePoseTable = readtable(poseMatchCsvPath, "TextType", "string");
    requiredFields = ["frame_index", "odom_x_m", "odom_y_m", "odom_z_m", "azimuth_deg", "roll_deg", "pitch_deg"];
    for fieldName = requiredFields
        assert(any(string(framePoseTable.Properties.VariableNames) == fieldName), ...
            "Pose match CSV is missing required field: %s.", fieldName);
    end
    availableFrames = double(framePoseTable.frame_index(:));
    missingFrames = setdiff(double(frameIndices(:)), availableFrames);
    assert(isempty(missingFrames), "Pose match CSV does not contain requested frame(s): %s.", strjoin(string(missingFrames(:).'), ", "));
    [~, keepIdx] = ismember(double(frameIndices(:)), availableFrames);
    framePoseTable = framePoseTable(keepIdx, :);
end
