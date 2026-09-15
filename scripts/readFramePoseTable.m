function framePoseTable = readFramePoseTable(poseMatchCsvPath, frameIndices)
% readFramePoseTable: Load the pre-extracted LiDAR-to-GNSS/INS pose
% match table and validate one global pose for every requested frame.
% Explicit pose_* XYZ/quaternion fields are preferred. Legacy ODOM tables
% remain readable for reproducing historical maps.
%
% Input:
%   poseMatchCsvPath: absolute path to the frame pose match CSV
%   frameIndices: [1 x F] requested one-based LiDAR frame indices
%
% Output:
%   framePoseTable: selected pose rows in the requested frame order
    assert(isfile(poseMatchCsvPath), "Pose match CSV not found: %s. Prepare the selected pose table first.", poseMatchCsvPath);
    framePoseTable = readtable(poseMatchCsvPath, "TextType", "string");
    if any(startsWith(string(framePoseTable.Properties.VariableNames),"pose_"))
        requiredFields=["frame_index","pose_x_m","pose_y_m","pose_z_m","pose_qw","pose_qx","pose_qy","pose_qz"];
    else
        requiredFields=["frame_index","odom_x_m","odom_y_m","odom_z_m","azimuth_deg","roll_deg","pitch_deg"];
    end
    for fieldName = requiredFields
        assert(any(string(framePoseTable.Properties.VariableNames) == fieldName), ...
            "Pose match CSV is missing required field: %s.", fieldName);
    end
    availableFrames = double(framePoseTable.frame_index(:));
    assert(numel(unique(availableFrames))==numel(availableFrames), ...
        'VehicleLocalization:DuplicateFrame','Pose table has duplicate frame indices.');
    missingFrames = setdiff(double(frameIndices(:)), availableFrames);
    assert(isempty(missingFrames), "Pose match CSV does not contain requested frame(s): %s.", strjoin(string(missingFrames(:).'), ", "));
    [~, keepIdx] = ismember(double(frameIndices(:)), availableFrames);
    framePoseTable = framePoseTable(keepIdx, :);
    for k=1:height(framePoseTable),poseRowToRigidTransform(framePoseTable(k,:));end
end
