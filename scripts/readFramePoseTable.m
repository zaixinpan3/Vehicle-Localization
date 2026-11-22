function framePoseTable = readFramePoseTable(poseMatchCsvPath, frameIndices)
% readFramePoseTable: Load the pre-extracted LiDAR-to-GNSS/INS pose
% match table and validate one global pose for every requested frame.
% Each frame carries synchronized pose_* translation and quaternion fields.
%
% Input:
%   poseMatchCsvPath: absolute path to the frame pose match CSV
%   frameIndices: [1 x F] requested one-based LiDAR frame indices
%
% Output:
%   framePoseTable: selected pose rows in the requested frame order
    assert(isfile(poseMatchCsvPath), "Pose match CSV not found: %s. Prepare the selected pose table first.", poseMatchCsvPath);
    framePoseTable = readtable(poseMatchCsvPath, "TextType", "string");
    required=["frame_index","pose_x_m","pose_y_m","pose_z_m","pose_qw","pose_qx","pose_qy","pose_qz"];
    assert(all(ismember(required,string(framePoseTable.Properties.VariableNames))), ...
        'VehicleLocalization:IncompletePose','The frame table must contain indexed, synchronized SE(3) poses.');
    availableFrames = double(framePoseTable.frame_index(:));
    assert(numel(unique(availableFrames))==numel(availableFrames), ...
        'VehicleLocalization:DuplicateFrame','Pose table has duplicate frame indices.');
    missingFrames = setdiff(double(frameIndices(:)), availableFrames);
    assert(isempty(missingFrames), "Pose match CSV does not contain requested frame(s): %s.", strjoin(string(missingFrames(:).'), ", "));
    [~, keepIdx] = ismember(double(frameIndices(:)), availableFrames);
    framePoseTable = framePoseTable(keepIdx, :);
    for k=1:height(framePoseTable),localizationSupport.poseRowToRigidTransform(framePoseTable(k,:));end
end
