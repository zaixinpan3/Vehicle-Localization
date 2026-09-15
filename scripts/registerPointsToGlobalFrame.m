function globalPoints = registerPointsToGlobalFrame(localPoints, poseRow)
% registerPointsToGlobalFrame Transform local feature points with a map pose.
% Points use x forward, y left, z up. Explicit pose_* XYZ/quaternion fields
% take precedence; legacy ODOM records retain their recorded interpretation.
    if isempty(localPoints)
        globalPoints=zeros(0,3);
        return;
    end
    [rotation,translation]=poseRowToRigidTransform(poseRow);
    assert(all(isfinite(translation)), 'VehicleLocalization:MissingHeight', ...
        'Global point projection requires finite XYZ translation.');
    globalPoints=double(localPoints)*rotation.'+translation;
end
