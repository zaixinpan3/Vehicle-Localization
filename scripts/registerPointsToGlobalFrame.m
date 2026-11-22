function globalPoints = registerPointsToGlobalFrame(localPoints, poseRow)
% registerPointsToGlobalFrame Transform local feature points with a map pose.
% Points use x forward, y left, z up. The map pose uses synchronized XYZ and quaternion fields.
    if isempty(localPoints)
        globalPoints=zeros(0,3);
        return;
    end
    [rotation,translation]=localizationSupport.poseRowToRigidTransform(poseRow);
    globalPoints=double(localPoints)*rotation.'+translation;
end
