function [pose, tiltRotation, heightTranslation] = poseRowToPlanarPose(row)
% poseRowToPlanarPose Split the selected mapping pose into SE(2) and tilt.
% Uses exactly the transform consumed by registerPointsToGlobalFrame.
% R_global = R_yaw * tiltRotation. Height is external mapping metadata,
% not a registration state; it is NaN for legacy records without height.
    [rotation,translation]=poseRowToRigidTransform(row);
    yaw=atan2(rotation(2,1),rotation(1,1));
    r=[cos(yaw),-sin(yaw),0;sin(yaw),cos(yaw),0;0,0,1];
    tiltRotation=r.'*rotation;
    pose=[translation(1:2),yaw];
    heightTranslation=translation(3);
end
