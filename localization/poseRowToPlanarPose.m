function [pose, tiltRotation, heightTranslation] = poseRowToPlanarPose(row)
% poseRowToPlanarPose: Split a mapping pose into SE(2) and known IMU tilt.
% Uses the same vehicle x-forward/y-left/z-up convention and quaternion as
% registerPointsToGlobalFrame. R_global = R_yaw * tiltRotation. Applying
% tiltRotation to source moments before XY projection makes D2D consistent
% with the full 3D offline map transform. Azimuth-only records have zero tilt.
% Third output heightTranslation is the moving-origin map Z, or NaN when
% absent. It is an external reference, not an estimated registration state.
    fields=["odom_qw" "odom_qx" "odom_qy" "odom_qz"];
    rotation=[];
    if all(ismember(fields,string(row.Properties.VariableNames)))
        q=double(row{1,cellstr(fields)});
        if all(isfinite(q)) && norm(q)>0
            q=q/norm(q); w=q(1); x=q(2); y=q(3); z=q(4);
            rotation=[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w); ...
                2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w); ...
                2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)];
        end
    end
    if isempty(rotation)
        yaw=deg2rad(90-double(row.azimuth_deg(1)));
        tiltRotation=eye(3);
    else
        yaw=atan2(rotation(2,1),rotation(1,1));
        r=[cos(yaw) -sin(yaw) 0;sin(yaw) cos(yaw) 0;0 0 1];
        tiltRotation=r.'*rotation;
    end
    pose=[double(row.odom_x_m(1)),double(row.odom_y_m(1)),yaw];
    heightTranslation = NaN;
    if ismember("odom_z_m",string(row.Properties.VariableNames))
        heightTranslation = double(row.odom_z_m(1));
    end
end
