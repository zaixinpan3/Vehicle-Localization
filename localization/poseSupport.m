classdef poseSupport
% poseSupport Conversions of recorded pose-table rows.
% Static methods return the planar [X Y psi] pose with map height and the rigid
% transform of one recorded navigation pose row.
% Example: [pose,~,z] = poseSupport.poseRowToPlanarPose(row).

    methods (Static)
        function [pose, tiltRotation, heightTranslation] = poseRowToPlanarPose(row)
        % poseRowToPlanarPose Split the selected mapping pose into SE(2) and tilt.
        % Uses exactly the transform consumed by registerPointsToGlobalFrame.
        % R_global = R_yaw * tiltRotation. Height is external mapping metadata,
        % not a registration state; it is NaN for legacy records without height.
            [rotation,translation]=poseSupport.poseRowToRigidTransform(row);
            yaw=atan2(rotation(2,1),rotation(1,1));
            r=[cos(yaw),-sin(yaw),0;sin(yaw),cos(yaw),0;0,0,1];
            tiltRotation=r.'*rotation;
            pose=[translation(1:2),yaw];
            heightTranslation=translation(3);
        end

        function [rotation,translation] = poseRowToRigidTransform(row)
        % poseRowToRigidTransform Resolve explicit mapping poses or legacy ODOM rows.
        % Canonical pose_* fields take precedence and must be complete and finite.
        % Legacy records retain their original quaternion or yaw-only interpretation.
            names=string(row.Properties.VariableNames);
            assert(height(row)==1,'VehicleLocalization:PoseRow','Exactly one pose row is required.');
            canonical=any(startsWith(names,"pose_"));
            if canonical
                fields=["pose_x_m","pose_y_m","pose_z_m","pose_qw","pose_qx","pose_qy","pose_qz"];
                assert(all(ismember(fields,names)),'VehicleLocalization:IncompletePose','Canonical pose is incomplete.');
                values=double(row{1,cellstr(fields)});
                assert(all(isfinite(values)),'VehicleLocalization:InvalidPose','Canonical pose must be finite.');
                translation=values(1:3);q=values(4:7);
                assert(norm(q)>0,'VehicleLocalization:InvalidPose','Quaternion must be nonzero.');
            else
                translation=[double(row.odom_x_m(1)),double(row.odom_y_m(1)),NaN];
                if ismember("odom_z_m",names),translation(3)=double(row.odom_z_m(1));end
                q=[];fields=["odom_qw","odom_qx","odom_qy","odom_qz"];
                if all(ismember(fields,names))
                    candidate=double(row{1,cellstr(fields)});
                    if all(isfinite(candidate)) && norm(candidate)>0,q=candidate;end
                end
                if isempty(q)
                    yaw=deg2rad(90-double(row.azimuth_deg(1)));
                    rotation=[cos(yaw),-sin(yaw),0;sin(yaw),cos(yaw),0;0,0,1];
                    return;
                end
            end
            q=q/norm(q);w=q(1);x=q(2);y=q(3);z=q(4);
            rotation=[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w); ...
                2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w); ...
                2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)];
        end
    end
end
