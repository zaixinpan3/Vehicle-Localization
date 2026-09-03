function globalPoints = registerPointsToGlobalFrame(localPoints, poseRow)
% registerPointsToGlobalFrame: Project feature points from the local
% vehicle frame into the global odom/UTM-like metric frame using the
% matched GNSS/INS vehicle pose. The local point convention is x forward,
% y left, z up. The full odom quaternion is used when available. If only
% INSPVA azimuth is available, azimuth is interpreted as the clockwise
% angle from true north to vehicle forward, so the ENU/UTM yaw is
% 90 degrees minus azimuth.
%
% Input:
%   localPoints: [N x 3] feature points in the vehicle-local frame
%   poseRow: one-row table with odom position and either odom quaternion
%       fields or azimuth_deg
%
% Output:
%   globalPoints: [N x 3] feature points in the global metric frame
    if isempty(localPoints)
        globalPoints = zeros(0, 3);
        return;
    end
    localPoints = double(localPoints);
    translation = [double(poseRow.odom_x_m(1)), double(poseRow.odom_y_m(1)), double(poseRow.odom_z_m(1))];
    if hasOdomQuaternion(poseRow)
        rotationMatrix = quaternionToRotationMatrix([double(poseRow.odom_qw(1)), double(poseRow.odom_qx(1)), double(poseRow.odom_qy(1)), double(poseRow.odom_qz(1))]);
        globalPoints = localPoints * rotationMatrix.' + translation;
    else
        globalPoints = projectPointsWithGnssAzimuth(localPoints, translation, double(poseRow.azimuth_deg(1)));
    end
end

function tf = hasOdomQuaternion(poseRow)
% hasOdomQuaternion: Determine whether one pose-table row contains a
% finite odom quaternion that can be used for full 3D local-to-global
% projection.
%
% Input:
%   poseRow: one-row table with optional odom_qw, odom_qx, odom_qy, odom_qz
%
% Output:
%   tf: logical scalar true when a finite nonzero quaternion is available
    requiredFields = ["odom_qw", "odom_qx", "odom_qy", "odom_qz"];
    tf = all(ismember(requiredFields, string(poseRow.Properties.VariableNames)));
    if ~tf
        return;
    end
    quaternion = [double(poseRow.odom_qw(1)), double(poseRow.odom_qx(1)), double(poseRow.odom_qy(1)), double(poseRow.odom_qz(1))];
    tf = all(isfinite(quaternion)) && norm(quaternion) > 0;
end

function rotationMatrix = quaternionToRotationMatrix(quaternionWxyz)
% quaternionToRotationMatrix: Convert a [w x y z] unit or non-unit
% quaternion into a 3D rotation matrix without relying on toolbox-specific
% quaternion helpers.
%
% Input:
%   quaternionWxyz: [1 x 4] quaternion values in [w x y z] order
%
% Output:
%   rotationMatrix: [3 x 3] local-to-global rotation matrix
    quaternionWxyz = double(quaternionWxyz(:).');
    assert(numel(quaternionWxyz) == 4 && all(isfinite(quaternionWxyz)) && norm(quaternionWxyz) > 0, ...
        "A finite nonzero [w x y z] quaternion is required.");
    quaternionWxyz = quaternionWxyz ./ norm(quaternionWxyz);
    w = quaternionWxyz(1);
    x = quaternionWxyz(2);
    y = quaternionWxyz(3);
    z = quaternionWxyz(4);
    rotationMatrix = [ ...
        1 - 2 .* (y .* y + z .* z), 2 .* (x .* y - z .* w), 2 .* (x .* z + y .* w); ...
        2 .* (x .* y + z .* w), 1 - 2 .* (x .* x + z .* z), 2 .* (y .* z - x .* w); ...
        2 .* (x .* z - y .* w), 2 .* (y .* z + x .* w), 1 - 2 .* (x .* x + y .* y)];
end

function globalPoints = projectPointsWithGnssAzimuth(localPoints, translation, azimuthDeg)
% projectPointsWithGnssAzimuth: Apply a yaw-only local-to-global
% projection from NovAtel INSPVA heading when a full odom quaternion is not
% available. The azimuth convention is clockwise from true north to vehicle
% forward, while global x/y are easting/northing.
%
% Input:
%   localPoints: [N x 3] points in x-forward, y-left, z-up local frame
%   translation: [1 x 3] global easting, northing, and height
%   azimuthDeg: scalar NovAtel heading angle in degrees
%
% Output:
%   globalPoints: [N x 3] yaw-projected global points
    azimuthRad = deg2rad(double(azimuthDeg));
    localX = localPoints(:, 1);
    localY = localPoints(:, 2);
    localZ = localPoints(:, 3);
    globalPoints = zeros(size(localPoints));
    globalPoints(:, 1) = translation(1) + sin(azimuthRad) .* localX - cos(azimuthRad) .* localY;
    globalPoints(:, 2) = translation(2) + cos(azimuthRad) .* localX + sin(azimuthRad) .* localY;
    globalPoints(:, 3) = translation(3) + localZ;
end
