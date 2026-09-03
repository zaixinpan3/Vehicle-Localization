function model = lateralBicycleModel(vehicle)
% lateralBicycleModel: Build the 2-DOF lateral bicycle model of the ego
% vehicle in the affine-in-scheduling form used by the LPV observer. With
% the state x = [vy; r], the input u = delta, and the output y = [ay; r],
% linear tire forces give
%
%   vydot = -(Cf+Cr)/(m Vx) vy + ((lr Cr - lf Cf)/(m Vx) - Vx) r + Cf/m delta
%   rdot  = (lr Cr - lf Cf)/(Iz Vx) vy - (lf^2 Cf + lr^2 Cr)/(Iz Vx) r + lf Cf/Iz delta
%   ay    = -(Cf+Cr)/(m Vx) vy + (lr Cr - lf Cf)/(m Vx) r + Cf/m delta
%
% Every speed-dependent entry is affine in the scheduling parameter
% rho = [Vx; 1/Vx], so A(rho) = A0 + rho(1) A1 + rho(2) A2 and
% C(rho) = C0 + rho(2) C2 with constant B and D. That affine structure is
% what makes the polytopic representation exact.
%
% Input:
%   vehicle: struct with mass, yawInertia, lf, lr, frontCorneringStiffness,
%       and rearCorneringStiffness
%
% Output:
%   model: struct with the affine basis matrices A0, A1, A2, C0, C2, the
%       constant B and D, and the source vehicle parameters
    requiredFields = ["mass", "yawInertia", "lf", "lr", "frontCorneringStiffness", "rearCorneringStiffness"];
    for fieldName = requiredFields
        assert(isfield(vehicle, fieldName) && isscalar(vehicle.(fieldName)) && ...
            isfinite(vehicle.(fieldName)) && vehicle.(fieldName) > 0, ...
            "vehicle.%s must be a positive finite scalar.", fieldName);
    end

    mass = double(vehicle.mass);
    yawInertia = double(vehicle.yawInertia);
    lf = double(vehicle.lf);
    lr = double(vehicle.lr);
    frontStiffness = double(vehicle.frontCorneringStiffness);
    rearStiffness = double(vehicle.rearCorneringStiffness);

    stiffnessSum = frontStiffness + rearStiffness;
    stiffnessMoment = (lr .* rearStiffness) - (lf .* frontStiffness);
    stiffnessSecondMoment = (lf.^2 .* frontStiffness) + (lr.^2 .* rearStiffness);

    model = struct();
    model.vehicle = vehicle;
    model.stateNames = ["lateralVelocity", "yawRate"];
    model.outputNames = ["lateralAcceleration", "yawRate"];

    % A(rho) = A0 + Vx A1 + (1/Vx) A2
    model.A0 = zeros(2, 2);
    model.A1 = [0.0, -1.0; 0.0, 0.0];
    model.A2 = [-stiffnessSum ./ mass, stiffnessMoment ./ mass; ...
        stiffnessMoment ./ yawInertia, -stiffnessSecondMoment ./ yawInertia];

    % C(rho) = C0 + (1/Vx) C2; the yaw rate is measured directly
    model.C0 = [0.0, 0.0; 0.0, 1.0];
    model.C2 = [-stiffnessSum ./ mass, stiffnessMoment ./ mass; 0.0, 0.0];

    model.B = [frontStiffness ./ mass; (lf .* frontStiffness) ./ yawInertia];
    model.D = [frontStiffness ./ mass; 0.0];
end
