function [alpha, alphaRate] = schedulingCoordinates(polytope, longitudinalSpeed, longitudinalAcceleration)
% schedulingCoordinates: Express one scheduling point in the barycentric
% coordinates of the polytope, together with their time derivative. Solving
% S alpha = [rho; 1] gives coordinates that sum to one and reproduce rho,
% so any matrix affine in rho equals the same convex combination of its
% vertex values. Differentiating along the speed trajectory with
% rhodot = [ax; -ax/Vx^2] gives S alphaRate = [rhodot; 0], which is the
% Pdot term of the parameter-dependent Lyapunov function.
%
% Input:
%   polytope: struct from buildSchedulingPolytope
%   longitudinalSpeed: scalar Vx in meters per second, strictly positive
%   longitudinalAcceleration: scalar ax in meters per second squared
%
% Output:
%   alpha: [3 x 1] barycentric coordinates of rho(Vx)
%   alphaRate: [3 x 1] time derivative of alpha at the given acceleration
    longitudinalSpeed = double(longitudinalSpeed);
    longitudinalAcceleration = double(longitudinalAcceleration);
    assert(isscalar(longitudinalSpeed) && isfinite(longitudinalSpeed) && longitudinalSpeed > 0, ...
        "longitudinalSpeed must be a positive finite scalar.");
    assert(isscalar(longitudinalAcceleration) && isfinite(longitudinalAcceleration), ...
        "longitudinalAcceleration must be a finite scalar.");

    alpha = polytope.S \ [longitudinalSpeed; 1.0 ./ longitudinalSpeed; 1.0];
    alphaRate = polytope.S \ (longitudinalAcceleration .* [1.0; -1.0 ./ (longitudinalSpeed.^2); 0.0]);
end
