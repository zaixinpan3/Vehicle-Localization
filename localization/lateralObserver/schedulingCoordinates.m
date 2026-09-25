function alpha = schedulingCoordinates(polytope, longitudinalSpeed)
% schedulingCoordinates: Express one scheduling point in the barycentric
% coordinates of the polytope. Solving S alpha = [rho; 1] gives coordinates
% that sum to one and reproduce rho, so any matrix affine in rho, including
% the observer gain, equals the same convex combination of its vertex
% values.
%
% Input:
%   polytope: struct from buildSchedulingPolytope
%   longitudinalSpeed: scalar Vx in meters per second, strictly positive
%
% Output:
%   alpha: [3 x 1] barycentric coordinates of rho(Vx) = [Vx; 1/Vx]
    longitudinalSpeed = double(longitudinalSpeed);
    assert(isscalar(longitudinalSpeed) && isfinite(longitudinalSpeed) && longitudinalSpeed > 0, ...
        "longitudinalSpeed must be a positive finite scalar.");

    alpha = polytope.S \ [longitudinalSpeed; 1.0 ./ longitudinalSpeed; 1.0];
end
