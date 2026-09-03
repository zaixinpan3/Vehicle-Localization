function polytope = buildSchedulingPolytope(speedRange)
% buildSchedulingPolytope: Build the triangle that covers the scheduling
% curve rho(Vx) = [Vx; 1/Vx] over Vx in [Vmin, Vmax]. Because 1/Vx is
% convex in Vx, the curve lies below the chord joining its endpoints, so a
% triangle whose first two vertices are those endpoints and whose third
% vertex lies below the curve contains the whole arc:
%
%   rho1 = [Vmin; 1/Vmin]
%   rho2 = [Vmax; 1/Vmax]
%   rho3 = [2 Vmin Vmax/(Vmin+Vmax); 2/(Vmin+Vmax)]
%
% The third vertex takes the harmonic mean speed with the reciprocal of the
% arithmetic mean, which is strictly below the curve because 4 Vmin Vmax
% < (Vmin+Vmax)^2. The barycentric coordinates of any rho then follow from
% the vertex matrix S, and they are nonnegative exactly because the arc is
% inside the triangle.
%
% Input:
%   speedRange: [1 x 2] positive longitudinal speed bounds [Vmin, Vmax]
%
% Output:
%   polytope: struct with vertices [2 x 3], S [3 x 3], speedRange, and the
%       vertex speeds used for reporting
    speedRange = double(speedRange(:).');
    assert(numel(speedRange) == 2 && all(isfinite(speedRange)) && all(speedRange > 0) && ...
        speedRange(1) < speedRange(2), ...
        "speedRange must be [Vmin, Vmax] with 0 < Vmin < Vmax.");

    minimumSpeed = speedRange(1);
    maximumSpeed = speedRange(2);
    harmonicMeanSpeed = (2.0 .* minimumSpeed .* maximumSpeed) ./ (minimumSpeed + maximumSpeed);

    vertices = [minimumSpeed, maximumSpeed, harmonicMeanSpeed; ...
        1.0 ./ minimumSpeed, 1.0 ./ maximumSpeed, 2.0 ./ (minimumSpeed + maximumSpeed)];
    S = [vertices; ones(1, 3)];
    assert(abs(det(S)) > eps(max(abs(S(:)))), "The scheduling polytope is degenerate for the requested speed range.");

    polytope = struct();
    polytope.vertices = vertices;
    polytope.S = S;
    polytope.speedRange = speedRange;
    polytope.vertexSpeeds = vertices(1, :);
end
