function L = scheduleLateralObserverGain(design, longitudinalSpeed)
% scheduleLateralObserverGain: Evaluate the observer gain of one operating
% point as the barycentric blend of the three vertex gains. The gain is
% affine in the barycentric coordinates alpha of rho = [Vx; 1/Vx] on the
% scheduling triangle, so L(rho) = sum_i alpha_i L_i with the same alpha
% that reproduces rho. A speed outside the designed range is clamped to the
% nearer end, where alpha is a unit vector and the gain is that vertex gain
% itself.
%
% Input:
%   design: struct with polytope and vertexGains from
%       designLateralObserverGains
%   longitudinalSpeed: scalar Vx in meters per second
%
% Output:
%   L: [2 x 2] observer gain at the operating point
    longitudinalSpeed = double(longitudinalSpeed);
    assert(isscalar(longitudinalSpeed) && isfinite(longitudinalSpeed), ...
        "The scheduling speed must be a finite scalar.");
    speedRange = design.polytope.speedRange;
    clampedSpeed = min(max(longitudinalSpeed, speedRange(1)), speedRange(2));
    alpha = schedulingCoordinates(design.polytope, clampedSpeed);

    L = (alpha(1) .* design.vertexGains(:, :, 1)) + (alpha(2) .* design.vertexGains(:, :, 2)) + ...
        (alpha(3) .* design.vertexGains(:, :, 3));
end
