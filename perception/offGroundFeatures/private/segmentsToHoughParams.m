function [rho, theta, valid] = segmentsToHoughParams(lines)
% segmentsToHoughParams: Convert 2D line segments to an infinite-line
% parameterization compatible with the Hough line equation:
%   x*cos(theta) + y*sin(theta) = rho.
%
% Input:
%   lines: [N x 4] double matrix [x1 y1 x2 y2] in meters
%
% Output:
%   rho: [N x 1] double rho values in meters
%   theta: [N x 1] double normal angles in degrees in [0, 180)
%   valid: [N x 1] logical valid flags (finite, non-degenerate segments)
    lines = double(lines);
    if isempty(lines)
        rho = zeros(0, 1);
        theta = zeros(0, 1);
        valid = false(0, 1);
        return;
    end

    dx = lines(:, 3) - lines(:, 1);
    dy = lines(:, 4) - lines(:, 2);
    segLen = hypot(dx, dy);
    valid = isfinite(segLen) & (segLen > eps);

    xMid = 0.5 * (lines(:, 1) + lines(:, 3));
    yMid = 0.5 * (lines(:, 2) + lines(:, 4));

    theta = atan2d(dy, dx) + 90;
    theta = mod(theta, 180);
    theta(~valid) = NaN;

    rho = xMid .* cosd(theta) + yMid .* sind(theta);
    rho(~valid) = NaN;
    valid = valid & isfinite(rho) & isfinite(theta);
end
