function [L, P] = scheduleLateralObserverGain(design, longitudinalSpeed, longitudinalAcceleration)
% scheduleLateralObserverGain: Look up the observer gain of the current
% operating point by bilinear interpolation over the design grid, clamping
% to the grid when the vehicle leaves the range the synthesis covered.
%
% The certificate of designLateralObserverGains holds at the grid points
% themselves. Interpolating between them keeps the gain continuous along a
% trajectory, which the certificate does not by itself guarantee; refine
% cfg.scheduling.speedGridCount when a denser guarantee is required.
%
% Input:
%   design: struct from designLateralObserverGains
%   longitudinalSpeed: scalar Vx in meters per second
%   longitudinalAcceleration: scalar ax in meters per second squared
%
% Output:
%   L: [2 x 2] interpolated observer gain
%   P: [2 x 2] interpolated Lyapunov matrix at the same operating point
    [speedLowerIdx, speedUpperIdx, speedWeight] = interpolationWeights(design.speedGrid, longitudinalSpeed);
    [accelLowerIdx, accelUpperIdx, accelWeight] = interpolationWeights(design.accelerationGrid, longitudinalAcceleration);

    L = bilinearBlend(design.gains, speedLowerIdx, speedUpperIdx, speedWeight, ...
        accelLowerIdx, accelUpperIdx, accelWeight);
    P = bilinearBlend(design.lyapunovMatrices, speedLowerIdx, speedUpperIdx, speedWeight, ...
        accelLowerIdx, accelUpperIdx, accelWeight);
end

function [lowerIdx, upperIdx, weight] = interpolationWeights(gridValues, queryValue)
% interpolationWeights: Locate one query value in a monotonically increasing
% grid, returning the bracketing indices and the upper-node weight. Queries
% outside the grid are clamped to its ends.
%
% Input:
%   gridValues: [1 x n] increasing grid values
%   queryValue: scalar query
%
% Output:
%   lowerIdx, upperIdx: bracketing grid indices
%   weight: weight of the upper node, in [0, 1]
    gridValues = double(gridValues(:).');
    queryValue = double(queryValue);
    assert(isscalar(queryValue) && isfinite(queryValue), "The scheduling query must be a finite scalar.");
    if isscalar(gridValues)
        lowerIdx = 1;
        upperIdx = 1;
        weight = 0.0;
        return;
    end
    clamped = min(max(queryValue, gridValues(1)), gridValues(end));
    upperIdx = find(gridValues >= clamped, 1, "first");
    if upperIdx == 1
        lowerIdx = 1;
        upperIdx = 2;
    else
        lowerIdx = upperIdx - 1;
    end
    span = gridValues(upperIdx) - gridValues(lowerIdx);
    weight = 0.0;
    if span > 0
        weight = (clamped - gridValues(lowerIdx)) ./ span;
    end
end

function blended = bilinearBlend(table, rowLowerIdx, rowUpperIdx, rowWeight, colLowerIdx, colUpperIdx, colWeight)
% bilinearBlend: Blend the four bracketing matrices of a
% [2 x 2 x rows x cols] lookup table.
%
% Input:
%   table: [2 x 2 x rows x cols] matrix lookup table
%   rowLowerIdx, rowUpperIdx, rowWeight: bracketing rows and upper weight
%   colLowerIdx, colUpperIdx, colWeight: bracketing columns and upper weight
%
% Output:
%   blended: [2 x 2] interpolated matrix
    lowerRow = ((1.0 - colWeight) .* table(:, :, rowLowerIdx, colLowerIdx)) + ...
        (colWeight .* table(:, :, rowLowerIdx, colUpperIdx));
    upperRow = ((1.0 - colWeight) .* table(:, :, rowUpperIdx, colLowerIdx)) + ...
        (colWeight .* table(:, :, rowUpperIdx, colUpperIdx));
    blended = ((1.0 - rowWeight) .* lowerRow) + (rowWeight .* upperRow);
end
