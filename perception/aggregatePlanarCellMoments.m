function moments = aggregatePlanarCellMoments(points, cellIndices, numCells, projectionRotation)
% aggregatePlanarCellMoments: Compute sufficient statistics before semantics.
% Cell indices use the caller's linear layout. Empirical within-cell scatter
% is preserved; downstream regularization supplies a finite sensor floor.
% Optional proper 3D rotation applies known tilt before XY projection; this
% is linear sufficient-statistic accumulation, not point feature selection.
    if nargin >= 4
        rotation = double(projectionRotation);
        assert(isequal(size(rotation),[3 3]) && all(isfinite(rotation),'all') && ...
            norm(rotation*rotation.'-eye(3),'fro')<1e-8 && det(rotation)>0, ...
            'projectionRotation must be a proper rotation.');
        points = double(points)*rotation.';
    end
    counts = zeros(numCells, 1);
    meanXY = zeros(numCells, 2);
    covariance = zeros(numCells, 3);
    if ~isempty(points)
        cellIndices = double(cellIndices(:));
        counts = accumarray(cellIndices, 1, [numCells, 1], @sum, 0);
        for k = 1:2
            meanXY(:, k) = accumarray(cellIndices, double(points(:, k)), ...
                [numCells, 1], @sum, 0) ./ max(counts, 1);
        end
        residual = double(points(:, 1:2)) - meanXY(cellIndices, :);
        products = [residual(:, 1).^2, residual(:, 1).*residual(:, 2), residual(:, 2).^2];
        for k = 1:3
            covariance(:, k) = accumarray(cellIndices, products(:, k), ...
                [numCells, 1], @sum, 0) ./ max(counts, 1);
        end
    end
    moments = struct("count", counts, "mean", meanXY, "covariance", covariance);
end
