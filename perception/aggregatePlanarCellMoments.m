function moments = aggregatePlanarCellMoments(points, cellIndices, numCells, projectionRotation, projectionTranslation, useNative)
% aggregatePlanarCellMoments: Compute XY and retained height moments before semantics.
% Cell indices use the caller's linear layout. Empirical within-cell scatter
% is preserved; downstream regularization supplies a finite sensor floor.
% Optional proper 3D rotation applies known tilt before XY projection; this
% is linear sufficient-statistic accumulation, not point feature selection.
% XYZ input adds meanZ and heightCovariance [xz yz zz]; the existing mean
% and covariance fields remain the unchanged XY marginal. XY input has no Z.
    if nargin >= 4
        rotation = double(projectionRotation);
        assert(isequal(size(rotation),[3 3]) && all(isfinite(rotation),'all') && ...
            norm(rotation*rotation.'-eye(3),'fro')<1e-8 && det(rotation)>0, ...
            'projectionRotation must be a proper rotation.');
        points = double(points)*rotation.';
    end
    if nargin >= 5
        translation=double(projectionTranslation(:).');
        assert(numel(translation)==3 && all(isfinite(translation)), 'Invalid projection translation.');
        if any(translation~=0), points=points+translation; end
    end
    if nargin>=6 && useNative
        [counts,mu,scatter]=perceptionKernelsMex('cellMoments',double(points),double(cellIndices(:)),double(numCells));
        moments=struct("count",counts,"mean",mu(:,1:2),"covariance",scatter(:,1:3));
        if size(points,2)==3
            moments.meanZ=mu(:,3);
            moments.heightCovariance=scatter(:,4:6);
        end
        return;
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
    if size(points, 2) == 3
        % Preserve height and its horizontal coupling without changing XY.
        meanZ = zeros(numCells, 1);
        heightCovariance = zeros(numCells, 3);
        if ~isempty(points)
            meanZ = accumarray(cellIndices, double(points(:, 3)), ...
                [numCells, 1], @sum, 0)./max(counts, 1);
            residualZ = double(points(:, 3))-meanZ(cellIndices);
            productsZ = [residual(:, 1).*residualZ, residual(:, 2).*residualZ, residualZ.^2];
            for k = 1:3
                heightCovariance(:, k) = accumarray(cellIndices, productsZ(:, k), ...
                    [numCells, 1], @sum, 0)./max(counts, 1);
            end
        end
        moments.meanZ = meanZ;
        moments.heightCovariance = heightCovariance; % xz, yz, zz
    end
end
