function [t, n] = pcaDirections(points, pcaEpsilon)
% pcaDirections: Estimate patch-level dominant and minor PCA directions
% from the regularized two-dimensional covariance matrix. Degenerate patches
% without a well-defined dominant direction are invalid under fail-fast
% construction.
%
% Input:
%   points: [N x 2] patch point coordinates
%   pcaEpsilon: scalar covariance diagonal regularization
%
% Output:
%   t: [2 x 1] dominant PCA direction
%   n: [2 x 1] minor PCA direction
    if size(points, 1) <= 1
        error("buildTemporalStabilityGmmMap:DegeneratePatchPca", ...
            "Patch PCA requires at least two points with a well-defined dominant direction.");
    end

    centeredPoints = points - mean(points, 1);
    covarianceMatrix = (centeredPoints.' * centeredPoints) ./ size(points, 1) + pcaEpsilon .* eye(2);
    [t, n] = covarianceDirections(covarianceMatrix);
end
