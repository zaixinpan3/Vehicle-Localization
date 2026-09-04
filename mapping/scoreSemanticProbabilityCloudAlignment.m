function [similarity, details] = scoreSemanticProbabilityCloudAlignment(fixedCloud, movingCloud, poseXYTheta)
% scoreSemanticProbabilityCloudAlignment: Evaluate a semantic D2D-NDT
% alignment by the normalized L2 inner product of matching-class Gaussian
% mixtures. A value of one denotes identical probability fields and values
% near zero denote little same-class overlap.
%
% Input:
%   fixedCloud: semanticNDTProbabilityCloud2D reference cloud
%   movingCloud: semanticNDTProbabilityCloud2D cloud to transform
%   poseXYTheta: [tx ty theta] planar moving-to-fixed transform, meters and
%       radians
%
% Output:
%   similarity: scalar normalized same-semantic Gaussian overlap in [0, 1]
%   details: struct with cross/self energies and squared L2 distance
    fixedComponents = validateCloud(fixedCloud, "fixedCloud");
    movingComponents = validateCloud(movingCloud, "movingCloud");
    poseXYTheta = double(poseXYTheta(:).');
    assert(numel(poseXYTheta) == 3 && all(isfinite(poseXYTheta)), ...
        "poseXYTheta must contain finite [tx ty theta].");

    rotation = [cos(poseXYTheta(3)), -sin(poseXYTheta(3)); ...
        sin(poseXYTheta(3)), cos(poseXYTheta(3))];
    transformedMoving = movingComponents;
    transformedMoving.mean = movingComponents.mean * rotation.' + poseXYTheta(1:2);
    for componentIdx = 1:movingComponents.numComponents
        transformedMoving.covariance(:, :, componentIdx) = rotation * ...
            movingComponents.covariance(:, :, componentIdx) * rotation.';
    end

    crossEnergy = semanticMixtureInnerProduct(fixedComponents, transformedMoving);
    fixedSelfEnergy = semanticMixtureInnerProduct(fixedComponents, fixedComponents);
    movingSelfEnergy = semanticMixtureInnerProduct(transformedMoving, transformedMoving);
    normalization = sqrt(max(fixedSelfEnergy .* movingSelfEnergy, 0));
    if normalization > 0 && isfinite(normalization)
        similarity = min(max(crossEnergy ./ normalization, 0), 1);
    else
        similarity = 0;
    end
    squaredL2Distance = max( ...
        fixedSelfEnergy + movingSelfEnergy - (2 .* crossEnergy), 0);
    details = struct( ...
        "crossEnergy", double(crossEnergy), ...
        "fixedSelfEnergy", double(fixedSelfEnergy), ...
        "movingSelfEnergy", double(movingSelfEnergy), ...
        "squaredL2Distance", double(squaredL2Distance), ...
        "poseXYTheta", poseXYTheta);
end

function components = validateCloud(cloud, argumentName)
% validateCloud: Validate the compact component schema used by the scorer.
    assert(isstruct(cloud) && isfield(cloud, "components"), ...
        "%s must contain components.", argumentName);
    components = cloud.components;
    requiredFields = ["semanticName", "mean", "covariance", ...
        "mixtureWeight", "numComponents"];
    assert(isstruct(components) && all(isfield(components, requiredFields)), ...
        "%s.components has an invalid probability-cloud schema.", argumentName);
    numComponents = double(components.numComponents);
    assert(isscalar(numComponents) && numComponents >= 0 && ...
        size(components.mean, 1) == numComponents && ...
        size(components.mean, 2) == 2 && ...
        isequal(size(components.covariance), [2, 2, numComponents]) && ...
        numel(components.semanticName) == numComponents && ...
        numel(components.mixtureWeight) == numComponents, ...
        "%s.components arrays are not mutually aligned.", argumentName);
end

function value = semanticMixtureInnerProduct(first, second)
% semanticMixtureInnerProduct: Integrate the product of two multi-channel
% Gaussian mixtures, treating distinct semantics as orthogonal channels.
    value = 0;
    if first.numComponents < 1 || second.numComponents < 1
        return;
    end
    firstNames = string(first.semanticName(:));
    secondNames = string(second.semanticName(:));
    sharedNames = intersect(unique(firstNames, "stable"), ...
        unique(secondNames, "stable"), "stable");
    for semanticName = sharedNames.'
        firstIdx = find(firstNames == semanticName);
        secondIdx = find(secondNames == semanticName);
        value = value + classMixtureInnerProduct(first, second, firstIdx, secondIdx);
    end
end

function value = classMixtureInnerProduct(first, second, firstIdx, secondIdx)
% classMixtureInnerProduct: Sum closed-form pairwise Gaussian overlaps for
% one semantic channel.
    value = 0;
    for firstLocalIdx = 1:numel(firstIdx)
        firstComponentIdx = firstIdx(firstLocalIdx);
        firstMean = first.mean(firstComponentIdx, :);
        firstCovariance = first.covariance(:, :, firstComponentIdx);
        firstWeight = double(first.mixtureWeight(firstComponentIdx));
        for secondLocalIdx = 1:numel(secondIdx)
            secondComponentIdx = secondIdx(secondLocalIdx);
            covarianceSum = firstCovariance + ...
                second.covariance(:, :, secondComponentIdx);
            covarianceSum = (covarianceSum + covarianceSum.') ./ 2;
            determinant = det(covarianceSum);
            if ~isfinite(determinant) || determinant <= 0
                continue;
            end
            delta = (firstMean - second.mean(secondComponentIdx, :)).';
            exponent = -0.5 .* (delta.' * (covarianceSum \ delta));
            overlap = exp(exponent) ./ (2 .* pi .* sqrt(determinant));
            pairWeight = firstWeight .* ...
                double(second.mixtureWeight(secondComponentIdx));
            value = value + (pairWeight .* overlap);
        end
    end
end
