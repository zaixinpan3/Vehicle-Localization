function [meansXYZ, covariancesXYZ] = fitConditionalHeight(points, components, varianceFloor)
% fitConditionalHeight: Fit p(z|XY,k) after the XY temporal GMM is finalized.
% Use the same temporally resampled returns and final XY responsibilities.
% This conditional regression preserves the existing XY marginal exactly.
    normalized = components;
    mass = sum([normalized.mixtureWeight]);
    for k = 1:numel(normalized)
        normalized(k).mixtureWeight = normalized(k).mixtureWeight/mass;
    end
    responsibility = gaussianExpectation(points(:, 1:2), normalized);
    meansXYZ = zeros(numel(components), 3);
    covariancesXYZ = zeros(3, 3, numel(components));
    for k = 1:numel(components)
        weight = responsibility(:, k);
        assert(sum(weight) > 0, 'Height component has no sample support.');
        weight = weight/sum(weight);
        origin = components(k).mean;
        xy = points(:, 1:2)-origin;
        meanXY = weight.'*xy;
        zOrigin = points(1, 3);
        z = points(:, 3)-zOrigin;
        meanZ = weight.'*z;
        dx = xy-meanXY;
        dz = z-meanZ;
        scatter = dx.'*(weight.*dx);
        slope = pinv(scatter)*(dx.'*(weight.*dz));
        residual = dz-dx*slope;
        conditionalVariance = max(sum(weight.*residual.^2), varianceFloor);
        covarianceXY = components(k).covariance;
        cross = covarianceXY*slope;
        meansXYZ(k, :) = [origin, zOrigin+meanZ-meanXY*slope];
        covariancesXYZ(:, :, k) = [covarianceXY, cross; cross.', conditionalVariance+slope.'*cross];
    end
end
