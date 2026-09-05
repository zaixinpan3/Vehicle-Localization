function components = validateSemanticProbabilityCloud(cloud)
% validateSemanticProbabilityCloud: Validate the common planar D2D schema.
    assert(isstruct(cloud) && isfield(cloud, 'components'), 'A probability cloud needs components.');
    components = cloud.components;
    required = ["semanticName", "mean", "covariance", "mixtureWeight", "numComponents"];
    assert(all(isfield(components, required)), 'Incomplete Gaussian component schema.');
    n = double(components.numComponents);
    assert(isscalar(n) && isfinite(n) && n >= 0 && n == floor(n), 'Invalid component count.');
    assert(isequal(size(components.mean), [n 2]) && ...
        size(components.covariance,1) == 2 && size(components.covariance,2) == 2 && ...
        size(components.covariance,3) == n && numel(components.covariance) == 4*n && ...
        numel(components.semanticName) == n && numel(components.mixtureWeight) == n, 'Misaligned component arrays.');
    assert(isreal(components.mean) && isreal(components.covariance) && ...
        all(isfinite(components.mean), 'all') && all(isfinite(components.covariance), 'all'), 'Nonfinite or complex Gaussian geometry.');
    components.semanticName = string(components.semanticName(:));
    assert(all(strlength(components.semanticName)>0), 'Empty semantic labels.');
    components.mixtureWeight = double(components.mixtureWeight(:));
    assert(isreal(components.mixtureWeight) && all(isfinite(components.mixtureWeight) & components.mixtureWeight >= 0), 'Invalid mixture weights.');
    for k = 1:n
        covariance = components.covariance(:,:,k);
        assert(norm(covariance-covariance.', 'fro') <= 1e-9*max(norm(covariance,'fro'),1), 'Asymmetric covariance.');
        [~, flag] = chol(covariance);
        assert(flag == 0, 'VehicleLocalization:InvalidCovariance', 'Covariances must be positive definite.');
    end
    if n > 0
        assert(sum(components.mixtureWeight)>0, 'A nonempty cloud must have positive mass.');
    end
end
