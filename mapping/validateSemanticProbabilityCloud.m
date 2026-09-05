function components = validateSemanticProbabilityCloud(cloud)
% validateSemanticProbabilityCloud: Validate planar or spatial Gaussian clouds.
    assert(isstruct(cloud) && isfield(cloud, 'components'), 'A probability cloud needs components.');
    components = cloud.components;
    required = ["semanticName", "mean", "covariance", "mixtureWeight", "numComponents"];
    assert(all(isfield(components, required)), 'Incomplete Gaussian component schema.');
    n = double(components.numComponents);
    dimension = size(components.mean, 2);
    assert(isscalar(n) && isfinite(n) && n >= 0 && n == floor(n), 'Invalid component count.');
    assert(ismember(dimension, [2 3]) && isequal(size(components.mean), [n dimension]) && ...
        size(components.covariance,1) == dimension && size(components.covariance,2) == dimension && ...
        size(components.covariance,3) == n && numel(components.covariance) == dimension^2*n && ...
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
    if isfield(components, 'heightAvailable')
        assert(islogical(components.heightAvailable) && numel(components.heightAvailable)==n && ...
            isequal(size(components.meanXYZ),[n 3]) && ...
            size(components.covarianceXYZ,1)==3 && size(components.covarianceXYZ,2)==3 && ...
            size(components.covarianceXYZ,3)==n && numel(components.covarianceXYZ)==9*n, 'Invalid retained XYZ arrays.');
        for k = find(components.heightAvailable(:)).'
            spatial = components.covarianceXYZ(:,:,k);
            assert(isreal(spatial) && all(isfinite(spatial),'all') && ...
                isreal(components.meanXYZ(k,:)) && all(isfinite(components.meanXYZ(k,:))), 'Invalid retained XYZ geometry.');
            assert(norm(spatial-spatial.','fro')<1e-9*max(1,norm(spatial,'fro')), 'Asymmetric retained XYZ covariance.');
            [~,flag] = chol(spatial);
            assert(flag==0,'VehicleLocalization:InvalidCovariance','Retained XYZ covariance must be positive definite.');
            assert(norm(components.meanXYZ(k,1:2)-components.mean(k,1:2))<1e-8 && ...
                norm(spatial(1:2,1:2)-components.covariance(1:2,1:2,k),'fro')<1e-9, 'XYZ and XY marginal disagree.');
        end
    end
end
