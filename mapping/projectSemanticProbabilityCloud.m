function projected = projectSemanticProbabilityCloud(cloud, dimension)
% projectSemanticProbabilityCloud: Select XYZ or its exact XY marginal.
% Mixture mass is unchanged. Height is a normalized conditional density,
% not a second peak amplitude or an extra length-dependent mixture weight.
    assert(isscalar(dimension) && ismember(dimension,[2 3]), 'Expected dimension 2 or 3.');
    source = validateSemanticProbabilityCloud(cloud);
    if dimension == 3 && size(source.mean,2)==2
        assert(isfield(source,'heightAvailable') && all(source.heightAvailable), ...
            'VehicleLocalization:HeightUnavailable','This cloud has no complete height model.');
        means = source.meanXYZ;
        covariance = source.covarianceXYZ;
    else
        means = source.mean(:,1:dimension);
        covariance = source.covariance(1:dimension,1:dimension,:);
    end
    components = struct('semanticName',source.semanticName,'mean',means, ...
        'covariance',covariance,'mixtureWeight',source.mixtureWeight,'numComponents',source.numComponents);
    for name=["semanticProbability","occupancyProbability","supportAmplitude"]
        if isfield(source,name), components.(name)=source.(name); end
    end
    projected = struct('components',components,'dimension',dimension);
    if isfield(cloud,'frameCalibration'), projected.frameCalibration=cloud.frameCalibration; end
    if isfield(cloud,'coordinateFrame'), projected.coordinateFrame=cloud.coordinateFrame; end
    if dimension==3 && isfield(cloud,'spatialCoordinateFrame')
        projected.coordinateFrame=cloud.spatialCoordinateFrame;
    end
end
