function cloud = temporalMapToProbabilityCloud(map, batchIndex)
% temporalMapToProbabilityCloud: Export one temporal map window for D2D.
% The normalized SUM of its peak-normalized Gaussian supports is represented
% exactly. It is a registration surrogate for the original noisy-OR/max query,
% not an algebraically equivalent query. Never concatenate overlapping windows.
% Mass = priorScore * supportAmplitude * 2*pi*sqrt(det(covariance)).
% Optional XYZ fields lift the XY distribution with normalized p(z|XY).
% They retain the SAME integrated XY mass; do not multiply by vertical extent.
% Old XY maps expose heightAvailable=false and never invent height.
    if nargin < 2
        batchIndex = 1;
    end
    if isfield(map,'batchMaps')
        assert(isscalar(batchIndex) && batchIndex==floor(batchIndex) && batchIndex>=1 && ...
            batchIndex<=numel(map.batchMaps), 'Invalid map window.');
        map = map.batchMaps(batchIndex).gmmMap;
    end
    names = strings(0,1); means=zeros(0,2); covariance=zeros(2,2,0); weights=zeros(0,1);
    amplitudes=zeros(0,1);
    meansXYZ = zeros(0,3); covarianceXYZ = zeros(3,3,0); heightAvailable = false(0,1);
    if ~isempty(map)
        for layer = map.layers(:).'
            n = size(layer.componentMeans,1);
            for k = 1:n
                amplitude = double(layer.priorScore)*double(layer.componentSupportAmplitudes(k));
                if amplitude <= 0
                    continue;
                end
                cov = double(layer.componentCovariances(:,:,k));
                [~,flag] = chol(cov);
                assert(flag==0 && all(isfinite(cov),'all'), 'Invalid map covariance.');
                names(end+1,1)=string(layer.classLabel); %#ok<AGROW>
                means(end+1,:)=double(layer.componentMeans(k,:)); %#ok<AGROW>
                covariance(:,:,end+1)=cov; %#ok<AGROW>
                weights(end+1,1)=amplitude*2*pi*sqrt(det(cov)); %#ok<AGROW>
                amplitudes(end+1,1)=amplitude; %#ok<AGROW>
                hasHeight = isfield(layer,'componentMeansXYZ') && ...
                    size(layer.componentMeansXYZ,1)==n && ~isempty(layer.componentMeansXYZ);
                heightAvailable(end+1,1)=hasHeight; %#ok<AGROW>
                if hasHeight
                    meansXYZ(end+1,:)=layer.componentMeansXYZ(k,:); %#ok<AGROW>
                    covarianceXYZ(:,:,end+1)=layer.componentCovariancesXYZ(:,:,k); %#ok<AGROW>
                else
                    meansXYZ(end+1,:)=[layer.componentMeans(k,:),NaN]; %#ok<AGROW>
                    covarianceXYZ(:,:,end+1)=nan(3); %#ok<AGROW>
                end
            end
        end
    end
    components=struct('semanticName',names,'mean',means,'covariance',covariance, ...
        'mixtureWeight',weights/max(sum(weights),eps),'numComponents',numel(weights), ...
        'supportAmplitude',amplitudes);
    components.meanXYZ=meansXYZ;
    components.covarianceXYZ=covarianceXYZ;
    components.heightAvailable=heightAvailable;
    cloud=struct('mapType',"semanticNDTProbabilityCloud2D",'dimension',2, ...
        'coordinateFrame',"mapXY",'components',components,'sourceBatchIndex',batchIndex, ...
        'weightSemantics',"normalizedIntegratedTemporalGaussianSupport", ...
        'queryRelationship',"sumSurrogateForNoisyOrAndMax");
    cloud.spatialDimension=2+any(heightAvailable);
    cloud.spatialCoordinateFrame="mapXYZ";
    cloud.heightModel="unavailable";
    if any(heightAvailable), cloud.heightModel="conditionalGaussianGivenXY"; end
    validateSemanticProbabilityCloud(cloud);
end
