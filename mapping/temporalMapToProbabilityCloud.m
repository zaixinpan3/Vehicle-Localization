function cloud = temporalMapToProbabilityCloud(map, batchIndex)
% temporalMapToProbabilityCloud: Export one temporal map window for D2D.
% The normalized SUM of its peak-normalized Gaussian supports is represented
% exactly. It is a registration surrogate for the original noisy-OR/max query,
% not an algebraically equivalent query. Never concatenate overlapping windows.
% Mass = priorScore * supportAmplitude * 2*pi*sqrt(det(covariance)).
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
            end
        end
    end
    components=struct('semanticName',names,'mean',means,'covariance',covariance, ...
        'mixtureWeight',weights/max(sum(weights),eps),'numComponents',numel(weights), ...
        'supportAmplitude',amplitudes);
    cloud=struct('mapType',"semanticNDTProbabilityCloud2D",'dimension',2, ...
        'coordinateFrame',"mapXY",'components',components,'sourceBatchIndex',batchIndex, ...
        'weightSemantics',"normalizedIntegratedTemporalGaussianSupport", ...
        'queryRelationship',"sumSurrogateForNoisyOrAndMax");
    validateSemanticProbabilityCloud(cloud);
end
