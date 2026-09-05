function cloud = temporalMapToProbabilityCloud(map, batchIndex)
% temporalMapToProbabilityCloud: Normalize the same Gaussian field as queries.
% New clouds retain global totalMass, classTotalMass, per-class mixture weights,
% clutter intensities, coverage, and covariance semantics. For class c,
% Lambda_c = classTotalMass(c) * sum_k classMixtureWeight(k)*phi_k.
% Scheduling windows refer to one canonical map. A supplied batchIndex validates
% that schedule entry but does not select another statistical map.
% Legacy artifacts use their explicitly labelled historical sum surrogate.
    if nargin<2, batchIndex=1; end
    calibration=[];
    if isstruct(map) && isfield(map,'frameCalibration'), calibration=map.frameCalibration; end
    if isstruct(map) && isfield(map,'canonicalMap')
        assert(isscalar(batchIndex) && batchIndex==floor(batchIndex) && batchIndex>=1 && ...
            batchIndex<=numel(map.batchMaps),'Invalid schedule window.');
        map=map.canonicalMap;
    end
    if ~(isstruct(map) && isfield(map,'schemaVersion') && map.schemaVersion==2)
        cloud=legacyExport(map,batchIndex); return;
    end
    mappingSupport.validateRepeatedObservationMap(map);
    names=strings(0,1); means=zeros(0,2); covariances=zeros(2,2,0); masses=zeros(0,1);
    classWeights=zeros(0,1); meansXYZ=zeros(0,3); covariancesXYZ=zeros(3,3,0);
    height=false(0,1); ids=strings(0,1); referenceMass=zeros(0,1); repeatability=zeros(0,1);
    classNames=map.classLabels(:); classMass=zeros(numel(classNames),1);
    clutter=classMass; importance=classMass; coverage=cell(numel(classNames),1);
    for j=1:numel(map.layers)
        layer=map.layers(j); keep=find(layer.componentMasses>0); n=numel(keep);
        classMass(j)=layer.totalMass; clutter(j)=layer.clutterIntensity;
        importance(j)=layer.classImportance; coverage{j}=layer.coverageRegions;
        names=[names;repmat(layer.classLabel,n,1)]; %#ok<AGROW>
        means=[means;layer.componentMeans(keep,:)]; %#ok<AGROW>
        covariances=cat(3,covariances,layer.componentCovariances(:,:,keep));
        masses=[masses;layer.componentMasses(keep)]; %#ok<AGROW>
        classWeights=[classWeights;layer.componentMixtureWeights(keep)]; %#ok<AGROW>
        meansXYZ=[meansXYZ;layer.componentMeansXYZ(keep,:)]; %#ok<AGROW>
        covariancesXYZ=cat(3,covariancesXYZ,layer.componentCovariancesXYZ(:,:,keep));
        height=[height;layer.componentHeightAvailable(keep)]; %#ok<AGROW>
        ids=[ids;layer.classLabel+":"+layer.componentIds(keep)]; %#ok<AGROW>
        referenceMass=[referenceMass;layer.componentReferenceMasses(keep)]; %#ok<AGROW>
        repeatability=[repeatability;layer.componentRepeatability(keep)]; %#ok<AGROW>
    end
    total=sum(masses); weights=zeros(size(masses));
    if total>0, weights=masses/total; end
    components=struct('semanticName',names,'mean',means,'covariance',covariances, ...
        'mixtureWeight',weights,'classMixtureWeight',classWeights,'mass',masses, ...
        'numComponents',numel(masses),'meanXYZ',meansXYZ,'covarianceXYZ',covariancesXYZ, ...
        'heightAvailable',height,'componentId',ids,'referenceMass',referenceMass,'repeatability',repeatability);
    cloud=struct('mapType',"semanticNDTProbabilityCloud2D",'schemaVersion',2, ...
        'dimension',2,'coordinateFrame',"mapXY",'components',components,'totalMass',total, ...
        'classNames',classNames,'classTotalMass',classMass,'clutterIntensity',clutter, ...
        'classImportance',importance,'coverageRegions',{coverage},'sourceBatchIndex',0, ...
        'weightSemantics',"normalizedReferenceRepeatabilityMass", ...
        'queryRelationship',"exactIntensityNormalization",'spatialDimension',2+any(height), ...
        'heightModel',"conditionalGaussianGivenXY",'spatialCoordinateFrame',"mapXYZ", ...
        'covarianceSemantics',"withinBlockPlusStableOffsetPlusReferenceMeanUncertainty");
    if isfield(map,'frameCalibration'), calibration=map.frameCalibration; end
    if ~isempty(calibration), cloud.frameCalibration=validateLidarFrameCalibration(calibration); end
    mappingSupport.validateSemanticProbabilityCloud(cloud);
end

function cloud = legacyExport(map, batchIndex)
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
    calibration=[];
    if isfield(map,'frameCalibration'), calibration=map.frameCalibration; end
    if isfield(map,'batchMaps')
        assert(isscalar(batchIndex) && batchIndex==floor(batchIndex) && batchIndex>=1 && ...
            batchIndex<=numel(map.batchMaps), 'Invalid map window.');
        map = map.batchMaps(batchIndex).gmmMap;
    end
    if isfield(map,'frameCalibration'), calibration=map.frameCalibration; end
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
    if ~isempty(calibration), cloud.frameCalibration=validateLidarFrameCalibration(calibration); end
    if any(heightAvailable), cloud.heightModel="conditionalGaussianGivenXY"; end
    mappingSupport.validateSemanticProbabilityCloud(cloud);
end
