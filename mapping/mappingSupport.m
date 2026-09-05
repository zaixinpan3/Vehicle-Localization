classdef mappingSupport
% mappingSupport: Shared map diagnostics and numerical/schema validation.
% Static methods keep common operations in one file. No instance is needed.
% Example: components = mappingSupport.validateSemanticProbabilityCloud(mapCloud).

    methods (Static)
        function index=compileFieldIndex(layer)
        % compileFieldIndex: Rasterize error-bounded Gaussian extents.
        % Each of K positive components may omit at most epsilon/K intensity.
        % Large extents use a global candidate list to bound index memory.
            mass=layer.componentMasses(:); K=nnz(mass>0);
            tolerance=layer.params.queryIntensityTolerance;
            cellSize=layer.params.queryIndexCellSize;
            keys=zeros(0,2); values=zeros(0,1); globalCandidates=zeros(0,1);
            radiusSquared=inf(numel(mass),1);
            for k=find(mass>0).'
                C=layer.componentCovariances(:,:,k);
                peak=mass(k)/(2*pi*sqrt(det(C)));
                if tolerance==0
                    globalCandidates(end+1,1)=k; %#ok<AGROW>
                    continue;
                end
                radiusSquared(k)=max(0,2*log(peak*K/tolerance));
                extent=sqrt(radiusSquared(k)*diag(C)).';
                low=floor((layer.componentMeans(k,:)-extent)/cellSize);
                high=floor((layer.componentMeans(k,:)+extent)/cellSize);
                if prod(high-low+1)>10000
                    globalCandidates(end+1,1)=k; %#ok<AGROW>
                else
                    [a,b]=ndgrid(low(1):high(1),low(2):high(2));
                    keys=[keys;[a(:),b(:)]]; values=[values;repmat(k,numel(a),1)]; %#ok<AGROW>
                end
            end
            [cellKeys,~,group]=unique(keys,'rows');
            candidates=cell(size(cellKeys,1),1);
            if ~isempty(group), candidates=accumarray(group,values,[],@(v){unique(v)}); end
            index=struct('cellSize',cellSize,'cellKeys',cellKeys,'candidates',{candidates}, ...
                'globalCandidates',globalCandidates,'radiusSquared',radiusSquared, ...
                'intensityErrorBound',tolerance,'means',layer.componentMeans, ...
                'covariances',layer.componentCovariances,'masses',mass);
        end

        function validateRepeatedObservationMap(map)
        % validateRepeatedObservationMap: Check the field and ownership contract.
            assert(isfield(map,'schemaVersion') && map.schemaVersion==2,'Unsupported map schema.');
            for layer=map.layers(:).'
                K=size(layer.componentMeans,1); mass=layer.componentMasses(:);
                assert(numel(layer.components)==K,'Component arrays disagree.');
                assert(numel(mass)==K && all(isfinite(mass) & mass>=0), ...
                    'VehicleLocalization:InvalidMass','Invalid component mass.');
                assert(numel(unique(layer.componentIds))==K,'Duplicate component ownership.');
                assert(all(isfinite(layer.componentMeans),'all'),'Invalid means.');
                assert(isscalar(layer.clutterIntensity) && isfinite(layer.clutterIntensity) && layer.clutterIntensity>0, ...
                    'VehicleLocalization:InvalidClutter','Clutter intensity must be positive.');
                assert(size(layer.coverageRegions,2)==4 && all(isfinite(layer.coverageRegions),'all') && ...
                    all(layer.coverageRegions(:,3:4)>layer.coverageRegions(:,1:2),'all'),'Invalid coverage.');
                r=mappingSupport.clipUnit(layer.componentRepeatability);
                expected=layer.componentReferenceMasses.*r.*double(layer.componentPublished);
                assert(all(isfinite(layer.componentReferenceMasses) & layer.componentReferenceMasses>=0) && ...
                    norm(mass-expected,1)<1e-10*max(1,sum(mass)) && ...
                    abs(sum(mass)-layer.totalMass)<1e-10*max(1,sum(mass)), ...
                    'VehicleLocalization:MassMismatch','Field mass is not conserved.');
                assert(abs(sum(layer.componentReferenceMasses)+layer.backgroundReferenceMass-layer.referenceMass)<1e-10*max(1,layer.referenceMass), ...
                    'VehicleLocalization:ReferenceMassMismatch','Reference cells must conserve their area.');
                for k=1:K
                    mappingSupport.validateCovarianceMatrix(layer.componentCovariances(:,:,k),'predictive geometry');
                    c=layer.components(k);
                    assert(isequal(c.mean,layer.componentMeans(k,:)) && ...
                        isequal(c.covariance,layer.componentCovariances(:,:,k)) && ...
                        c.mass==mass(k) && c.referenceMass==layer.componentReferenceMasses(k) && ...
                        c.repeatability==r(k) && c.published==layer.componentPublished(k), ...
                        'VehicleLocalization:ComponentIndexMismatch','Component data and field arrays disagree.');
                    assert(~c.published || (c.status=="repeatable" && c.observedBlockCount>=layer.params.minObservedBlocks), ...
                        'Unconfirmed or variable components cannot be published.');
                    assert(norm(c.covariance-c.withinBlockCovariance-c.stableOffsetCovariance-c.referenceMeanCovariance,'fro')<1e-8, ...
                        'Predictive covariance semantics disagree.');
                    if c.heightAvailable
                        assert(norm(c.covarianceXYZ(1:2,1:2)-c.covariance,'fro')<1e-10 && ...
                            all(isfinite(c.meanXYZ)),'Invalid height marginal.');
                        [~,flag]=chol(c.covarianceXYZ); assert(flag==0,'Invalid joint covariance.');
                    end
                end
                for t=1:numel(layer.tiles)
                    tile=layer.tiles(t); ids=tile.representativeIndices;
                    assert(numel(unique(ids))==numel(ids),'A tile repeats an observation.');
                    assert(numel(unique(tile.observationBlockIds))==numel(tile.observationBlockIds),'Duplicate block IDs.');
                    owned=vertcat(layer.components.ownerTile)==tile.ownerTile;
                    owned=all(owned,2);
                    if any(owned)
                        cellMass=horzcat(layer.components(owned).referenceCellMass);
                        assert(all(abs(sum(cellMass,2)+tile.backgroundReferenceCellMass-layer.params.referenceResolution^2)<1e-10), ...
                            'Support-cell mass was duplicated.');
                    end
                end
            end
        end

        function tf = isLogEnabled(cfg, fieldName)
        % isLogEnabled: Read one logging switch without changing its default.
            if nargin < 2
                fieldName = "stepLogsEnabled";
            end
            tf = false;
            if isstruct(cfg) && isfield(cfg, fieldName) && isscalar(cfg.(fieldName))
                tf = logical(cfg.(fieldName));
            end
        end

        function logStep(cfg, logKey, message, varargin)
        % logStep: Print an offline mapping step when stepLogsEnabled is true.
            mappingSupport.printLog(mappingSupport.isLogEnabled(cfg), ...
                "map-test", logKey, message, varargin{:});
        end

        function printLog(enabled, prefix, logKey, message, varargin)
        % printLog: Emit one structured log while preserving the caller's prefix.
            if ~enabled
                return;
            end
            fprintf("[%s][%s] %s\n", char(string(prefix)), char(string(logKey)), ...
                sprintf(char(string(message)), varargin{:}));
        end

        function text = formatFrameIndexSet(frameIndices)
        % formatFrameIndexSet: Format a frame-index vector as a compact range
        % when contiguous and as an explicit list otherwise.
            frameIndices = double(frameIndices(:).');
            if isempty(frameIndices)
                text = "[]";
            elseif isscalar(frameIndices)
                text = string(frameIndices(1));
            elseif all(diff(frameIndices) == 1)
                text = sprintf("%d:%d", frameIndices(1), frameIndices(end));
            else
                text = "[" + strjoin(string(frameIndices), " ") + "]";
            end
        end

        function [minValue, medianValue, maxValue] = finiteSummary(values)
        % finiteSummary: Compute min, median, and max over finite numeric values
        % for compact map-builder diagnostic logging.
            values = double(values(:));
            values = values(isfinite(values));
            if isempty(values)
                minValue = NaN;
                medianValue = NaN;
                maxValue = NaN;
                return;
            end
            minValue = min(values);
            medianValue = median(values);
            maxValue = max(values);
        end

        function validateCovarianceMatrix(covariance, contextName, errorPrefix)
        % validateCovarianceMatrix: Validate that a covariance matrix is finite,
        % symmetric, and positive definite before it is used in EM or query
        % diagnostics. errorPrefix preserves builder/query error identifiers.
            if nargin < 3
                errorPrefix = "buildTemporalStabilityGmmMap";
            end
            if ~isnumeric(covariance) || ~isequal(size(covariance), [2 2]) || any(~isfinite(covariance), "all")
                error(errorPrefix + ":InvalidCovarianceMatrix", ...
                    "%s requires a finite [2 x 2] covariance matrix.", contextName);
            end
            if norm(covariance - covariance.', "fro") > 1.0e-10 .* max(1, norm(covariance, "fro"))
                error(errorPrefix + ":NonSymmetricCovariance", ...
                    "%s requires a symmetric covariance matrix.", contextName);
            end
            [~, cholFlag] = chol((covariance + covariance.') ./ 2);
            covarianceDeterminant = det(covariance);
            if cholFlag ~= 0 || ~isfinite(covarianceDeterminant) || covarianceDeterminant <= 0
                error(errorPrefix + ":NonPositiveDefiniteCovariance", ...
                    "%s requires a symmetric positive definite covariance matrix with a finite positive determinant.", contextName);
            end
        end

        function value = clipUnit(value, errorPrefix)
        % clipUnit: Validate numeric support values against the closed unit
        % interval, allowing only harmless round-off at the boundary.
            if nargin < 2
                errorPrefix = "buildTemporalStabilityGmmMap";
            end
            quantity = "Support and reliability quantities";
            if errorPrefix == "queryTemporalStabilityGmmMap"
                quantity = "Support values";
            end
            if any(~isfinite(value), "all")
                error(errorPrefix + ":InvalidUnitValue", ...
                    "%s must be finite.", quantity);
            end
            lowerMask = value < 0;
            upperMask = value > 1;
            if any(value(lowerMask) < -1.0e-12, "all") || any(value(upperMask) > 1 + 1.0e-12, "all")
                error(errorPrefix + ":UnitValueOutOfRange", ...
                    "%s must lie in [0, 1].", quantity);
            end
            value(lowerMask) = 0;
            value(upperMask) = 1;
        end

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
            if isfield(cloud,'queryRelationship') && cloud.queryRelationship=="exactIntensityNormalization"
                assert(isfinite(cloud.totalMass) && cloud.totalMass>=0 && ...
                    abs(sum(components.mass)-cloud.totalMass)<1e-10*max(1,cloud.totalMass) && ...
                    norm(components.mixtureWeight*cloud.totalMass-components.mass,1)<1e-10*max(1,cloud.totalMass), ...
                    'VehicleLocalization:CloudMassMismatch','Cloud weights must reconstruct field mass.');
                for j=1:numel(cloud.classNames)
                    keep=components.semanticName==cloud.classNames(j);
                    assert(abs(sum(components.mass(keep))-cloud.classTotalMass(j))<1e-10*max(1,cloud.classTotalMass(j)) && ...
                        norm(components.classMixtureWeight(keep)*cloud.classTotalMass(j)-components.mass(keep),1)<1e-10*max(1,cloud.classTotalMass(j)), ...
                        'VehicleLocalization:ClassMassMismatch','Class weights must reconstruct class intensity.');
                end
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
    end
end
