classdef mappingSupport
% mappingSupport: Shared map diagnostics and numerical/schema validation.
% Static methods keep common operations in one file. No instance is needed.
% Example: components = mappingSupport.validateSemanticProbabilityCloud(mapCloud).

    methods (Static)
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
