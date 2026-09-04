function [translationWeight, headingWeight, diagnostics] = computeLidarInformationWeights(informationMatrix, cfg)
% computeLidarInformationWeights Convert a scan-matching Hessian to [0, I].
% Translation eigenvectors are preserved while each nonnegative eigenvalue
% lambda is mapped to lambda/(lambda+lambda0). The heading diagonal receives
% the same rational map and is floored by the configured positive weight.

    arguments
        informationMatrix double
        cfg (1, 1) struct
    end

    validateLidarConfiguration(cfg);
    if isempty(informationMatrix)
        translationWeight = double(cfg.lidar.missingInformationTranslationWeight) .* eye(2);
        headingWeight = double(cfg.lidar.missingInformationHeadingWeight);
        diagnostics = buildDiagnostics([0.0; 0.0], 0.0, false);
        return;
    end
    assert(isequal(size(informationMatrix), [3, 3]) && all(isfinite(informationMatrix(:))), ...
        "informationMatrix must be empty or a finite [3 x 3] matrix.");

    symmetricInformation = 0.5 .* (informationMatrix + informationMatrix.');
    translationInformation = symmetricInformation(1:2, 1:2);
    [directions, eigenvalueMatrix] = eig(translationInformation, "vector");
    translationEigenvalues = max(real(eigenvalueMatrix), 0.0);
    translationScale = double(cfg.lidar.translationInformationScale);
    mappedEigenvalues = translationEigenvalues ./ (translationEigenvalues + translationScale);
    translationWeight = directions * diag(mappedEigenvalues) * directions.';
    translationWeight = projectWeightMatrix(translationWeight);

    headingInformation = max(real(symmetricInformation(3, 3)), 0.0);
    headingScale = double(cfg.lidar.headingInformationScale);
    mappedHeading = headingInformation ./ (headingInformation + headingScale);
    headingWeight = min(1.0, max(double(cfg.lidar.minimumHeadingWeight), mappedHeading));
    diagnostics = buildDiagnostics(translationEigenvalues, headingInformation, true);
    diagnostics.translationMappedEigenvalues = sort(mappedEigenvalues(:), "descend");
    diagnostics.headingMappedWeight = headingWeight;
end

function weight = projectWeightMatrix(weight)
% projectWeightMatrix Remove roundoff outside the symmetric unit interval.
    weight = 0.5 .* (weight + weight.');
    [directions, eigenvalues] = eig(weight, "vector");
    eigenvalues = min(max(real(eigenvalues), 0.0), 1.0);
    weight = directions * diag(eigenvalues) * directions.';
    weight = 0.5 .* (weight + weight.');
end

function diagnostics = buildDiagnostics(translationEigenvalues, headingInformation, hadInformation)
% buildDiagnostics Assemble traceable weight-construction inputs.
    diagnostics = struct();
    diagnostics.hadInformation = hadInformation;
    diagnostics.translationInformationEigenvalues = sort(translationEigenvalues(:), "descend");
    diagnostics.headingInformation = headingInformation;
end

function validateLidarConfiguration(cfg)
% validateLidarConfiguration Validate the rational information maps.
    assert(isfield(cfg, "lidar") && isstruct(cfg.lidar), "cfg.lidar is required.");
    positiveFields = ["translationInformationScale", "headingInformationScale"];
    unitFields = ["minimumHeadingWeight", "missingInformationTranslationWeight", ...
        "missingInformationHeadingWeight"];
    for fieldName = positiveFields
        value = double(cfg.lidar.(fieldName));
        assert(isscalar(value) && isfinite(value) && value > 0.0, ...
            "cfg.lidar.%s must be positive and finite.", fieldName);
    end
    for fieldName = unitFields
        value = double(cfg.lidar.(fieldName));
        assert(isscalar(value) && isfinite(value) && value >= 0.0 && value <= 1.0, ...
            "cfg.lidar.%s must lie in [0, 1].", fieldName);
    end
    assert(cfg.lidar.minimumHeadingWeight > 0.0, ...
        "cfg.lidar.minimumHeadingWeight must be strictly positive.");
end
