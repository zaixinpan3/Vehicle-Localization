function [translationWeight, headingWeight, diagnostics] = computeLidarInformationWeights(informationMatrix, cfg)
% computeLidarInformationWeights Admit full-pose information before injection.
% A qualified pose uses unit XY base gain. Full XY/yaw cross terms enter the
% information test; marginalized yaw information determines heading weight.
% Numerical full rank does not establish a calibrated matching-error bound.
    arguments
        informationMatrix double
        cfg (1, 1) struct
    end
    translationWeight = zeros(2);
    headingWeight = 0;
    diagnostics = struct('hadInformation', false, 'qualified', false, ...
        'reason', "missingInformation", 'minimumNormalizedEigenvalue', 0, ...
        'informationMargin', -Inf, 'headingInformation', 0, ...
        'translationInformationEigenvalues', zeros(2,1));
    if isempty(informationMatrix), return; end
    assert(isequal(size(informationMatrix), [3,3]), 'Information must be 3-by-3.');
    if any(~isfinite(informationMatrix), 'all'), return; end
    diagnostics.hadInformation = true;
    I = (informationMatrix + informationMatrix.') / 2;
    scales = double(cfg.lidar.poseScales(:));
    assert(numel(scales)==3 && all(isfinite(scales) & scales>0));
    D = diag(scales);
    lower = double(cfg.lidar.informationLowerBound);
    assert(isequal(size(lower),[3,3]) && all(isfinite(lower),'all') ...
        && norm(lower-lower.','fro')<1e-12 && min(eig(lower))>=-1e-12);
    diagnostics.minimumNormalizedEigenvalue = min(eig(D*I*D));
    diagnostics.informationMargin = min(eig(D*(I-lower)*D));
    if diagnostics.minimumNormalizedEigenvalue <= cfg.lidar.minimumNormalizedEigenvalue ...
            || diagnostics.informationMargin < -1e-12
        diagnostics.reason = "insufficientFullPoseInformation";
        return;
    end
    yawInformation = I(3,3)-I(3,1:2)*(I(1:2,1:2)\I(1:2,3));
    xyInformation = I(1:2,1:2)-I(1:2,3)*I(3,1:2)/I(3,3);
    diagnostics.translationInformationEigenvalues = sort(eig(xyInformation),'descend');
    diagnostics.headingInformation = yawInformation;
    diagnostics.qualified = true;
    diagnostics.reason = "qualified";
    translationWeight = eye(2);
    headingWeight = max(cfg.lidar.minimumHeadingWeight, ...
        yawInformation/(yawInformation+cfg.lidar.headingInformationScale));
end
