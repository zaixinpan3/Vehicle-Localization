function [similarity, details] = scoreSemanticProbabilityCloudAlignment(fixedCloud, movingCloud, poseXYTheta, cfg)
% scoreSemanticProbabilityCloudAlignment: Mean normalized per-class D2D overlap.
% Components are planar Gaussians in meters; pose is moving-to-fixed [x,y,yaw]
% with yaw in radians. Both means and covariances transform. Empty clouds
% return zero similarity. gradient is with respect to the three pose entries.
    if nargin<4, cfg=distributionRegistrationConfig(); end
    [fixed,moving,heightDetails] = prepareSemanticRegistration(fixedCloud,movingCloud,cfg);
    [fixed,moving] = balanceSemanticDistributions(fixed,moving);
    pose = double(poseXYTheta(:).');
    assert(numel(pose)==3 && all(isfinite(pose)), 'Expected finite [x y yaw].');
    [crossEnergy, gradient] = semanticGaussianOverlap(fixed, moving, pose);
    fixedSelf = semanticGaussianOverlap(fixed, fixed, [0 0 0]);
    movingSelf = semanticGaussianOverlap(moving, moving, [0 0 0]);
    normalization = sqrt(max(fixedSelf*movingSelf,0));
    similarity = 0;
    if normalization > 0
        similarity = min(max(crossEnergy/normalization,0),1);
        gradient = gradient/normalization;
    else
        gradient(:) = 0;
    end
    details = struct('crossEnergy',crossEnergy,'fixedSelfEnergy',fixedSelf, ...
        'movingSelfEnergy',movingSelf,'squaredL2Distance',max(fixedSelf+movingSelf-2*crossEnergy,0), ...
        'poseXYTheta',pose,'gradient',gradient);
    details.height=heightDetails;
end
