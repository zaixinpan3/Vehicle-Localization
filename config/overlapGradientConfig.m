function cfg=overlapGradientConfig()
% overlapGradientConfig Correspondence-free LiDAR channel from semanticGaussianOverlap.
% The full observer evaluates -log of the class-balanced normalized semantic
% overlap at its own post-baseline prediction and injects the pose gradient.
% There is no pose optimization or correspondence selection. Optional frame
% gates act on the evaluated similarity and curvature, never on a solved pose.
% Curvature comes from central differences of the analytic gradient.
    cfg=struct();
    registration=distributionRegistrationConfig();
    cfg.localMapRadius=registration.localMapRadius;   % m, map crop around the prediction
    cfg.viewConditioning=true;                        % condition view-modeled map components at the prediction
    cfg.differenceSteps=[1e-4,1e-4,1e-5];             % m, m, rad
    % Horizontal Gaussian smoothing of both mixtures. Zero keeps the exact overlap
    % of scoreSemanticProbabilityCloudAlignment; a positive value widens the
    % gradient basin at the cost of curvature.
    cfg.kernelBandwidth=0;                            % m
    % Absolute eigenvalues give a saddle-free Newton step: always a descent step,
    % also where the overlap is locally concave between alignments. positivePart
    % withholds those directions and can stall at a constrained optimum.
    cfg.curvature="absolute";                         % absolute | positivePart
    % Registration's evidence factors, applied to the mixture masses before
    % class balancing: map view reliability and source temporal stability.
    cfg.evidenceWeights=true;
    % Frame gates on the evaluated similarity and curvature spectrum. Zero
    % disables a gate. Values below are engineering settings, not calibrated.
    cfg.minimumSimilarity=0;                          % balanced overlap, 0..1
    cfg.minimumCurvature=0;                           % largest |eigenvalue| of the curvature
    cfg.minimumCurvatureRatio=0;                      % smallest / largest |eigenvalue|
    cfg.registration=registration;                    % calibration, projection and height mode only
end
