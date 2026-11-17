function cfg=overlapGradientConfig()
% overlapGradientConfig Correspondence-free LiDAR channel from semanticGaussianOverlap.
% At the observer prediction, each feature channel scores its own map and
% source mixtures. Average the channel negative-log gradients and metrics
% at each bandwidth, then sum over bandwidths.
    cfg=struct();
    geometry=semanticGeometryConfig();
    cfg.localMapRadius=geometry.localMapRadius;       % m, map crop around the prediction
    % Horizontal smoothing bandwidths. 0 is the exact overlap; a wider scale is
    % the expected overlap under an isotropic prediction error of that size.
    % The dyadic ladder adds broader overlap terms up to a 2 m bandwidth;
    % this is not a guaranteed capture radius for a multimodal map. On the
    % recorded MnCAV drive [0 0.5 1], [0 0.5 1 2] and [0 0.5 1 2 4] agreed
    % within 1 mm, while omitting 0.5 m was worse.
    cfg.scaleLadder=[0 0.5 1 2];                      % m
    cfg.geometry=geometry;                          % calibration, projection, height and support axes
end
