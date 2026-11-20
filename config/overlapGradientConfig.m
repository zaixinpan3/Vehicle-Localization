function cfg=overlapGradientConfig()
% overlapGradientConfig Correspondence-free LiDAR channel from semanticGaussianOverlap.
% At the observer prediction, each shared feature channel evaluates its exact
% single-scale Cauchy-Schwarz divergence (negative log L2 overlap) and
% Gauss-Newton Fisher information, averaged across evaluable semantic classes.
    cfg=struct();
    geometry=semanticGeometryConfig();
    cfg.localMapRadius=geometry.localMapRadius;       % m, map crop around the prediction
    cfg.geometry=geometry;                          % calibration, projection, height and support axes
end
