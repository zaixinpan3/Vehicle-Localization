function cfg=overlapGradientConfig(options)
% overlapGradientConfig Correspondence-free LiDAR channel from semanticGaussianOverlap.
% At the observer prediction, each shared feature channel evaluates its exact
% Cauchy-Schwarz divergence (negative log L2 overlap) at each requested scale and
% intrinsic scan Jacobian Gram, averaged across evaluable semantic classes.
% UniformBackground mixes each class with uniform spatial density over the map crop.
    arguments
        options.UniformBackground (1,1) logical=false
    end
    cfg=struct('uniformBackground',options.UniformBackground);
    geometry=semanticGeometryConfig();
    cfg.localMapRadius=geometry.localMapRadius;       % m, map crop around the prediction
    cfg.geometry=geometry;                          % calibration, projection, height and support axes
end
