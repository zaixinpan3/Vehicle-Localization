function cfg=overlapGradientConfig()
% overlapGradientConfig Correspondence-free LiDAR channel from semanticGaussianOverlap.
% At the observer prediction, each feature channel scores its own map and
% source mixtures. Average the channel negative-log gradients and metrics
% at each bandwidth, then sum over bandwidths.
    cfg=struct();
    geometry=semanticGeometryConfig();
    cfg.localMapRadius=geometry.localMapRadius;       % m, map crop around the prediction
    cfg.viewConditioning=true;                        % condition view-modeled map components at the prediction
    % Horizontal smoothing bandwidths. 0 is the exact overlap; a wider scale is
    % the expected overlap under an isotropic prediction error of that size.
    % The dyadic ladder adds broader overlap terms up to a 2 m bandwidth;
    % this is not a guaranteed capture radius for a multimodal map. On the
    % recorded MnCAV drive [0 0.5 1], [0 0.5 1 2] and [0 0.5 1 2 4] agreed
    % within 1 mm, while omitting 0.5 m was worse.
    cfg.scaleLadder=[0 0.5 1 2];                      % m
    % Evidence factors on mixture masses before class balancing:
    % map view reliability and source temporal stability.
    cfg.evidenceWeights=true;
    % "channel": one normalized overlap per shared semantic feature class.
    % "frame": -log of the class-balanced overlap of the whole horizon.
    % "landmark": -sum_j v_j log(outlierDensity + sum_i u_i N_ij), the mixture
    % likelihood of every source landmark under the map with a uniform outlier
    % floor (localizationSupport.semanticLandmarkLikelihood); evidence factors
    % then apply after class normalization.
    cfg.aggregation="channel";
    cfg.outlierDensity=1e-4;                          % 1/m^2, landmark aggregation only
    % Landmark aggregation only: components carry their support axis,
    % and pairs are weighted by exp(-kappa*sin(dtheta)^2/2) with the configured
    % angular floor and axis variances.
    cfg.orientation=false;
    cfg.geometry=geometry;                          % calibration, projection, height and support axes
end
