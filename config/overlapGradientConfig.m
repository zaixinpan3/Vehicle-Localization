function cfg=overlapGradientConfig()
% overlapGradientConfig Correspondence-free LiDAR channel from semanticGaussianOverlap.
% At its own post-baseline prediction the full observer evaluates the
% class-balanced semantic overlap of the source horizon and the map at every
% bandwidth of the scale ladder, and injects the summed gradient of -log
% overlap with the summed Gauss-Newton metric as information
% (lidarInjectionSupport.evaluateOverlapGradient). No pose is optimized and no
% correspondence is selected.
    cfg=struct();
    registration=distributionRegistrationConfig();
    cfg.localMapRadius=registration.localMapRadius;   % m, map crop around the prediction
    cfg.viewConditioning=true;                        % condition view-modeled map components at the prediction
    % Horizontal smoothing bandwidths. 0 is the exact overlap; a wider scale is
    % the expected overlap under an isotropic prediction error of that size.
    % The dyadic ladder keeps a restoring force out to about 2 m. On the
    % recorded MnCAV drive [0 0.5 1], [0 0.5 1 2] and [0 0.5 1 2 4] agreed
    % within 1 mm, while omitting 0.5 m was worse.
    cfg.scaleLadder=[0 0.5 1 2];                      % m
    % Registration's evidence factors on the mixture masses before class
    % balancing: map view reliability and source temporal stability.
    cfg.evidenceWeights=true;
    % "frame": -log of the class-balanced overlap of the whole horizon.
    % "landmark": -sum_j v_j log(outlierDensity + sum_i u_i N_ij), the mixture
    % likelihood of every source landmark under the map with a uniform outlier
    % floor (registrationSupport.semanticLandmarkLikelihood); evidence factors
    % then apply after class normalization, as registration's weights.
    cfg.aggregation="frame";
    cfg.outlierDensity=1e-4;                          % 1/m^2, landmark aggregation only
    % Landmark aggregation only: components carry registration's support axis,
    % and pairs are weighted by exp(-kappa*sin(dtheta)^2/2) with registration's
    % angular floor and axis variances.
    cfg.orientation=false;
    cfg.registration=registration;                    % calibration, projection, height mode and support geometry
end
