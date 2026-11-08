function design = designLateralObserverGains(cfg)
% designLateralObserverGains Synthesize a yaw-rate-only LPV lateral observer.
% With x = [vy; r], y = r + nu and C = [0,1], the error satisfies
%   edot = (A(rho)-L(rho)*C)*e + df + E*w - L(rho)*nu.
% A and L are affine on the three-vertex scheduling triangle. A common
% P >= I and Y_i = P*L_i give three ordinary convex ISS LMIs:
%   [He(P*A_i-Y_i*C)+2*kappa*P+tau*gamma_f^2*I, S_i, P;
%    S_i', -gamma^2*I, 0; P, 0, -tau*I] < 0,
% where S_i = [P*E*Ww, -Y_i*sigma_r]. For gamma_f = 0 the last row and
% column are omitted, giving 5-by-5 LMIs and no Young multiplier.
% The positive Young multiplier tau
% is optimized directly. No affine extension, slack matrix, scheduling-rate
% bound, parameter search or parameter-dependent Lyapunov matrix is needed.
% First minimize gamma^2, then minimize vertex-gain norm bounds within the
% configured ISS relaxation. See localization/lateralObserver/README.md.
% The certificate concerns continuous time inside the positive-speed domain;
% it does not certify sampled integration or identify sensor bias.
% Requires YALMIP and the configured SDP solver on the MATLAB path.
    if nargin < 1 || isempty(cfg)
        cfg = lateralObserverConfig();
    end
    assert(exist("sdpvar", "file") == 2, ...
        "YALMIP must be on the MATLAB path before synthesizing the lateral observer gains.");
    scales = disturbanceScales(cfg.synthesis);
    model = lateralObserverSupport.lateralBicycleModel(cfg.vehicle);
    polytope = lateralObserverSupport.buildSchedulingPolytope(cfg.scheduling.speedRange);
    domain = lateralObserverSupport.buildCertificateDomain(model, polytope);
    relaxation = double(cfg.synthesis.issGainRelaxation);
    assert(isscalar(relaxation) && isfinite(relaxation) && relaxation > 0, ...
        "cfg.synthesis.issGainRelaxation must be a positive finite scalar.");

    variables = synthesisVariables(cfg.synthesis.lipschitzConstant);
    gainSquared = sdpvar(1, 1);
    constraints = certificateConstraints(variables, gainSquared, domain, model, scales, cfg.synthesis);
    options = sdpsettings('solver', char(string(cfg.synthesis.solver)), ...
        'verbose', double(cfg.synthesis.verbose), 'dualize', double(cfg.synthesis.dualize));
    diagnostics = optimize(constraints, gainSquared, options);
    assert(diagnostics.problem == 0, ...
        "The yaw-rate-only common-Lyapunov ISS synthesis failed: %s", diagnostics.info);
    optimalIssGain = sqrt(max(value(gainSquared), 0));
    gainLimit = (1 + relaxation)*optimalIssGain;

    variables = synthesisVariables(cfg.synthesis.lipschitzConstant);
    bounds = sdpvar(3, 1);
    constraints = certificateConstraints(variables, gainLimit^2, domain, model, scales, cfg.synthesis);
    for idx = 1:3
        % P >= I implies ||P\Y_i|| <= ||Y_i|| <= bounds(i).
        constraints = [constraints, ...
            [bounds(idx)*eye(2), variables.Y{idx}; variables.Y{idx}.', bounds(idx)] >= 0]; %#ok<AGROW>
    end
    diagnostics = optimize(constraints, sum(bounds), options);
    assert(diagnostics.problem == 0, ...
        "The yaw-rate-only minimum-gain program failed: %s", diagnostics.info);

    design = struct();
    design.model = model;
    design.polytope = polytope;
    design.certificateDomain = domain;
    design.certificateMethod = "yaw-rate-common-lyapunov-v1";
    design.lyapunovMatrix = value(variables.P);
    design.vertexGains = zeros(2, 1, 3);
    for idx = 1:3
        design.vertexGains(:, :, idx) = design.lyapunovMatrix \ value(variables.Y{idx});
    end
    design.tau = value(variables.tau);
    design.decayRate = double(cfg.synthesis.decayRate);
    design.axleForceMismatchScale = scales.mismatch;
    design.measurementErrorScale = scales.measurement;
    design.vertexGainBounds = value(bounds);
    design.maxVertexGainNorm = max(vecnorm(reshape(design.vertexGains, 2, 3)));
    design.optimalIssGain = optimalIssGain;
    design.issGainLimit = gainLimit;
    design.issGain = gainLimit;
    design.errorBound = gainLimit/sqrt(2*design.decayRate);
    design.lipschitzConstant = double(cfg.synthesis.lipschitzConstant);
    design.cfg = cfg;
    design.solverSolved = diagnostics.problem == 0;
    design.solverInfo = string(diagnostics.info);
end

function scales = disturbanceScales(cfg)
% disturbanceScales Normalize axle-force errors and the scalar gyro error.
    scales = struct('mismatch', double(cfg.axleForceMismatchScale(:)), ...
        'measurement', double(cfg.measurementErrorScale));
    assert(numel(scales.mismatch) == 2 && all(isfinite(scales.mismatch)) && all(scales.mismatch > 0), ...
        "cfg.synthesis.axleForceMismatchScale must hold two positive finite scales [front; rear].");
    assert(isscalar(scales.measurement) && isfinite(scales.measurement) && scales.measurement > 0, ...
        "VehicleLocalization:ObsoleteLateralConfiguration", ...
        "cfg.synthesis.measurementErrorScale must be one positive finite yaw-rate scale. Re-synthesize the yaw-rate-only design.");
end

function variables = synthesisVariables(lipschitzConstant)
% synthesisVariables Common metric, three unconstrained gain columns and tau.
    variables = struct();
    variables.P = sdpvar(2, 2, 'symmetric');
    variables.Y = {sdpvar(2, 1, 'full'), sdpvar(2, 1, 'full'), sdpvar(2, 1, 'full')};
    if lipschitzConstant == 0
        variables.tau = 0; % No nonlinearity difference or Young inequality.
    else
        variables.tau = sdpvar(1, 1);
    end
end

function constraints = certificateConstraints(variables, gainSquared, domain, model, scales, cfg)
% certificateConstraints Three 7-by-7 affine vertex LMIs with a common P.
    kappa = double(cfg.decayRate);
    gammaF = double(cfg.lipschitzConstant);
    margin = double(cfg.strictnessEpsilon);
    pCeiling = double(cfg.pCeiling);
    assert(isscalar(kappa) && isfinite(kappa) && kappa > 0, ...
        "cfg.synthesis.decayRate must be positive and finite.");
    assert(isscalar(gammaF) && isfinite(gammaF) && gammaF >= 0, ...
        "cfg.synthesis.lipschitzConstant must be nonnegative and finite.");
    assert(isscalar(margin) && isfinite(margin) && margin > 0, ...
        "cfg.synthesis.strictnessEpsilon must be positive and finite.");
    assert(isscalar(pCeiling) && pCeiling >= 1 + margin, ...
        "cfg.synthesis.pCeiling must allow the normalized Lyapunov matrix.");
    P = variables.P;
    tau = variables.tau;
    constraints = [P >= (1 + margin)*eye(2), gainSquared >= 0];
    if gammaF > 0
        constraints = [constraints, tau >= margin];
    end
    if isfinite(pCeiling)
        constraints = [constraints, P <= pCeiling*eye(2)];
    end
    for idx = 1:3
        Y = variables.Y{idx};
        R = P*domain.vertexA(:, :, idx) - Y*model.C;
        S = [P*model.E*diag(scales.mismatch), -Y*scales.measurement];
        Phi = R + R.' + 2*kappa*P + tau*gammaF^2*eye(2);
        if gammaF == 0
            block = [Phi, S; S.', -gainSquared*eye(3)];
        else
            block = [Phi, S, P; S.', -gainSquared*eye(3), zeros(3, 2); ...
                P, zeros(2, 3), -tau*eye(2)];
        end
        constraints = [constraints, block <= -margin*eye(size(block, 1))]; %#ok<AGROW>
    end
end
