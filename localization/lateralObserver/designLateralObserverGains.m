function design = designLateralObserverGains(cfg)
% designLateralObserverGains Synthesize the two-state yaw-only ISS observer.
% Structured stiffness deviations are represented exactly by the true-state
% regressors [eta_f*vy; eta_f*r; eta_f*delta; eta_r*vy; eta_r*r]. The gyro
% error is the sixth normalized input. This Zhou-inspired model retains the
% existing common-metric ISS objective and does not reproduce an augmented
% L1 observer. The bound is conditional on bounded physical motion; persistent
% stiffness error can produce a nonzero estimation error.
    if nargin < 1 || isempty(cfg)
        cfg = lateralObserverConfig();
    end
    assert(exist("sdpvar", "file") == 2, ...
        "YALMIP must be on the MATLAB path before synthesizing the lateral observer gains.");
    scales = disturbanceScales(cfg.synthesis);
    model = lateralObserverSupport.lateralBicycleModel(cfg.vehicle);
    uncertainty = lateralObserverSupport.stiffnessUncertainty(model, scales.relativeBound);
    polytope = lateralObserverSupport.buildSchedulingPolytope(cfg.scheduling.speedRange);
    domain = lateralObserverSupport.buildCertificateDomain(model, polytope);
    domain.vertexReciprocalSpeed = polytope.vertices(2,:);
    relaxation = double(cfg.synthesis.issGainRelaxation);
    assert(isscalar(relaxation) && isfinite(relaxation) && relaxation > 0, ...
        "cfg.synthesis.issGainRelaxation must be a positive finite scalar.");

    variables = synthesisVariables(cfg.synthesis.lipschitzConstant);
    gainSquared = sdpvar(1, 1);
    constraints = certificateConstraints(variables, gainSquared, domain, model, scales, uncertainty, cfg.synthesis);
    options = sdpsettings('solver', char(string(cfg.synthesis.solver)), ...
        'verbose', double(cfg.synthesis.verbose), 'dualize', double(cfg.synthesis.dualize));
    diagnostics = optimize(constraints, gainSquared, options);
    assert(diagnostics.problem == 0, ...
        "The yaw-rate-only common-Lyapunov ISS synthesis failed: %s", diagnostics.info);
    optimalIssGain = sqrt(max(value(gainSquared), 0));
    gainLimit = (1 + relaxation)*optimalIssGain;

    variables = synthesisVariables(cfg.synthesis.lipschitzConstant);
    bounds = sdpvar(3, 1);
    constraints = certificateConstraints(variables, gainLimit^2, domain, model, scales, uncertainty, cfg.synthesis);
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
    design.certificateMethod = "yaw-rate-stiffness-regressor-iss-v1";
    design.lyapunovMatrix = value(variables.P);
    design.vertexGains = zeros(2, 1, 3);
    for idx = 1:3
        design.vertexGains(:, :, idx) = design.lyapunovMatrix \ value(variables.Y{idx});
    end
    design.tau = value(variables.tau);
    design.decayRate = double(cfg.synthesis.decayRate);
    design.stiffnessUncertainty = uncertainty;
    design.stiffnessRegressorScale = scales.regressor;
    design.normalizedInputSize = 6;
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
% disturbanceScales Normalize physical stiffness regressors and gyro error.
    scales = struct('relativeBound',double(cfg.stiffnessRelativeErrorBound(:)), ...
        'regressor',double(cfg.stiffnessRegressorScale(:)), ...
        'measurement',double(cfg.measurementErrorScale));
    assert(numel(scales.regressor)==5 && all(isfinite(scales.regressor)) && ...
        all(scales.regressor>0), 'Require five positive stiffness regressor scales.');
    assert(isscalar(scales.measurement) && isfinite(scales.measurement) && scales.measurement>0, ...
        'VehicleLocalization:ObsoleteLateralConfiguration', ...
        'Require one positive yaw-rate error scale.');
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

function constraints = certificateConstraints(variables, gainSquared, domain, model, scales, uncertainty, cfg)
% certificateConstraints Three affine vertex LMIs with a common P.
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
        reciprocalSpeed = domain.vertexReciprocalSpeed(idx);
        G = [reciprocalSpeed*uncertainty.frontState, uncertainty.frontSteering, ...
            reciprocalSpeed*uncertainty.rearState]*diag(scales.regressor);
        S = [P*G, -Y*scales.measurement];
        Phi = R + R.' + 2*kappa*P + tau*gammaF^2*eye(2);
        if gammaF == 0
            block = [Phi, S; S.', -gainSquared*eye(6)];
        else
            block = [Phi, S, P; S.', -gainSquared*eye(6), zeros(6, 2); ...
                P, zeros(2, 6), -tau*eye(2)];
        end
        constraints = [constraints, block <= -margin*eye(size(block, 1))]; %#ok<AGROW>
    end
end
