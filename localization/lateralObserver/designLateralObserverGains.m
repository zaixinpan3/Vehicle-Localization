function design = designLateralObserverGains(cfg)
% designLateralObserverGains Synthesize column-separated affine lateral gains.
% Measurements are [Vx*ay; r], with known steering feedthrough subtracted.
% Schedule P(v) independently of Y(v) = X*L(v), with a common invertible X.
% The acceleration column is affine in 1/v, the yaw column affine in v.
% The acceleration column is strictly reciprocal, so v*L_a(v) is constant.
% Twelve expanded LMIs use v*(1/v)=1 only on the physical curve, including
% the affine noise extension at auxiliary vertices and the derivative Pdot.
% Each LMI is 6-by-6 for the nominal linear LPV model.
    if nargin < 1 || isempty(cfg)
        cfg = lateralObserverConfig();
    end
    assert(exist("sdpvar", "file") == 2, ...
        "YALMIP must be on the MATLAB path before synthesizing the lateral observer gains.");
    validateConfiguration(cfg);
    sigma = double(cfg.synthesis.measurementErrorScale(:));
    model = lateralObserverSupport.lateralBicycleModel(cfg.vehicle);
    polytope = lateralObserverSupport.buildSchedulingPolytope(cfg.scheduling.speedRange);
    domain = lateralObserverSupport.buildCertificateDomain(model, polytope, cfg.scheduling.speedRateRange);
    options = sdpsettings('solver', char(string(cfg.synthesis.solver)), ...
        'verbose', double(cfg.synthesis.verbose), 'dualize', double(cfg.synthesis.dualize));
    weight = double(cfg.synthesis.initialErrorWeight);

    variables = synthesisVariables();
    gainSquared = sdpvar(1, 1);
    [constraints, blocks, noiseMatrices] = certificateConstraints( ...
        variables, gainSquared, domain, model, polytope, sigma, cfg.synthesis);
    objective = gainSquared + weight*variables.pUpper;
    diagnostics = optimize(constraints, objective, options);
    assert(diagnostics.problem == 0, ...
        "The column-affine expanded ISS synthesis failed: %s", diagnostics.info);
    optimalObjective = value(objective);
    maximumEigenvalue = max(cellfun(@(block) max(eig(value(block))), blocks));
    assert(isfinite(maximumEigenvalue) && maximumEigenvalue < 0, ...
        'VehicleLocalization:InvalidLateralCertificate', ...
        'The numerical solution fails an expanded inequality (maximum eigenvalue %.9g).', maximumEigenvalue);

    design = struct();
    design.model = model;
    design.polytope = polytope;
    design.certificateDomain = domain;
    design.certificateMethod = "column-affine-expanded-iss-single-v6";
    design.gainRecoveryMatrix = value(variables.X);
    design.gainBasis = struct('accelerationReciprocal', value(variables.basis.accelerationReciprocal), ...
        'yawConstant', value(variables.basis.yawConstant), 'yawSpeed', value(variables.basis.yawSpeed));
    design.vertexLyapunovMatrices = zeros(2, 2, 3);
    design.vertexY = zeros(2, 2, 3);
    design.vertexGains = zeros(2, 2, 3);
    for idx = 1:3
        design.vertexLyapunovMatrices(:, :, idx) = value(variables.P{idx});
        design.vertexY(:, :, idx) = lateralObserverSupport.evaluateLateralGainBasis( ...
            design.gainBasis, polytope.vertices(:, idx));
        design.vertexGains(:, :, idx) = design.gainRecoveryMatrix\design.vertexY(:, :, idx);
    end
    design.decayRate = double(cfg.synthesis.decayRate);
    design.dilationTimeScale = double(cfg.synthesis.dilationTimeScale);
    design.normalizedInputSize = 2;
    design.measurementErrorScale = sigma;
    % Diagnostics only: no second-stage noise optimization.
    design.vertexNoiseBounds = cellfun(@(S) norm(value(S), 2), noiseMatrices);
    design.uniformNoiseGainNormBound = max(design.vertexNoiseBounds);
    design.maxVertexGainNorm = max(arrayfun(@(idx) norm(design.vertexGains(:, :, idx)), 1:3));
    design.uniformGainNormBound = design.maxVertexGainNorm;
    design.maximumCertificateEigenvalue = maximumEigenvalue;
    design.metricUpperBound = max(arrayfun(@(idx) max(eig(design.vertexLyapunovMatrices(:, :, idx))), 1:3));
    design.optimalObjective = optimalObjective;
    design.initialErrorWeight = weight;
    design.issGainLimit = sqrt(max(value(gainSquared), 0));
    design.issGain = design.issGainLimit;
    design.errorBound = design.issGain/sqrt(2*design.decayRate);
    design.cfg = cfg;
    design.solverSolved = diagnostics.problem == 0;
    design.solverInfo = string(diagnostics.info);
end

function validateConfiguration(cfg)
% validateConfiguration Reject stale or inconsistent synthesis assumptions.
    assert(~isfield(cfg.synthesis, 'lipschitzConstant') && ...
        ~isfield(cfg.observer, 'nonlinearity'), ...
        'VehicleLocalization:ObsoleteLateralConfiguration', ...
        'Use the current linear LPV configuration; nonlinear lateral dynamics are no longer supported.');
    assert(isfield(cfg.scheduling, 'speedRateRange') && ...
        isfield(cfg.synthesis, 'dilationTimeScale') && ~isfield(cfg.synthesis, 'accelerationGainConstant') && ...
        ~isfield(cfg.synthesis, 'stiffnessRelativeErrorBound') && ...
        ~isfield(cfg.synthesis, 'stiffnessRegressorScale'), ...
        'VehicleLocalization:ObsoleteLateralConfiguration', ...
        'Use the current column-affine configuration with a strictly reciprocal acceleration gain, speed-rate bounds and a fixed dilation time scale.');
    sigma = double(cfg.synthesis.measurementErrorScale(:));
    assert(numel(sigma) == 2 && all(isfinite(sigma)) && all(sigma > 0), ...
        'VehicleLocalization:ObsoleteLateralConfiguration', ...
        'Require two positive physical error scales in [ay; r] order.');
    assert(~isfield(cfg.synthesis, 'issGainRelaxation'), ...
        'VehicleLocalization:ObsoleteLateralConfiguration', ...
        'Remove issGainRelaxation; lateral gains are synthesized with a single SDP.');
    positive = ["decayRate", "strictnessEpsilon", "dilationTimeScale"];
    for name = positive
        scalar = double(cfg.synthesis.(name));
        assert(isscalar(scalar) && isfinite(scalar) && scalar > 0, ...
            "cfg.synthesis.%s must be positive and finite.", name);
    end
    weight = double(cfg.synthesis.initialErrorWeight);
    assert(isscalar(weight) && isfinite(weight) && weight >= 0, ...
        "cfg.synthesis.initialErrorWeight must be nonnegative and finite.");
    ceiling = double(cfg.synthesis.pCeiling);
    assert(isscalar(ceiling) && ceiling >= 1 + cfg.synthesis.strictnessEpsilon, ...
        "cfg.synthesis.pCeiling must allow the normalized Lyapunov matrices.");
end

function variables = synthesisVariables()
% synthesisVariables Independent metric vertices, common X and affine columns.
    variables = struct();
    variables.pUpper = sdpvar(1, 1);
    variables.P = {sdpvar(2, 2, 'symmetric'), sdpvar(2, 2, 'symmetric'), sdpvar(2, 2, 'symmetric')};
    variables.X = sdpvar(2, 2, 'full');
    variables.basis = struct('accelerationReciprocal', sdpvar(2, 1, 'full'), ...
        'yawConstant', sdpvar(2, 1, 'full'), 'yawSpeed', sdpvar(2, 1, 'full'));
end

function [constraints, blocks, noiseMatrices] = certificateConstraints( ...
        variables, gainSquared, domain, model, polytope, sigma, cfg)
% certificateConstraints Jointly affine expanded LMIs, three positions x four rates.
% On the physical curve, R=X*(A-L*C) and S=X*Bd.
% A null-space congruence recovers the original P-metric inequality. The
% off-curve noise extension must use Gq, not v*q*Gq.
    margin = double(cfg.strictnessEpsilon);
    epsilon = double(cfg.dilationTimeScale);
    X = variables.X;
    constraints = [gainSquared >= 0, variables.pUpper >= 1, X + X.' >= 2*eye(2)];
    blocks = cell(domain.numLmis, 1);
    noiseMatrices = cell(3, 1);
    for idx = 1:3
        P = variables.P{idx};
        rho = polytope.vertices(:, idx);
        Y = lateralObserverSupport.evaluateLateralGainBasis(variables.basis, rho);
        R = X*domain.vertexA(:, :, idx) - Y*model.C;
        speedColumn = variables.basis.accelerationReciprocal;
        S = [-sigma(1)*speedColumn, -sigma(2)*Y(:, 2)];
        noiseMatrices{idx} = S;
        constraints = [constraints, P >= (1 + margin)*eye(2), P <= variables.pUpper*eye(2)]; %#ok<AGROW>
        if isfinite(cfg.pCeiling)
            constraints = [constraints, P <= cfg.pCeiling*eye(2)]; %#ok<AGROW>
        end
        for rateIdx = 1:domain.numRateVertices
            rate = domain.coordinateRateVertices(:, rateIdx);
            Pdot = rate(1)*variables.P{1} + rate(2)*variables.P{2} + rate(3)*variables.P{3};
            Phi = Pdot + R + R.' + 2*cfg.decayRate*P;
            block = [Phi, P - X + epsilon*R.', S; ...
                P - X.' + epsilon*R, -epsilon*(X + X.'), epsilon*S; ...
                S.', epsilon*S.', -gainSquared*eye(2)];
            constraints = [constraints, block <= -margin*eye(size(block, 1))]; %#ok<AGROW>
            blocks{domain.numRateVertices*(idx - 1) + rateIdx} = block;
        end
    end
end
