function design = designLateralObserverGains(cfg)
% designLateralObserverGains Synthesize a scheduled virtual-acceleration observer.
% Nominal bicycle model, y = [Vx*ay; r]. Schedule P and Y = P*L with the same
% triangle coordinates, and recover L = P\Y. Six symmetric position pairs
% and four coordinate-rate vertices certify Pdot and all product terms.
% The ISS inputs are physical accelerometer and gyro errors divided by their scales;
% speed amplification is retained explicitly. No stiffness mismatch is modeled.
    if nargin < 1 || isempty(cfg)
        cfg = lateralObserverConfig();
    end
    assert(exist("sdpvar", "file") == 2, ...
        "YALMIP must be on the MATLAB path before synthesizing the lateral observer gains.");
    assert(isfield(cfg.scheduling, 'speedRateRange') && ...
        ~isfield(cfg.synthesis, 'stiffnessRelativeErrorBound') && ...
        ~isfield(cfg.synthesis, 'stiffnessRegressorScale'), ...
        'VehicleLocalization:ObsoleteLateralConfiguration', ...
        'Use the current virtual-output configuration with a scheduling speed-rate interval.');
    sigma = double(cfg.synthesis.measurementErrorScale(:));
    assert(numel(sigma) == 2 && all(isfinite(sigma)) && all(sigma > 0), ...
        'VehicleLocalization:ObsoleteLateralConfiguration', ...
        'Require two positive physical error scales in [ay; r] order.');
    model = lateralObserverSupport.lateralBicycleModel(cfg.vehicle);
    polytope = lateralObserverSupport.buildSchedulingPolytope(cfg.scheduling.speedRange);
    domain = lateralObserverSupport.buildCertificateDomain(model, polytope, cfg.scheduling.speedRateRange);
    relaxation = double(cfg.synthesis.issGainRelaxation);
    assert(isscalar(relaxation) && isfinite(relaxation) && relaxation > 0, ...
        "cfg.synthesis.issGainRelaxation must be a positive finite scalar.");
    weight = double(cfg.synthesis.initialErrorWeight);
    assert(isscalar(weight) && isfinite(weight) && weight >= 0, ...
        "cfg.synthesis.initialErrorWeight must be finite and nonnegative.");
    options = sdpsettings('solver', char(string(cfg.synthesis.solver)), ...
        'verbose', double(cfg.synthesis.verbose), 'dualize', double(cfg.synthesis.dualize));

    variables = synthesisVariables(cfg.synthesis.lipschitzConstant);
    gainSquared = sdpvar(1, 1);
    constraints = certificateConstraints(variables, gainSquared, domain, model, polytope, sigma, cfg.synthesis);
    objective = gainSquared + weight*variables.pUpper;
    diagnostics = optimize(constraints, objective, options);
    assert(diagnostics.problem == 0, ...
        "The scheduled virtual-output ISS synthesis failed: %s", diagnostics.info);
    optimalObjective = value(objective);
    firstStageIssGain = sqrt(max(value(gainSquared), 0));
    objectiveLimit = (1 + relaxation)^2*optimalObjective;

    variables = synthesisVariables(cfg.synthesis.lipschitzConstant);
    gainSquared = sdpvar(1, 1);
    bounds = sdpvar(3, 1);
    [constraints, blocks] = certificateConstraints( ...
        variables, gainSquared, domain, model, polytope, sigma, cfg.synthesis);
    constraints = [constraints, gainSquared + weight*variables.pUpper <= objectiveLimit];
    for idx = 1:3
        % P(v) >= I and convex Y(v) give ||L(v)|| <= max_i ||Y_i||.
        constraints = [constraints, ...
            [bounds(idx)*eye(2), variables.Y{idx}; variables.Y{idx}.', bounds(idx)*eye(2)] >= 0]; %#ok<AGROW>
    end
    diagnostics = optimize(constraints, sum(bounds), options);
    assert(diagnostics.problem == 0, ...
        "The scheduled virtual-output minimum-gain program failed: %s", diagnostics.info);
    maximumEigenvalue = max(cellfun(@(block) max(eig(value(block))), blocks));
    assert(isfinite(maximumEigenvalue) && maximumEigenvalue < 0, ...
        'VehicleLocalization:InvalidLateralCertificate', ...
        'The numerical solution fails a certificate inequality (maximum eigenvalue %.9g).', maximumEigenvalue);

    design = struct();
    design.model = model;
    design.polytope = polytope;
    design.certificateDomain = domain;
    design.certificateMethod = "virtual-ay-yaw-scheduled-metric-iss-v2";
    design.vertexLyapunovMatrices = zeros(2, 2, 3);
    design.vertexY = zeros(2, 2, 3);
    design.vertexGains = zeros(2, 2, 3); % Endpoint/vertex reporting only.
    for idx = 1:3
        design.vertexLyapunovMatrices(:, :, idx) = value(variables.P{idx});
        design.vertexY(:, :, idx) = value(variables.Y{idx});
        design.vertexGains(:, :, idx) = design.vertexLyapunovMatrices(:, :, idx)\design.vertexY(:, :, idx);
    end
    design.tau = value(variables.tau);
    design.decayRate = double(cfg.synthesis.decayRate);
    design.normalizedInputSize = 2;
    design.measurementErrorScale = sigma;
    design.vertexGainBounds = value(bounds);
    design.uniformGainNormBound = max(design.vertexGainBounds);
    design.maxVertexGainNorm = max(arrayfun(@(idx) norm(design.vertexGains(:, :, idx)), 1:3));
    design.maximumCertificateEigenvalue = maximumEigenvalue;
    design.metricUpperBound = max(arrayfun(@(idx) max(eig(design.vertexLyapunovMatrices(:, :, idx))), 1:3));
    design.optimalObjective = optimalObjective;
    design.objectiveLimit = objectiveLimit;
    design.firstStageIssGain = firstStageIssGain;
    design.initialErrorWeight = weight;
    design.issGainLimit = sqrt(max(value(gainSquared), 0));
    design.issGain = design.issGainLimit;
    design.errorBound = design.issGain/sqrt(2*design.decayRate);
    design.lipschitzConstant = double(cfg.synthesis.lipschitzConstant);
    design.cfg = cfg;
    design.solverSolved = diagnostics.problem == 0;
    design.solverInfo = string(diagnostics.info);
end

function variables = synthesisVariables(lipschitzConstant)
% synthesisVariables Three metrics and transformed gains, with common tau.
    variables = struct();
    variables.pUpper = sdpvar(1, 1);
    variables.P = {sdpvar(2, 2, 'symmetric'), sdpvar(2, 2, 'symmetric'), sdpvar(2, 2, 'symmetric')};
    variables.Y = {sdpvar(2, 2, 'full'), sdpvar(2, 2, 'full'), sdpvar(2, 2, 'full')};
    if lipschitzConstant == 0
        variables.tau = 0;
    else
        variables.tau = sdpvar(1, 1);
    end
end

function [constraints, blocks] = certificateConstraints(variables, gainSquared, domain, model, polytope, sigma, cfg)
% certificateConstraints Enclose P*A and Vx*Y by symmetric position pairs.
% Blending diagonal blocks with alpha_i^2 and off-diagonal blocks with
% 2*alpha_i*alpha_j reconstructs the physical inequality exactly. A separate
% convex blend encloses Pdot. Matched position vertices alone do not suffice.
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
    tau = variables.tau;
    constraints = [gainSquared >= 0, variables.pUpper >= 1];
    if gammaF > 0
        constraints = [constraints, tau >= margin];
    end
    for idx = 1:3
        constraints = [constraints, variables.P{idx} >= (1 + margin)*eye(2), ...
            variables.P{idx} <= variables.pUpper*eye(2)]; %#ok<AGROW>
        if isfinite(pCeiling)
            constraints = [constraints, variables.P{idx} <= pCeiling*eye(2)]; %#ok<AGROW>
        end
    end
    blocks = cell(domain.numVertexPairs*domain.numRateVertices, 1);
    blockIdx = 0;
    for left = 1:3
        for right = left:3
            P = (variables.P{left} + variables.P{right})/2;
            Y = (variables.Y{left} + variables.Y{right})/2;
            R = (variables.P{left}*domain.vertexA(:, :, right) + ...
                variables.P{right}*domain.vertexA(:, :, left))/2 - Y*model.C;
            speedY = (polytope.vertices(1, right)*variables.Y{left} + ...
                polytope.vertices(1, left)*variables.Y{right})/2;
            noise = [-sigma(1)*speedY(:, 1), -sigma(2)*Y(:, 2)];
            nonlinear = P - speedY(:, 1)*[1, 0];
            for rateIdx = 1:domain.numRateVertices
                rate = domain.coordinateRateVertices(:, rateIdx);
                Pdot = rate(1)*variables.P{1} + rate(2)*variables.P{2} + rate(3)*variables.P{3};
                Phi = Pdot + R + R.' + 2*kappa*P + tau*gammaF^2*eye(2);
                if gammaF == 0
                    block = [Phi, noise; noise.', -gainSquared*eye(2)];
                else
                    block = [Phi, noise, nonlinear; noise.', -gainSquared*eye(2), zeros(2); ...
                        nonlinear.', zeros(2), -tau*eye(2)];
                end
                constraints = [constraints, block <= -margin*eye(size(block, 1))]; %#ok<AGROW>
                blockIdx = blockIdx + 1;
                blocks{blockIdx} = block;
            end
        end
    end
end
