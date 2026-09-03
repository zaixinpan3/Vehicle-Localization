function design = designLateralObserverGains(cfg)
% designLateralObserverGains: Synthesize the parameter-dependent gain of the
% LPV lateral-velocity observer by semidefinite programming. The error
% dynamics of the Luenberger observer are
%
%   edot = (A(rho) - L(rho) C(rho)) e + df - L(rho) v,
%
% with df the Lipschitz-bounded nonlinearity mismatch and v the measurement
% noise. With the parameter-dependent Lyapunov function V = e' P(rho) e,
% P(rho) = sum_i alpha_i(rho) P_i, bounding 2 e' P df by Young's inequality
% with a fixed scalar tau, and weighting the performance output
% z = Q^(1/2) e, internal stability and the H2 bound ||T_{v->z}||^2 < mu
% follow from
%
%   P Acl + Acl' P + Pdot + Q + tau P^2 + (gamma_f^2/tau) I < 0,
%   trace(L' P L) < mu.
%
% Those are made convex by Y = P L, by the slack matrix X that decouples P
% from A through He(P A - Y C) = He(X A - Y C) + He((P - X) A) and a second
% Young step with eta, and by two Schur complements. The resulting LMIs are
% imposed on a grid of scheduling points and longitudinal accelerations; the
% recovered gains are then checked against the original, non-convexified
% certificate above.
%
% Requires YALMIP and an SDP solver (SeDuMi by default) on the MATLAB path.
%
% Input:
%   cfg: optional struct from lateralObserverConfig
%
% Output:
%   design: struct with the model, the scheduling polytope, the design grid,
%       the Lyapunov basis matrices, the gain schedule, the selected tau and
%       eta, the H2 bound mu, and the verification margins
    if nargin < 1 || isempty(cfg)
        cfg = lateralObserverConfig();
    end
    assert(exist("sdpvar", "file") == 2, ...
        "YALMIP must be on the MATLAB path before synthesizing the lateral observer gains.");

    model = lateralBicycleModel(cfg.vehicle);
    polytope = buildSchedulingPolytope(cfg.scheduling.speedRange);
    grid = buildDesignGrid(model, polytope, cfg.scheduling);

    tauCandidates = sort(double(cfg.synthesis.tauCandidates(:).'));
    assert(~isempty(tauCandidates) && all(isfinite(tauCandidates)) && all(tauCandidates > 0), ...
        "cfg.synthesis.tauCandidates must contain positive finite scalars.");

    best = struct("found", false, "mu", inf);
    attempts = repmat(struct("tau", 0, "solved", false, "mu", NaN, "certified", false, "info", ""), numel(tauCandidates), 1);
    for tauIdx = 1:numel(tauCandidates)
        tau = tauCandidates(tauIdx);
        solution = solveSynthesisLmi(grid, cfg.synthesis, tau);
        attempts(tauIdx).tau = tau;
        attempts(tauIdx).solved = solution.solved;
        attempts(tauIdx).info = solution.info;
        if ~solution.solved
            continue;
        end
        schedule = recoverGainSchedule(grid, solution);
        verification = verifyCertificate(grid, schedule, solution, cfg.synthesis, tau);
        attempts(tauIdx).mu = solution.mu;
        attempts(tauIdx).certified = verification.certified;
        if verification.certified && solution.mu < best.mu
            best = struct("found", true, "mu", solution.mu, "tau", tau, ...
                "solution", solution, "schedule", schedule, "verification", verification);
        end
    end
    assert(best.found, ...
        "No tau candidate produced a certified lateral observer design. Widen cfg.synthesis.tauCandidates or relax the operating range.");

    design = struct();
    design.model = model;
    design.polytope = polytope;
    design.grid = grid;
    design.speedGrid = grid.speeds;
    design.accelerationGrid = grid.accelerations;
    design.gains = best.schedule.gains;
    design.lyapunovMatrices = best.schedule.lyapunovMatrices;
    design.lyapunovBasis = best.solution.P;
    design.slackMatrix = best.solution.X;
    design.tau = best.tau;
    design.eta = best.solution.eta;
    design.h2Bound = best.mu;
    design.errorWeight = double(cfg.synthesis.errorWeight);
    design.lipschitzConstant = double(cfg.synthesis.lipschitzConstant);
    design.certificateMargins = best.verification.certificateMargins;
    design.h2Values = best.verification.h2Values;
    design.errorEigenvalues = best.verification.errorEigenvalues;
    design.maxCertificateMargin = max(best.verification.certificateMargins);
    design.maxErrorEigenvalueRealPart = max(real(best.verification.errorEigenvalues(:)));
    design.certified = best.verification.certified;
    design.tauAttempts = attempts;
    design.cfg = cfg;
end

function grid = buildDesignGrid(model, polytope, schedulingCfg)
% buildDesignGrid: Enumerate the scheduling and acceleration grid points of
% the synthesis. Each point stores the physical scheduling parameter, its
% barycentric coordinates and their rate, and the model matrices there.
% Because the only acceleration-dependent term of the LMI is Pdot, which is
% linear in the acceleration, the two extreme accelerations already cover
% the whole interval.
%
% Input:
%   model: struct from lateralBicycleModel
%   polytope: struct from buildSchedulingPolytope
%   schedulingCfg: cfg.scheduling struct with the ranges and grid counts
%
% Output:
%   grid: struct with speeds, accelerations, and a struct array of points
    speedRange = double(schedulingCfg.speedRange(:).');
    accelerationRange = double(schedulingCfg.longitudinalAccelerationRange(:).');
    speedCount = round(double(schedulingCfg.speedGridCount));
    accelerationCount = round(double(schedulingCfg.accelerationGridCount));
    assert(speedCount >= 2 && accelerationCount >= 1, ...
        "The design grid needs at least two speeds and one acceleration.");
    assert(numel(accelerationRange) == 2 && accelerationRange(1) <= accelerationRange(2), ...
        "longitudinalAccelerationRange must be [aMin, aMax].");

    speeds = linspace(speedRange(1), speedRange(2), speedCount);
    if accelerationCount == 1
        accelerations = mean(accelerationRange);
    else
        accelerations = linspace(accelerationRange(1), accelerationRange(2), accelerationCount);
    end

    pointTemplate = struct("speed", 0, "acceleration", 0, "rho", zeros(2, 1), ...
        "alpha", zeros(3, 1), "alphaRate", zeros(3, 1), "A", zeros(2, 2), "C", zeros(2, 2), ...
        "speedIdx", 0, "accelerationIdx", 0);
    points = repmat(pointTemplate, speedCount .* accelerationCount, 1);
    pointIdx = 0;
    for speedIdx = 1:speedCount
        for accelerationIdx = 1:accelerationCount
            pointIdx = pointIdx + 1;
            speed = speeds(speedIdx);
            acceleration = accelerations(accelerationIdx);
            rho = [speed; 1.0 ./ speed];
            [alpha, alphaRate] = schedulingCoordinates(polytope, speed, acceleration);
            [A, C] = evaluateLateralModel(model, rho);
            points(pointIdx).speed = speed;
            points(pointIdx).acceleration = acceleration;
            points(pointIdx).rho = rho;
            points(pointIdx).alpha = alpha;
            points(pointIdx).alphaRate = alphaRate;
            points(pointIdx).A = A;
            points(pointIdx).C = C;
            points(pointIdx).speedIdx = speedIdx;
            points(pointIdx).accelerationIdx = accelerationIdx;
        end
    end

    grid = struct();
    grid.speeds = speeds;
    grid.accelerations = accelerations;
    grid.points = points;
    grid.numPoints = numel(points);
end

function solution = solveSynthesisLmi(grid, synthesisCfg, tau)
% solveSynthesisLmi: Solve the convexified synthesis for one fixed tau,
% minimizing the H2 bound mu. The decision variables are the three Lyapunov
% basis matrices, the slack matrix X, the transformed gains Y_k and the
% auxiliary matrices W_k of every grid point, the Young scalar eta, and mu.
%
% Input:
%   grid: struct from buildDesignGrid
%   synthesisCfg: cfg.synthesis struct
%   tau: fixed positive scalar of the Lipschitz Young inequality
%
% Output:
%   solution: struct with solved, P, X, Y, W, eta, mu, and solver info
    errorWeight = double(synthesisCfg.errorWeight);
    assert(isequal(size(errorWeight), [2, 2]) && norm(errorWeight - errorWeight.', "fro") <= 1.0e-12 && ...
        min(eig(errorWeight)) > 0, "cfg.synthesis.errorWeight must be a symmetric positive definite [2 x 2] matrix.");
    lipschitzConstant = double(synthesisCfg.lipschitzConstant);
    assert(isscalar(lipschitzConstant) && isfinite(lipschitzConstant) && lipschitzConstant >= 0, ...
        "cfg.synthesis.lipschitzConstant must be a nonnegative finite scalar.");
    pFloor = double(synthesisCfg.pFloor);
    pCeiling = double(synthesisCfg.pCeiling);
    etaFloor = double(synthesisCfg.etaFloor);
    strictnessEpsilon = double(synthesisCfg.strictnessEpsilon);

    P = {sdpvar(2, 2, 'symmetric'), sdpvar(2, 2, 'symmetric'), sdpvar(2, 2, 'symmetric')};
    X = sdpvar(2, 2, 'full');
    eta = sdpvar(1, 1);
    mu = sdpvar(1, 1);
    Y = cell(grid.numPoints, 1);
    W = cell(grid.numPoints, 1);

    constraints = eta >= etaFloor;
    for vertexIdx = 1:3
        constraints = [constraints, P{vertexIdx} >= pFloor .* eye(2)]; %#ok<AGROW>
        if isfinite(pCeiling)
            constraints = [constraints, P{vertexIdx} <= pCeiling .* eye(2)]; %#ok<AGROW>
        end
    end

    for pointIdx = 1:grid.numPoints
        point = grid.points(pointIdx);
        Y{pointIdx} = sdpvar(2, 2, 'full');
        W{pointIdx} = sdpvar(2, 2, 'symmetric');
        Pk = (point.alpha(1) .* P{1}) + (point.alpha(2) .* P{2}) + (point.alpha(3) .* P{3});
        PkRate = (point.alphaRate(1) .* P{1}) + (point.alphaRate(2) .* P{2}) + (point.alphaRate(3) .* P{3});
        crossTerm = (X * point.A) - (Y{pointIdx} * point.C);
        Phi = crossTerm + crossTerm.' + PkRate + errorWeight + ...
            ((lipschitzConstant.^2 ./ tau) .* eye(2)) + (eta .* (point.A.' * point.A));
        block = [Phi, Pk - X, sqrt(tau) .* Pk; ...
            (Pk - X).', -eta .* eye(2), zeros(2, 2); ...
            sqrt(tau) .* Pk, zeros(2, 2), -eye(2)];
        constraints = [constraints, block <= -strictnessEpsilon .* eye(6)]; %#ok<AGROW>
        constraints = [constraints, [W{pointIdx}, Y{pointIdx}.'; Y{pointIdx}, Pk] >= strictnessEpsilon .* eye(4)]; %#ok<AGROW>
        constraints = [constraints, trace(W{pointIdx}) <= mu]; %#ok<AGROW>
    end

    options = sdpsettings('solver', char(string(synthesisCfg.solver)), ...
        'verbose', double(synthesisCfg.verbose), 'dualize', double(synthesisCfg.dualize));
    diagnostics = optimize(constraints, mu, options);

    solution = struct();
    solution.solved = diagnostics.problem == 0;
    solution.info = string(diagnostics.info);
    solution.tau = tau;
    solution.P = zeros(2, 2, 3);
    solution.X = zeros(2, 2);
    solution.Y = zeros(2, 2, grid.numPoints);
    solution.W = zeros(2, 2, grid.numPoints);
    solution.eta = NaN;
    solution.mu = NaN;
    if ~solution.solved
        return;
    end
    for vertexIdx = 1:3
        solution.P(:, :, vertexIdx) = double(P{vertexIdx});
    end
    solution.X = double(X);
    for pointIdx = 1:grid.numPoints
        solution.Y(:, :, pointIdx) = double(Y{pointIdx});
        solution.W(:, :, pointIdx) = double(W{pointIdx});
    end
    solution.eta = double(eta);
    solution.mu = double(mu);
end

function schedule = recoverGainSchedule(grid, solution)
% recoverGainSchedule: Recover the observer gains L_k = P_k \ Y_k of every
% grid point and arrange them, with the corresponding Lyapunov matrices,
% as [2 x 2 x speedCount x accelerationCount] lookup tables.
%
% Input:
%   grid: struct from buildDesignGrid
%   solution: struct from solveSynthesisLmi
%
% Output:
%   schedule: struct with gains, lyapunovMatrices, and lyapunovRates
    speedCount = numel(grid.speeds);
    accelerationCount = numel(grid.accelerations);
    schedule = struct();
    schedule.gains = zeros(2, 2, speedCount, accelerationCount);
    schedule.lyapunovMatrices = zeros(2, 2, speedCount, accelerationCount);
    schedule.lyapunovRates = zeros(2, 2, speedCount, accelerationCount);
    for pointIdx = 1:grid.numPoints
        point = grid.points(pointIdx);
        Pk = lyapunovAt(solution.P, point.alpha);
        PkRate = lyapunovAt(solution.P, point.alphaRate);
        schedule.gains(:, :, point.speedIdx, point.accelerationIdx) = Pk \ solution.Y(:, :, pointIdx);
        schedule.lyapunovMatrices(:, :, point.speedIdx, point.accelerationIdx) = Pk;
        schedule.lyapunovRates(:, :, point.speedIdx, point.accelerationIdx) = PkRate;
    end
end

function verification = verifyCertificate(grid, schedule, solution, synthesisCfg, tau)
% verifyCertificate: Re-evaluate the original, non-convexified certificate
% with the recovered gains. This checks the whole chain of substitutions and
% Young inequalities rather than the LMI that was actually solved: the
% Lyapunov derivative condition must be negative definite and the weighted
% H2 quantity trace(L' P L) must stay below mu at every grid point.
%
% Input:
%   grid: struct from buildDesignGrid
%   schedule: struct from recoverGainSchedule
%   solution: struct from solveSynthesisLmi
%   synthesisCfg: cfg.synthesis struct
%   tau: fixed positive scalar used by the synthesis
%
% Output:
%   verification: struct with certificateMargins, h2Values,
%       errorEigenvalues, and the overall certified flag
    errorWeight = double(synthesisCfg.errorWeight);
    lipschitzConstant = double(synthesisCfg.lipschitzConstant);
    certificateMargins = zeros(grid.numPoints, 1);
    h2Values = zeros(grid.numPoints, 1);
    errorEigenvalues = zeros(2, grid.numPoints);
    for pointIdx = 1:grid.numPoints
        point = grid.points(pointIdx);
        L = schedule.gains(:, :, point.speedIdx, point.accelerationIdx);
        Pk = schedule.lyapunovMatrices(:, :, point.speedIdx, point.accelerationIdx);
        PkRate = schedule.lyapunovRates(:, :, point.speedIdx, point.accelerationIdx);
        closedLoop = point.A - (L * point.C);
        certificate = (Pk * closedLoop) + (closedLoop.' * Pk) + PkRate + errorWeight + ...
            (tau .* (Pk * Pk)) + ((lipschitzConstant.^2 ./ tau) .* eye(2));
        certificateMargins(pointIdx) = max(real(eig((certificate + certificate.') ./ 2.0)));
        h2Values(pointIdx) = trace(L.' * Pk * L);
        errorEigenvalues(:, pointIdx) = eig(closedLoop);
    end

    verification = struct();
    verification.certificateMargins = certificateMargins;
    verification.h2Values = h2Values;
    verification.errorEigenvalues = errorEigenvalues;
    verification.certified = all(certificateMargins < 0) && all(h2Values < solution.mu) && ...
        all(real(errorEigenvalues(:)) < 0);
end

function P = lyapunovAt(basis, coordinates)
% lyapunovAt: Combine the three Lyapunov basis matrices with the supplied
% barycentric coordinates or coordinate rates.
%
% Input:
%   basis: [2 x 2 x 3] Lyapunov basis matrices
%   coordinates: [3 x 1] barycentric coordinates or their rates
%
% Output:
%   P: [2 x 2] combined matrix
    P = (coordinates(1) .* basis(:, :, 1)) + (coordinates(2) .* basis(:, :, 2)) + ...
        (coordinates(3) .* basis(:, :, 3));
end
