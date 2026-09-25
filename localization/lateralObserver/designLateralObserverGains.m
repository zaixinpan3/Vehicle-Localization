function design = designLateralObserverGains(cfg)
% designLateralObserverGains: Synthesize the three vertex gains of the
% polytopic LPV lateral-velocity observer by semidefinite programming. The
% error dynamics of the Luenberger observer are
%
%   edot = (A(rho) - L(rho) C(rho)) e + df - L(rho) v,
%
% with df the Lipschitz-bounded nonlinearity mismatch and v the measurement
% noise. The gain is affine in the barycentric coordinates alpha of rho on
% the scheduling triangle,
%
%   L(rho) = sum_i alpha_i L_i,
%
% so the online observer only blends the three vertex gains with the alpha
% that reproduces rho (scheduleLateralObserverGain). With the quadratic
% Lyapunov function V = e' P e, bounding 2 e' P df by Young's inequality
% with a fixed scalar tau, and weighting the performance output
% z = Q^(1/2) e, internal stability and the H2 bound ||T_{v->z}||^2 < mu
% follow from
%
%   P Acl + Acl' P + Q + tau P^2 + (gamma_f^2/tau) I < 0,
%   trace(L' P L) < mu.
%
% The change of variables Y_i = P L_i keeps the gain affine exactly,
% Y(rho) = sum_i alpha_i Y_i = P L(rho), because P is constant: a
% parameter-dependent P would make P(rho) L(rho) quadratic in alpha and
% leave no exact convexification of an affine gain. The Lyapunov
% derivative condition then has no Pdot term, so the certificate holds for
% any rate of change of the speed and no acceleration range enters the
% design. The tau P^2 term and trace(L' P L) <= trace(W) become the LMIs
%
%   [ He(P A - Y C) + Q + (gamma_f^2/tau) I,  sqrt(tau) P ;  sqrt(tau) P,  -I ] < 0,
%   [ W, Y' ; Y, P ] >= 0,   trace(W) <= mu.
%
% Because Y(rho) C(rho) is quadratic in alpha, the LMIs are imposed on a
% grid of speeds; the recovered vertex gains L_i = P \ Y_i are then checked
% against the original, non-convexified certificate above at every grid
% point through the same scheduling function the observer runs online.
%
% Requires YALMIP and an SDP solver on the MATLAB path.
%
% Input:
%   cfg: optional struct from lateralObserverConfig
%
% Output:
%   design: struct with the model, the scheduling polytope, the design grid,
%       the vertex gains, the Lyapunov matrix, the selected tau, the H2
%       bound mu, and the verification margins
    if nargin < 1 || isempty(cfg)
        cfg = lateralObserverConfig();
    end
    assert(exist("sdpvar", "file") == 2, ...
        "YALMIP must be on the MATLAB path before synthesizing the lateral observer gains.");
    assert(~isfield(cfg.synthesis, "etaFloor"), ...
        "cfg.synthesis.etaFloor is obsolete: the vertex-gain synthesis has no slack scalar. Remove it.");
    assert(~isfield(cfg.scheduling, "longitudinalAccelerationRange") && ...
        ~isfield(cfg.scheduling, "accelerationGridCount"), ...
        "cfg.scheduling.longitudinalAccelerationRange and accelerationGridCount are obsolete: " + ...
        "the constant Lyapunov matrix has no Pdot term. Remove them.");

    model = lateralBicycleModel(cfg.vehicle);
    polytope = buildSchedulingPolytope(cfg.scheduling.speedRange);
    grid = buildDesignGrid(model, polytope, cfg.scheduling);

    tauCandidates = sort(double(cfg.synthesis.tauCandidates(:).'));
    assert(~isempty(tauCandidates) && all(isfinite(tauCandidates)) && all(tauCandidates > 0), ...
        "cfg.synthesis.tauCandidates must contain positive finite scalars.");

    best = struct("found", false, "mu", inf);
    attempts = repmat(struct("tau", 0, "solved", false, "mu", NaN, "certified", false, "info", ""), ...
        numel(tauCandidates), 1);
    for tauIdx = 1:numel(tauCandidates)
        tau = tauCandidates(tauIdx);
        solution = solveSynthesisLmi(grid, cfg.synthesis, tau);
        attempts(tauIdx).tau = tau;
        attempts(tauIdx).solved = solution.solved;
        attempts(tauIdx).info = solution.info;
        if ~solution.solved
            continue;
        end
        candidate = recoverVertexGains(polytope, solution);
        verification = verifyCertificate(grid, candidate, solution.mu, cfg.synthesis, tau);
        attempts(tauIdx).mu = solution.mu;
        attempts(tauIdx).certified = verification.certified;
        if verification.certified && solution.mu < best.mu
            best = struct("found", true, "mu", solution.mu, "tau", tau, ...
                "candidate", candidate, "verification", verification);
        end
    end
    assert(best.found, ...
        "No tau candidate produced a certified lateral observer design. Widen cfg.synthesis.tauCandidates or relax the operating range.");

    design = struct();
    design.model = model;
    design.polytope = polytope;
    design.grid = grid;
    design.vertexGains = best.candidate.vertexGains;
    design.lyapunovMatrix = best.candidate.lyapunovMatrix;
    design.tau = best.tau;
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
% buildDesignGrid: Enumerate the speed grid points of the synthesis. Each
% point stores the physical scheduling parameter, its barycentric
% coordinates, and the model matrices there.
%
% Input:
%   model: struct from lateralBicycleModel
%   polytope: struct from buildSchedulingPolytope
%   schedulingCfg: cfg.scheduling struct with speedRange and speedGridCount
%
% Output:
%   grid: struct with speeds and a struct array of points
    speedRange = double(schedulingCfg.speedRange(:).');
    speedCount = round(double(schedulingCfg.speedGridCount));
    assert(speedCount >= 2, "The design grid needs at least two speeds.");

    speeds = linspace(speedRange(1), speedRange(2), speedCount);
    pointTemplate = struct("speed", 0, "rho", zeros(2, 1), "alpha", zeros(3, 1), ...
        "A", zeros(2, 2), "C", zeros(2, 2));
    points = repmat(pointTemplate, speedCount, 1);
    for pointIdx = 1:speedCount
        speed = speeds(pointIdx);
        rho = [speed; 1.0 ./ speed];
        [A, C] = evaluateLateralModel(model, rho);
        points(pointIdx).speed = speed;
        points(pointIdx).rho = rho;
        points(pointIdx).alpha = schedulingCoordinates(polytope, speed);
        points(pointIdx).A = A;
        points(pointIdx).C = C;
    end

    grid = struct();
    grid.speeds = speeds;
    grid.points = points;
    grid.numPoints = numel(points);
end

function solution = solveSynthesisLmi(grid, synthesisCfg, tau)
% solveSynthesisLmi: Solve the convexified synthesis for one fixed tau,
% minimizing the H2 bound mu. The decision variables are the Lyapunov
% matrix P, the three transformed vertex gains Y_i = P L_i, the auxiliary
% matrices W_k of every grid point, and mu. At grid point k the blend
% Y_k = sum_i alpha_i Y_i is linear in the decision variables because alpha
% is a known number there.
%
% Input:
%   grid: struct from buildDesignGrid
%   synthesisCfg: cfg.synthesis struct
%   tau: fixed positive scalar of the Lipschitz Young inequality
%
% Output:
%   solution: struct with solved, P, Y, mu, and solver info
    errorWeight = double(synthesisCfg.errorWeight);
    assert(isequal(size(errorWeight), [2, 2]) && norm(errorWeight - errorWeight.', "fro") <= 1.0e-12 && ...
        min(eig(errorWeight)) > 0, "cfg.synthesis.errorWeight must be a symmetric positive definite [2 x 2] matrix.");
    lipschitzConstant = double(synthesisCfg.lipschitzConstant);
    assert(isscalar(lipschitzConstant) && isfinite(lipschitzConstant) && lipschitzConstant >= 0, ...
        "cfg.synthesis.lipschitzConstant must be a nonnegative finite scalar.");
    pFloor = double(synthesisCfg.pFloor);
    pCeiling = double(synthesisCfg.pCeiling);
    strictnessEpsilon = double(synthesisCfg.strictnessEpsilon);

    P = sdpvar(2, 2, 'symmetric');
    Y = {sdpvar(2, 2, 'full'), sdpvar(2, 2, 'full'), sdpvar(2, 2, 'full')};
    mu = sdpvar(1, 1);
    W = cell(grid.numPoints, 1);

    constraints = P >= pFloor .* eye(2);
    if isfinite(pCeiling)
        constraints = [constraints, P <= pCeiling .* eye(2)];
    end

    for pointIdx = 1:grid.numPoints
        point = grid.points(pointIdx);
        W{pointIdx} = sdpvar(2, 2, 'symmetric');
        Yk = (point.alpha(1) .* Y{1}) + (point.alpha(2) .* Y{2}) + (point.alpha(3) .* Y{3});
        crossTerm = (P * point.A) - (Yk * point.C);
        Phi = crossTerm + crossTerm.' + errorWeight + ((lipschitzConstant.^2 ./ tau) .* eye(2));
        block = [Phi, sqrt(tau) .* P; sqrt(tau) .* P, -eye(2)];
        constraints = [constraints, block <= -strictnessEpsilon .* eye(4)]; %#ok<AGROW>
        constraints = [constraints, [W{pointIdx}, Yk.'; Yk, P] >= strictnessEpsilon .* eye(4)]; %#ok<AGROW>
        constraints = [constraints, trace(W{pointIdx}) <= mu]; %#ok<AGROW>
    end

    options = sdpsettings('solver', char(string(synthesisCfg.solver)), ...
        'verbose', double(synthesisCfg.verbose), 'dualize', double(synthesisCfg.dualize));
    diagnostics = optimize(constraints, mu, options);

    solution = struct();
    solution.solved = diagnostics.problem == 0;
    solution.info = string(diagnostics.info);
    solution.tau = tau;
    solution.P = zeros(2, 2);
    solution.Y = zeros(2, 2, 3);
    solution.mu = NaN;
    if ~solution.solved
        return;
    end
    solution.P = double(P);
    for vertexIdx = 1:3
        solution.Y(:, :, vertexIdx) = double(Y{vertexIdx});
    end
    solution.mu = double(mu);
end

function candidate = recoverVertexGains(polytope, solution)
% recoverVertexGains: Recover the vertex gains L_i = P \ Y_i and arrange
% them with the Lyapunov matrix and the polytope as the struct that
% scheduleLateralObserverGain evaluates online.
%
% Input:
%   polytope: struct from buildSchedulingPolytope
%   solution: struct from solveSynthesisLmi
%
% Output:
%   candidate: struct with polytope, vertexGains, and lyapunovMatrix
    candidate = struct();
    candidate.polytope = polytope;
    candidate.vertexGains = zeros(2, 2, 3);
    for vertexIdx = 1:3
        candidate.vertexGains(:, :, vertexIdx) = solution.P \ solution.Y(:, :, vertexIdx);
    end
    candidate.lyapunovMatrix = solution.P;
end

function verification = verifyCertificate(grid, candidate, mu, synthesisCfg, tau)
% verifyCertificate: Re-evaluate the original, non-convexified certificate
% with the recovered vertex gains. The gain of every grid point comes from
% scheduleLateralObserverGain, so the check covers the scheduling the
% observer runs online as well as the chain of substitutions and Young
% inequalities: the Lyapunov derivative condition must be negative
% definite and the weighted H2 quantity trace(L' P L) must stay below mu
% at every grid point.
%
% Input:
%   grid: struct from buildDesignGrid
%   candidate: struct from recoverVertexGains
%   mu: H2 bound returned by the solver
%   synthesisCfg: cfg.synthesis struct
%   tau: fixed positive scalar used by the synthesis
%
% Output:
%   verification: struct with certificateMargins, h2Values,
%       errorEigenvalues, and the overall certified flag
    errorWeight = double(synthesisCfg.errorWeight);
    lipschitzConstant = double(synthesisCfg.lipschitzConstant);
    P = candidate.lyapunovMatrix;
    certificateMargins = zeros(grid.numPoints, 1);
    h2Values = zeros(grid.numPoints, 1);
    errorEigenvalues = zeros(2, grid.numPoints);
    for pointIdx = 1:grid.numPoints
        point = grid.points(pointIdx);
        L = scheduleLateralObserverGain(candidate, point.speed);
        closedLoop = point.A - (L * point.C);
        certificate = (P * closedLoop) + (closedLoop.' * P) + errorWeight + ...
            (tau .* (P * P)) + ((lipschitzConstant.^2 ./ tau) .* eye(2));
        certificateMargins(pointIdx) = max(real(eig((certificate + certificate.') ./ 2.0)));
        h2Values(pointIdx) = trace(L.' * P * L);
        errorEigenvalues(:, pointIdx) = eig(closedLoop);
    end

    verification = struct();
    verification.certificateMargins = certificateMargins;
    verification.h2Values = h2Values;
    verification.errorEigenvalues = errorEigenvalues;
    verification.certified = all(certificateMargins < 0) && all(h2Values < mu) && ...
        all(real(errorEigenvalues(:)) < 0);
end
