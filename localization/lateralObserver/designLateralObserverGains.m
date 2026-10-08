function design = designLateralObserverGains(cfg)
% designLateralObserverGains: Synthesize the three vertex gains of the LPV
% lateral-velocity observer as the gains with the smallest input-to-state
% stability (ISS) gain from axle-force model mismatch and measurement error
% to the estimation error, at a prescribed exponential decay rate, by
% semidefinite programming. The true vehicle obeys the kinematics exactly
% but its axle forces differ from the linear tire model, so with
% w = [dFyf; dFyr] / m the force error per unit mass and v the measurement
% error, the error dynamics of the Luenberger observer are
%
%   edot = (A(rho) - L(rho) C(rho)) e + df + (E - L(rho) F) w - L(rho) v,
%
% with df the Lipschitz-bounded nonlinearity mismatch. Wrong cornering
% stiffness, mass, steering zero, or tire saturation all enter through w.
% Without w the model is taken as exact, measurements only add noise, and
% the best gain is zero whenever the open-loop model already decays fast
% enough; w is what makes output injection worth its noise. Both
% disturbances are normalized by their configured scales,
% d = [Ww^(-1) w; Wv^(-1) v], so that Bd(rho) = [(E - L F) Ww, -L Wv].
%
% Both the Lyapunov matrix and the gain are affine in the barycentric
% coordinates alpha of rho on the scheduling triangle,
%
%   P(rho) = sum_i alpha_i P_i,    L(rho) = sum_i alpha_i L_i,
%
% so the online observer only blends the three vertex gains with the alpha
% that reproduces rho (lateralObserverSupport.scheduleLateralObserverGain). With V = e' P(rho) e,
% bounding 2 e' P df by Young's inequality with a fixed scalar tau, and the
% decay rate kappa, the certificate
%
%   [ Psi,  P Bd ;  Bd' P,  -gamma^2 I ] < 0,
%   Psi = P Acl + Acl' P + Pdot + 2 kappa P + tau P^2 + (gamma_f^2/tau) I
%
% gives Vdot <= -2 kappa V + gamma^2 d' d. The normalization P(rho) >= I
% turns that into an error bound: the disturbance-free error decays at
% least as exp(-kappa t), and a disturbance of normalized magnitude |d|
% leaves |e| <= gamma |d| / sqrt(2 kappa). The products P(rho) L(rho) are
% quadratic in alpha, so the classical change of variables Y = P L would
% not keep the gain affine. Instead a constant slack matrix X defines
% Y_i = X L_i, so Y(rho) = sum_i alpha_i Y_i equals X L(rho) exactly, and
% Finsler's lemma with the multiplier [X; epsilon X; 0] on the identity
% [Acl, -I, Bd] [e; edot_nominal; d] = 0 gives, with R = X A - Y C and
% S = [(X E - Y F) Ww, -Y Wv] = X Bd,
%
%   [ Pdot + 2 kappa P + (gamma_f^2/tau) I + He(R),  P - X + epsilon R',  S,           sqrt(tau) P ;
%     *,                                            -epsilon (X + X'),   epsilon S,   0 ;
%     *,                                            *,                   -gamma^2 I,  0 ;
%     *,                                            *,                   *,           -I ] < 0,
%
% which is linear in (P_i, X, Y_i, gamma^2). The congruence with the
% null-space basis [I, 0; Acl, Bd; 0, I] cancels every term in X and
% returns the certificate, so the LMI is sufficient; the multiplier
% structure makes it conservative, so the scalar epsilon is searched over
% candidates together with tau and the pair with the smallest gamma is
% kept. The worst-case gamma is set by a few scheduling points, which
% leaves the gains elsewhere undetermined, so a second program fixes them:
% with gamma relaxed by cfg.synthesis.issGainRelaxation it minimizes the sum
% of the bounds t_i on the transformed vertex gains through
% [t_i I, Y_i'; Y_i, t_i I] >= 0 under the normalization X + X' >= 2 I,
% which gives ||L_i|| <= ||Y_i|| <= t_i. The result is the smallest vertex
% gains that still achieve the relaxed optimal ISS gain. The LMIs are
% imposed on a grid of scheduling points and longitudinal accelerations
% (Pdot is linear in the acceleration, so its extremes suffice), and the
% recovered vertex gains L_i = X \ Y_i are checked against the certificate
% through the scheduling function the observer runs online.
%
% Requires YALMIP and an SDP solver on the MATLAB path.
%
% Input:
%   cfg: optional struct from lateralObserverConfig
%
% Output:
%   design: struct with the model, the scheduling polytope, the design grid,
%       the vertex gains, the Lyapunov basis, the slack matrix and its
%       scale, the selected tau, the decay rate, the disturbance scales, the
%       optimal and the achieved ISS gain with its mismatch and measurement
%       parts, the vertex-gain bounds, and the verification margins
    if nargin < 1 || isempty(cfg)
        cfg = lateralObserverConfig();
    end
    assert(exist("sdpvar", "file") == 2, ...
        "YALMIP must be on the MATLAB path before synthesizing the lateral observer gains.");
    assert(~isfield(cfg.synthesis, "etaFloor") && ~isfield(cfg.synthesis, "errorWeight") && ...
        ~isfield(cfg.synthesis, "pFloor"), ...
        "cfg.synthesis.etaFloor, errorWeight, and pFloor are obsolete: the ISS synthesis normalizes " + ...
        "P >= I and uses decayRate, axleForceMismatchScale, and measurementErrorScale. Remove them.");

    model = lateralObserverSupport.lateralBicycleModel(cfg.vehicle);
    polytope = lateralObserverSupport.buildSchedulingPolytope(cfg.scheduling.speedRange);
    grid = buildDesignGrid(model, polytope, cfg.scheduling);
    scales = disturbanceScales(cfg.synthesis);

    decayRate = double(cfg.synthesis.decayRate);
    assert(isscalar(decayRate) && isfinite(decayRate) && decayRate > 0, ...
        "cfg.synthesis.decayRate must be a positive finite scalar.");
    tauCandidates = sort(double(cfg.synthesis.tauCandidates(:).'));
    assert(~isempty(tauCandidates) && all(isfinite(tauCandidates)) && all(tauCandidates > 0), ...
        "cfg.synthesis.tauCandidates must contain positive finite scalars.");
    slackScaleCandidates = sort(double(cfg.synthesis.slackScaleCandidates(:).'));
    assert(~isempty(slackScaleCandidates) && all(isfinite(slackScaleCandidates)) && all(slackScaleCandidates > 0), ...
        "cfg.synthesis.slackScaleCandidates must contain positive finite scalars.");

    relaxation = double(cfg.synthesis.issGainRelaxation);
    assert(isscalar(relaxation) && isfinite(relaxation) && relaxation > 0, ...
        "cfg.synthesis.issGainRelaxation must be a positive finite scalar.");

    best = struct("found", false, "issGainBound", inf);
    attemptTemplate = struct("tau", 0, "slackScale", 0, "solved", false, "issGainBound", NaN, ...
        "info", "");
    attempts = repmat(attemptTemplate, numel(tauCandidates) .* numel(slackScaleCandidates), 1);
    attemptIdx = 0;
    for tau = tauCandidates
        for slackScale = slackScaleCandidates
            attemptIdx = attemptIdx + 1;
            solution = solveMinimumIssGain(grid, model, scales, cfg.synthesis, tau, slackScale);
            attempts(attemptIdx).tau = tau;
            attempts(attemptIdx).slackScale = slackScale;
            attempts(attemptIdx).solved = solution.solved;
            attempts(attemptIdx).info = solution.info;
            if ~solution.solved
                continue;
            end
            attempts(attemptIdx).issGainBound = solution.issGainBound;
            % A later candidate must improve the gain by more than solver
            % noise, so ties resolve to the first candidate reproducibly
            if solution.issGainBound < best.issGainBound .* (1.0 - 1.0e-6)
                best = struct("found", true, "issGainBound", solution.issGainBound, "tau", tau, ...
                    "slackScale", slackScale);
            end
        end
    end
    assert(best.found, ...
        "No (tau, slackScale) candidate solved the lateral observer ISS synthesis. " + ...
        "Lower cfg.synthesis.decayRate, widen the candidate lists, or relax the operating range.");

    issGainLimit = (1.0 + relaxation) .* best.issGainBound;
    solution = solveMinimumGain(grid, model, scales, cfg.synthesis, best.tau, best.slackScale, issGainLimit);
    assert(solution.solved, "The minimum-gain program failed at the relaxed ISS gain: %s", solution.info);
    candidate = recoverVertexGains(polytope, solution);
    verification = verifyCertificate(grid, model, scales, candidate, cfg.synthesis, best.tau);
    assert(verification.certified && verification.issGain <= issGainLimit .* (1.0 + 1.0e-6), ...
        "The recovered lateral observer gains do not satisfy the ISS certificate.");

    design = struct();
    design.model = model;
    design.polytope = polytope;
    design.grid = grid;
    design.vertexGains = candidate.vertexGains;
    design.lyapunovBasis = candidate.lyapunovBasis;
    design.slackMatrix = candidate.slackMatrix;
    design.slackScale = best.slackScale;
    design.tau = best.tau;
    design.decayRate = decayRate;
    design.axleForceMismatchScale = scales.mismatch;
    design.measurementErrorScale = scales.measurement;
    design.vertexGainBounds = solution.vertexGainBounds;
    design.maxVertexGainNorm = max([norm(design.vertexGains(:, :, 1)), norm(design.vertexGains(:, :, 2)), ...
        norm(design.vertexGains(:, :, 3))]);
    design.optimalIssGain = best.issGainBound;
    design.issGainLimit = issGainLimit;
    design.issGain = verification.issGain;
    design.mismatchIssGain = verification.mismatchIssGain;
    design.measurementIssGain = verification.measurementIssGain;
    design.errorBound = verification.issGain ./ sqrt(2.0 .* decayRate);
    design.lipschitzConstant = double(cfg.synthesis.lipschitzConstant);
    design.certificateMargins = verification.certificateMargins;
    design.errorEigenvalues = verification.errorEigenvalues;
    design.maxCertificateMargin = max(verification.certificateMargins);
    design.maxErrorEigenvalueRealPart = max(real(verification.errorEigenvalues(:)));
    design.minLyapunovEigenvalue = verification.minLyapunovEigenvalue;
    design.certified = verification.certified;
    design.attempts = attempts;
    design.cfg = cfg;
end

function scales = disturbanceScales(synthesisCfg)
% disturbanceScales: Read the normalization of the two disturbance channels
% of the ISS certificate.
%
% Input:
%   synthesisCfg: cfg.synthesis struct
%
% Output:
%   scales: struct with mismatch, the [2 x 1] front and rear axle-force
%       error per unit mass in meters per second squared, and measurement,
%       the [2 x 1] lateral-acceleration and yaw-rate error
    assert(isfield(synthesisCfg, "axleForceMismatchScale") && isfield(synthesisCfg, "measurementErrorScale"), ...
        "cfg.synthesis must define axleForceMismatchScale and measurementErrorScale. " + ...
        "Use the current lateralObserverConfig.");
    scales = struct();
    scales.mismatch = double(synthesisCfg.axleForceMismatchScale(:));
    scales.measurement = double(synthesisCfg.measurementErrorScale(:));
    assert(numel(scales.mismatch) == 2 && all(isfinite(scales.mismatch)) && all(scales.mismatch > 0), ...
        "cfg.synthesis.axleForceMismatchScale must hold two positive finite scales [front; rear].");
    assert(numel(scales.measurement) == 2 && all(isfinite(scales.measurement)) && all(scales.measurement > 0), ...
        "cfg.synthesis.measurementErrorScale must hold two positive finite scales [ay; r].");
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
%   model: struct from lateralObserverSupport.lateralBicycleModel
%   polytope: struct from lateralObserverSupport.buildSchedulingPolytope
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
        "alpha", zeros(3, 1), "alphaRate", zeros(3, 1), "A", zeros(2, 2), "C", zeros(2, 2));
    points = repmat(pointTemplate, speedCount .* accelerationCount, 1);
    pointIdx = 0;
    for speed = speeds
        for acceleration = accelerations
            pointIdx = pointIdx + 1;
            rho = [speed; 1.0 ./ speed];
            [alpha, alphaRate] = lateralObserverSupport.schedulingCoordinates(polytope, speed, acceleration);
            [A, C] = lateralObserverSupport.evaluateLateralModel(model, rho);
            points(pointIdx).speed = speed;
            points(pointIdx).acceleration = acceleration;
            points(pointIdx).rho = rho;
            points(pointIdx).alpha = alpha;
            points(pointIdx).alphaRate = alphaRate;
            points(pointIdx).A = A;
            points(pointIdx).C = C;
        end
    end

    grid = struct();
    grid.speeds = speeds;
    grid.accelerations = accelerations;
    grid.points = points;
    grid.numPoints = numel(points);
end

function solution = solveMinimumIssGain(grid, model, scales, synthesisCfg, tau, slackScale)
% solveMinimumIssGain: Solve the Finsler-convexified ISS synthesis for one
% fixed tau and multiplier scale, minimizing the squared ISS gain from the
% normalized mismatch and measurement error.
%
% Input:
%   grid: struct from buildDesignGrid
%   model: struct from lateralObserverSupport.lateralBicycleModel with the mismatch channel E, F
%   scales: struct from disturbanceScales
%   synthesisCfg: cfg.synthesis struct
%   tau: fixed positive scalar of the Lipschitz Young inequality
%   slackScale: fixed positive scalar epsilon of the second multiplier
%
% Output:
%   solution: struct with solved, issGainBound, and solver info
    variables = synthesisVariables();
    issGainSquared = sdpvar(1, 1);
    constraints = certificateConstraints(variables, issGainSquared, grid, model, scales, ...
        synthesisCfg, tau, slackScale);
    diagnostics = optimize(constraints, issGainSquared, solverOptions(synthesisCfg));

    solution = struct();
    solution.solved = diagnostics.problem == 0;
    solution.info = string(diagnostics.info);
    solution.issGainBound = NaN;
    if solution.solved
        solution.issGainBound = sqrt(max(double(issGainSquared), 0.0));
    end
end

function solution = solveMinimumGain(grid, model, scales, synthesisCfg, tau, slackScale, issGainLimit)
% solveMinimumGain: With the ISS gain fixed at issGainLimit, minimize the
% sum of the bounds t_i on the transformed vertex gains. The normalization
% X + X' >= 2 I gives ||L_i|| <= ||Y_i|| / lambda_min((X + X')/2) <=
% ||Y_i|| <= t_i, so the program returns the smallest vertex gains that
% achieve the fixed ISS gain. Summing the bounds keeps every vertex gain
% small, not only the largest one.
%
% Input:
%   grid: struct from buildDesignGrid
%   model: struct from lateralObserverSupport.lateralBicycleModel with the mismatch channel E, F
%   scales: struct from disturbanceScales
%   synthesisCfg: cfg.synthesis struct
%   tau: fixed positive scalar of the Lipschitz Young inequality
%   slackScale: fixed positive scalar epsilon of the second multiplier
%   issGainLimit: ISS gain the certificate must achieve
%
% Output:
%   solution: struct with solved, P, X, Y, vertexGainBounds, and solver info
    variables = synthesisVariables();
    vertexGainBounds = sdpvar(3, 1);
    constraints = certificateConstraints(variables, issGainLimit.^2, grid, model, scales, ...
        synthesisCfg, tau, slackScale);
    constraints = [constraints, (variables.X + variables.X.') >= 2.0 .* eye(2)];
    for vertexIdx = 1:3
        constraints = [constraints, [vertexGainBounds(vertexIdx) .* eye(2), variables.Y{vertexIdx}.'; ...
            variables.Y{vertexIdx}, vertexGainBounds(vertexIdx) .* eye(2)] >= 0]; %#ok<AGROW>
    end
    diagnostics = optimize(constraints, sum(vertexGainBounds), solverOptions(synthesisCfg));

    solution = struct();
    solution.solved = diagnostics.problem == 0;
    solution.info = string(diagnostics.info);
    solution.P = zeros(2, 2, 3);
    solution.Y = zeros(2, 2, 3);
    solution.X = zeros(2, 2);
    solution.vertexGainBounds = NaN(3, 1);
    if ~solution.solved
        return;
    end
    for vertexIdx = 1:3
        solution.P(:, :, vertexIdx) = double(variables.P{vertexIdx});
        solution.Y(:, :, vertexIdx) = double(variables.Y{vertexIdx});
    end
    solution.X = double(variables.X);
    solution.vertexGainBounds = double(vertexGainBounds);
end

function variables = synthesisVariables()
% synthesisVariables: Declare the three Lyapunov basis matrices P_i, the
% three transformed vertex gains Y_i = X L_i, and the common slack matrix X.
%
% Input:
%   none
%
% Output:
%   variables: struct with the cells P and Y and the matrix X
    variables = struct();
    variables.P = {sdpvar(2, 2, 'symmetric'), sdpvar(2, 2, 'symmetric'), sdpvar(2, 2, 'symmetric')};
    variables.Y = {sdpvar(2, 2, 'full'), sdpvar(2, 2, 'full'), sdpvar(2, 2, 'full')};
    variables.X = sdpvar(2, 2, 'full');
end

function options = solverOptions(synthesisCfg)
% solverOptions: Build the YALMIP solver settings of the synthesis.
%
% Input:
%   synthesisCfg: cfg.synthesis struct
%
% Output:
%   options: sdpsettings struct
    options = sdpsettings('solver', char(string(synthesisCfg.solver)), ...
        'verbose', double(synthesisCfg.verbose), 'dualize', double(synthesisCfg.dualize));
end

function constraints = certificateConstraints(variables, issGainSquared, grid, model, scales, ...
        synthesisCfg, tau, slackScale)
% certificateConstraints: Assemble the normalization P_i >= I and the
% Finsler-convexified ISS certificate at every grid point. At grid point k
% the blends P_k, Pdot_k, and Y_k are linear in the decision variables
% because alpha and alphaRate are known numbers there.
%
% Input:
%   variables: struct from synthesisVariables
%   issGainSquared: squared ISS gain, a decision variable or a number
%   grid: struct from buildDesignGrid
%   model: struct from lateralObserverSupport.lateralBicycleModel with the mismatch channel E, F
%   scales: struct from disturbanceScales
%   synthesisCfg: cfg.synthesis struct
%   tau: fixed positive scalar of the Lipschitz Young inequality
%   slackScale: fixed positive scalar epsilon of the second multiplier
%
% Output:
%   constraints: YALMIP constraint set
    lipschitzConstant = double(synthesisCfg.lipschitzConstant);
    assert(isscalar(lipschitzConstant) && isfinite(lipschitzConstant) && lipschitzConstant >= 0, ...
        "cfg.synthesis.lipschitzConstant must be a nonnegative finite scalar.");
    decayRate = double(synthesisCfg.decayRate);
    pCeiling = double(synthesisCfg.pCeiling);
    strictnessEpsilon = double(synthesisCfg.strictnessEpsilon);
    mismatchWeight = diag(scales.mismatch);
    measurementWeight = diag(scales.measurement);
    P = variables.P;
    Y = variables.Y;
    X = variables.X;

    constraints = [];
    for vertexIdx = 1:3
        constraints = [constraints, P{vertexIdx} >= eye(2)]; %#ok<AGROW>
        if isfinite(pCeiling)
            constraints = [constraints, P{vertexIdx} <= pCeiling .* eye(2)]; %#ok<AGROW>
        end
    end

    for pointIdx = 1:grid.numPoints
        point = grid.points(pointIdx);
        Pk = blendVertexCells(P, point.alpha);
        PkRate = blendVertexCells(P, point.alphaRate);
        Yk = blendVertexCells(Y, point.alpha);
        crossTerm = (X * point.A) - (Yk * point.C);
        Phi = crossTerm + crossTerm.' + PkRate + (2.0 .* decayRate .* Pk) + ...
            ((lipschitzConstant.^2 ./ tau) .* eye(2));
        coupling = Pk - X + (slackScale .* crossTerm.');
        disturbance = [((X * model.E) - (Yk * model.F)) * mismatchWeight, -Yk * measurementWeight];
        block = [Phi, coupling, disturbance, sqrt(tau) .* Pk; ...
            coupling.', -slackScale .* (X + X.'), slackScale .* disturbance, zeros(2, 2); ...
            disturbance.', slackScale .* disturbance.', -issGainSquared .* eye(4), zeros(4, 2); ...
            sqrt(tau) .* Pk, zeros(2, 2), zeros(2, 4), -eye(2)];
        constraints = [constraints, block <= -strictnessEpsilon .* eye(10)]; %#ok<AGROW>
    end
end

function candidate = recoverVertexGains(polytope, solution)
% recoverVertexGains: Recover the vertex gains L_i = X \ Y_i and arrange
% them with the Lyapunov basis, the slack matrix, and the polytope as the
% struct that lateralObserverSupport.scheduleLateralObserverGain evaluates online.
%
% Input:
%   polytope: struct from lateralObserverSupport.buildSchedulingPolytope
%   solution: struct from solveMinimumGain
%
% Output:
%   candidate: struct with polytope, vertexGains, lyapunovBasis, slackMatrix
    candidate = struct();
    candidate.polytope = polytope;
    candidate.vertexGains = zeros(2, 2, 3);
    for vertexIdx = 1:3
        candidate.vertexGains(:, :, vertexIdx) = solution.X \ solution.Y(:, :, vertexIdx);
    end
    candidate.lyapunovBasis = solution.P;
    candidate.slackMatrix = solution.X;
end

function verification = verifyCertificate(grid, model, scales, candidate, synthesisCfg, tau)
% verifyCertificate: Re-evaluate the original, non-convexified certificate
% with the recovered vertex gains and compute the resulting ISS gain. The
% gain and the Lyapunov matrix of every grid point come from
% lateralObserverSupport.scheduleLateralObserverGain, so the check covers the scheduling the
% observer runs online as well as the Finsler and Young steps: Psi must be
% negative definite, P must respect its normalization, and the
% disturbance-free error matrix must be stable at every grid point. The ISS
% gain is gamma^2 = max lambda(Bd' P (-Psi)^(-1) P Bd) over the grid, and
% the same expression restricted to the mismatch or the measurement columns
% of Bd gives the two partial gains.
%
% Input:
%   grid: struct from buildDesignGrid
%   model: struct from lateralObserverSupport.lateralBicycleModel
%   scales: struct from disturbanceScales
%   candidate: struct from recoverVertexGains
%   synthesisCfg: cfg.synthesis struct
%   tau: fixed positive scalar used by the synthesis
%
% Output:
%   verification: struct with certificateMargins, errorEigenvalues, the ISS
%       gain and its mismatch and measurement parts, the smallest Lyapunov
%       eigenvalue, and the overall certified flag
    lipschitzConstant = double(synthesisCfg.lipschitzConstant);
    decayRate = double(synthesisCfg.decayRate);
    mismatchWeight = diag(scales.mismatch);
    measurementWeight = diag(scales.measurement);
    certificateMargins = zeros(grid.numPoints, 1);
    errorEigenvalues = zeros(2, grid.numPoints);
    gainSquared = zeros(1, 3);
    minLyapunovEigenvalue = inf;
    for pointIdx = 1:grid.numPoints
        point = grid.points(pointIdx);
        [L, Pk] = lateralObserverSupport.scheduleLateralObserverGain(candidate, point.speed);
        PkRate = blendVertexMatrices(candidate.lyapunovBasis, point.alphaRate);
        closedLoop = point.A - (L * point.C);
        certificate = (Pk * closedLoop) + (closedLoop.' * Pk) + PkRate + (2.0 .* decayRate .* Pk) + ...
            (tau .* (Pk * Pk)) + ((lipschitzConstant.^2 ./ tau) .* eye(2));
        certificate = (certificate + certificate.') ./ 2.0;
        certificateMargins(pointIdx) = max(eig(certificate));
        errorEigenvalues(:, pointIdx) = eig(closedLoop);
        minLyapunovEigenvalue = min(minLyapunovEigenvalue, min(eig((Pk + Pk.') ./ 2.0)));
        if certificateMargins(pointIdx) < 0
            mismatchInput = (model.E - (L * model.F)) * mismatchWeight;
            measurementInput = -L * measurementWeight;
            inputs = {[mismatchInput, measurementInput], mismatchInput, measurementInput};
            for inputIdx = 1:3
                weighted = Pk * inputs{inputIdx};
                term = weighted.' * ((-certificate) \ weighted);
                gainSquared(inputIdx) = max(gainSquared(inputIdx), max(eig((term + term.') ./ 2.0)));
            end
        end
    end

    verification = struct();
    verification.certificateMargins = certificateMargins;
    verification.errorEigenvalues = errorEigenvalues;
    verification.issGain = sqrt(gainSquared(1));
    verification.mismatchIssGain = sqrt(gainSquared(2));
    verification.measurementIssGain = sqrt(gainSquared(3));
    verification.minLyapunovEigenvalue = minLyapunovEigenvalue;
    verification.certified = all(certificateMargins < 0) && all(real(errorEigenvalues(:)) < 0) && ...
        minLyapunovEigenvalue >= 1.0 - 1.0e-6;
end

function blended = blendVertexCells(vertexMatrices, coordinates)
% blendVertexCells: Combine three decision-variable matrices with the
% supplied barycentric coordinates or coordinate rates.
%
% Input:
%   vertexMatrices: {1 x 3} cell of [2 x 2] sdpvar matrices
%   coordinates: [3 x 1] barycentric coordinates or their rates
%
% Output:
%   blended: [2 x 2] combined sdpvar matrix
    blended = (coordinates(1) .* vertexMatrices{1}) + (coordinates(2) .* vertexMatrices{2}) + ...
        (coordinates(3) .* vertexMatrices{3});
end

function blended = blendVertexMatrices(basis, coordinates)
% blendVertexMatrices: Combine three numeric vertex matrices with the
% supplied barycentric coordinates or coordinate rates.
%
% Input:
%   basis: [2 x 2 x 3] vertex matrices
%   coordinates: [3 x 1] barycentric coordinates or their rates
%
% Output:
%   blended: [2 x 2] combined matrix
    blended = (coordinates(1) .* basis(:, :, 1)) + (coordinates(2) .* basis(:, :, 2)) + ...
        (coordinates(3) .* basis(:, :, 3));
end
