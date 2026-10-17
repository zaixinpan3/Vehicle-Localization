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
% P(rho) and L(rho) blend three vertex matrices. Only the acceleration
% innovation column is restricted: L(:,1) = g0 + Vx*g1. Equivalently,
% Y3(:,1) = b/(a+b)*Y1(:,1) + a/(a+b)*Y2(:,1), with Y_i = X*L_i.
% On the physical curve rho1*rho2 = 1 this makes A-L*C affine. At the
% virtual third vertex the Finsler term must be corrected by
% -(b-a)^2/(a+b)^2 * H*c', H = (Y2(:,1)-Y1(:,1))/(b-a), c'=C2(1,:).
% This affine extension equals the physical error model at every speed.
%
% The 10-by-10 dilated LMI uses a common, nonsymmetric slack X, a fixed
% multiplier scale epsilon and Young scalar tau, but independent P_i >= I:
% [ He(Rtilde)+Pdot+2*kappa*P+(gamma_f^2/tau)*I, P-X+epsilon*Rtilde', S, sqrt(tau)*P;
%   *, -epsilon*(X+X'), epsilon*S, 0;
%   *, *, -gamma^2*I, 0; *, *, *, -I ] < 0,
% where S = [(X*E-Y*F)*Ww, -Y*Wv]. The scheduling triangle and four
% derivative vertices enclose every admissible (Vx,dVx/dt) pair. The
% complete dilated matrix is jointly affine in position and derivative,
% so its 3*4 vertex LMIs certify the entire continuous speed interval.
% A null-space congruence recovers the original ISS inequality; see
% research/lateral_affine_extension_20261016/derivation.tex for the proof.
%
% We search fixed (tau,epsilon) candidates, minimize gamma, then minimize
% transformed vertex-gain norm bounds within issGainRelaxation. The result
% reports the certified interval bound, never a smaller sampled maximum.
% Physical grid checks are supplementary diagnostics, not synthesis
% constraints. The certificate covers the continuous LPV error subsystem,
% not discretization, speed measurement mismatch or the hybrid master state.
%
% Requires YALMIP and an SDP solver on the MATLAB path.
%
% Input:
%   cfg: optional struct from lateralObserverConfig
%
% Output:
%   design: struct with the model, the scheduling polytope, the diagnostic grid,
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
    domain = lateralObserverSupport.buildCertificateDomain(model, polytope, ...
        cfg.scheduling.longitudinalAccelerationRange);
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
            solution = solveMinimumIssGain(domain, model, scales, cfg.synthesis, tau, slackScale);
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
    solution = solveMinimumGain(domain, model, scales, cfg.synthesis, best.tau, best.slackScale, issGainLimit);
    assert(solution.solved, "The minimum-gain program failed at the relaxed ISS gain: %s", solution.info);
    candidate = recoverVertexGains(polytope, solution);
    diagnostics = verifyPhysicalGrid(grid, model, scales, candidate, cfg.synthesis, best.tau);


    design = struct();
    design.model = model;
    design.polytope = polytope;
    design.grid = grid;
    design.certificateDomain = domain;
    design.certificateMethod = "physical-curve-affine-extension-v1";
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
    design.issGain = issGainLimit;
    design.errorBound = issGainLimit ./ sqrt(2.0 .* decayRate);
    design.lipschitzConstant = double(cfg.synthesis.lipschitzConstant);
    design.certificateMargins = diagnostics.certificateMargins;
    design.errorEigenvalues = diagnostics.errorEigenvalues;
    design.maxCertificateMargin = max(diagnostics.certificateMargins);
    design.maxErrorEigenvalueRealPart = max(real(diagnostics.errorEigenvalues(:)));
    design.minLyapunovEigenvalue = diagnostics.minLyapunovEigenvalue;
    design.gridDiagnostics = diagnostics;
    design.cfg = cfg;
    certificate = verifyLateralObserverCertificate(design);
    assert(certificate.certified, "VehicleLocalization:InvalidLateralCertificate", ...
        "The recovered gains failed continuous vertex certification (max eigenvalue %.3g).", ...
        max(certificate.vertexMargins(:)));
    assert(diagnostics.certified && diagnostics.issGain <= issGainLimit * (1 + 1e-6), ...
        "The physical diagnostic grid contradicts the continuous certificate.");
    design.continuousCertificate = certificate;
    design.mismatchIssGain = certificate.mismatchIssGain;
    design.measurementIssGain = certificate.measurementIssGain;
    design.certified = certificate.certified;
    design.attempts = attempts;
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
% buildDesignGrid: Enumerate supplementary physical diagnostic points. Each point stores the physical scheduling parameter, its
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
        "The diagnostic grid needs at least two speeds and one speed derivative.");
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

function solution = solveMinimumIssGain(domain, model, scales, synthesisCfg, tau, slackScale)
% solveMinimumIssGain: Solve the Finsler-convexified ISS synthesis for one
% fixed tau and multiplier scale, minimizing the squared ISS gain from the
% normalized mismatch and measurement error.
%
% Input:
%   domain: position/rate certificate vertices
%   model: struct from lateralObserverSupport.lateralBicycleModel with the mismatch channel E, F
%   scales: struct from disturbanceScales
%   synthesisCfg: cfg.synthesis struct
%   tau: fixed positive scalar of the Lipschitz Young inequality
%   slackScale: fixed positive scalar epsilon of the second multiplier
%
% Output:
%   solution: struct with solved, issGainBound, and solver info
    variables = synthesisVariables(domain);
    issGainSquared = sdpvar(1, 1);
    constraints = certificateConstraints(variables, issGainSquared, domain, model, scales, ...
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

function solution = solveMinimumGain(domain, model, scales, synthesisCfg, tau, slackScale, issGainLimit)
% solveMinimumGain: With the ISS gain fixed at issGainLimit, minimize the
% sum of the bounds t_i on the transformed vertex gains. The normalization
% X + X' >= 2 I gives ||L_i|| <= ||Y_i|| / lambda_min((X + X')/2) <=
% ||Y_i|| <= t_i, so the program returns the smallest vertex gains that
% achieve the fixed ISS gain. Summing the bounds keeps every vertex gain
% small, not only the largest one.
%
% Input:
%   domain: position/rate certificate vertices
%   model: struct from lateralObserverSupport.lateralBicycleModel with the mismatch channel E, F
%   scales: struct from disturbanceScales
%   synthesisCfg: cfg.synthesis struct
%   tau: fixed positive scalar of the Lipschitz Young inequality
%   slackScale: fixed positive scalar epsilon of the second multiplier
%   issGainLimit: ISS gain the certificate must achieve
%
% Output:
%   solution: struct with solved, P, X, Y, vertexGainBounds, and solver info
    variables = synthesisVariables(domain);
    vertexGainBounds = sdpvar(3, 1);
    constraints = certificateConstraints(variables, issGainLimit.^2, domain, model, scales, ...
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

function variables = synthesisVariables(domain)
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
    variables.Y = {sdpvar(2, 2, 'full'), sdpvar(2, 2, 'full'), []};
    % Eliminate the two constrained variables exactly, rather than relying
    % on an equality constraint with a solver feasibility tolerance.
    weights = domain.firstColumnWeights;
    variables.Y{3} = [weights(1)*variables.Y{1}(:,1) + ...
        weights(2)*variables.Y{2}(:,1), sdpvar(2, 1, 'full')];
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

function constraints = certificateConstraints(variables, issGainSquared, domain, model, scales, ...
        synthesisCfg, tau, slackScale)
% certificateConstraints: Twelve jointly affine position/rate vertex LMIs.
% The virtual vertex uses the affine extension, not the raw product Y_i*C_i.
    lipschitzConstant = double(synthesisCfg.lipschitzConstant);
    assert(isscalar(lipschitzConstant) && isfinite(lipschitzConstant) && lipschitzConstant >= 0, ...
        "cfg.synthesis.lipschitzConstant must be a nonnegative finite scalar.");
    decayRate = double(synthesisCfg.decayRate);
    pCeiling = double(synthesisCfg.pCeiling);
    strictnessEpsilon = double(synthesisCfg.strictnessEpsilon);
    assert(isscalar(strictnessEpsilon) && isfinite(strictnessEpsilon) && strictnessEpsilon > 0, ...
        "cfg.synthesis.strictnessEpsilon must be positive and finite.");
    assert(isscalar(pCeiling) && pCeiling >= 1 + strictnessEpsilon, ...
        "cfg.synthesis.pCeiling must allow the normalized Lyapunov matrices.");
    mismatchWeight = diag(scales.mismatch);
    measurementWeight = diag(scales.measurement);
    P = variables.P;
    Y = variables.Y;
    X = variables.X;

    constraints = [issGainSquared >= 0];
    for vertexIdx = 1:3
        constraints = [constraints, P{vertexIdx} >= (1 + strictnessEpsilon) .* eye(2)]; %#ok<AGROW>
        if isfinite(pCeiling)
            constraints = [constraints, P{vertexIdx} <= pCeiling .* eye(2)]; %#ok<AGROW>
        end
    end

    % The speed difference is read from the original triangle coordinates.
    H = (Y{2}(:,1) - Y{1}(:,1)) ./ domain.speedSpan;
    for vertexIdx = 1:3
        Pk = P{vertexIdx};
        Yk = Y{vertexIdx};
        crossTerm = X * domain.vertexA(:,:,vertexIdx) - Yk * domain.vertexC(:,:,vertexIdx) + ...
            domain.extensionFactors(vertexIdx) .* H * model.C2(1,:);
        coupling = Pk - X + slackScale .* crossTerm.';
        disturbance = [((X * model.E) - (Yk * model.F)) * mismatchWeight, -Yk * measurementWeight];
        for rateIdx = 1:4
            PkRate = blendVertexCells(P, domain.rateVertices(:,rateIdx));
            Phi = crossTerm + crossTerm.' + PkRate + 2.0 .* decayRate .* Pk + ...
                (lipschitzConstant.^2 ./ tau) .* eye(2);
            block = [Phi, coupling, disturbance, sqrt(tau) .* Pk; ...
                coupling.', -slackScale .* (X + X.'), slackScale .* disturbance, zeros(2,2); ...
                disturbance.', slackScale .* disturbance.', -issGainSquared .* eye(4), zeros(4,2); ...
                sqrt(tau) .* Pk, zeros(2,2), zeros(2,4), -eye(2)];
            constraints = [constraints, block <= -strictnessEpsilon .* eye(10)]; %#ok<AGROW>
        end
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
    a = polytope.speedRange(1);
    b = polytope.speedRange(2);
    candidate.vertexGains(:,1,3) = b/(a+b)*candidate.vertexGains(:,1,1) + ...
        a/(a+b)*candidate.vertexGains(:,1,2);
    candidate.lyapunovBasis = solution.P;
    candidate.slackMatrix = solution.X;
end

function diagnostics = verifyPhysicalGrid(grid, model, scales, candidate, synthesisCfg, tau)
% verifyPhysicalGrid: Re-evaluate the original, non-convexified certificate
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
%   grid: supplementary physical diagnostic points
%   model: struct from lateralObserverSupport.lateralBicycleModel
%   scales: struct from disturbanceScales
%   candidate: struct from recoverVertexGains
%   synthesisCfg: cfg.synthesis struct
%   tau: fixed positive scalar used by the synthesis
%
% Output:
%   diagnostics: struct with certificateMargins, errorEigenvalues, the ISS
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

    diagnostics = struct();
    diagnostics.certificateMargins = certificateMargins;
    diagnostics.errorEigenvalues = errorEigenvalues;
    diagnostics.issGain = sqrt(gainSquared(1));
    diagnostics.mismatchIssGain = sqrt(gainSquared(2));
    diagnostics.measurementIssGain = sqrt(gainSquared(3));
    diagnostics.minLyapunovEigenvalue = minLyapunovEigenvalue;
    diagnostics.certified = all(certificateMargins < 0) && all(real(errorEigenvalues(:)) < 0) && ...
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
