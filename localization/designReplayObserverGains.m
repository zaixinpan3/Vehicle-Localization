function design = designReplayObserverGains(cfg)
% designReplayObserverGains: Run the offline strict-replay observer gain design. The
% function constructs the full high-rate output polytope and a conservative
% timestamped pose-output Jacobian enclosure, then solves one main LMI that
% jointly synthesizes the Lyapunov matrix P, high-rate gain variable Z = P N,
% and discrete pose-correction variable W = P Kd. Sampled transition
% matrices are retained only as diagnostic validation data.
%
% Input:
%   cfg: struct from replayObserverConfig with replayDesign fields
%
% Output:
%   design: struct containing ph, poseCorrection, main, transition, and
%       updated cfg
    assert(nargin >= 1, ...
        "designReplayObserverGains requires an explicit cfg from replayObserverConfig; implicit design defaults are not allowed.");
    assertExplicitReplayConfig(cfg, "designReplayObserverGains");

    assertPoseYawSpeedBound(cfg);
    ph = constructHighRateOutputPolytope(cfg);
    poseCorrection = constructPoseCorrectionPolytope(cfg, ph);
    main = solveReplayMainLmi(ph, poseCorrection, cfg);
    cfgWithMain = cfg;
    cfgWithMain.observer.N = main.N;
    transition = constructTransitionPolytope(ph, main, cfgWithMain);

    cfgDesigned = cfgWithMain;
    cfgDesigned.replay.Kd = main.Kd;
    cfgDesigned.replay.timestampCorrectionGain = [];

    design = struct();
    design.cfg = cfgDesigned;
    design.ph = ph;
    design.poseCorrection = poseCorrection;
    design.main = main;
    design.flow = main;
    design.transition = transition;
    design.lifted = discreteCorrectionCompatibility(main, poseCorrection);
    design.summary = designSummary(ph, poseCorrection, main, transition);

    saveDesign(design, cfgDesigned);
end

function assertExplicitReplayConfig(cfg, callerName)
% assertExplicitReplayConfig: Validate that replay observer design
% functions received an explicit configuration object instead of relying on
% internally constructed design defaults.
%
% Input:
%   cfg: candidate observer configuration struct
%   callerName: name of the function requiring the explicit configuration
%
% Output:
%   none
    assert(~isempty(cfg) && isstruct(cfg), ...
        "%s requires an explicit cfg struct; implicit design defaults are not allowed.", ...
        callerName);
end

function summary = designSummary(ph, poseCorrection, main, transition)
% designSummary: Build a compact numeric summary of the synthesized
% replay observer design for quick inspection after running the workflow.
%
% Input:
%   ph: high-rate output polytope struct
%   poseCorrection: timestamped pose-output polytope struct
%   main: main joint LMI solution struct
%   transition: diagnostic transition polytope struct
%
% Output:
%   summary: scalar struct with key design metrics
    summary = struct();
    summary.numPhVertices = ph.numVertices;
    summary.highRateEnvelopeVerified = ph.verified;
    summary.numCertifiedHighRateVertices = main.numCertifiedHighRateVertices;
    summary.numPoseCorrectionVertices = poseCorrection.numVertices;
    summary.numTransitionDiagnosticVertices = transition.numVertices;
    summary.minPEigenvalue = main.minPEigenvalue;
    summary.maxPEigenvalue = main.maxPEigenvalue;
    summary.flowAh = main.aH;
    summary.maxFlowMargin = max(main.flowVertexMargins);
    summary.jumpRho = main.rho;
    summary.netContractionFactorBound = main.netContractionFactorBound;
    summary.netContractionCertified = main.netContractionCertified;
    summary.maxJumpMargin = max(main.jumpVertexMargins);
    summary.transitionDiagnosticOnly = transition.diagnosticOnly;
    summary.transitionVerified = transition.verified;
    summary.certificationScope = "All vertices of the constructed high-rate box polytope and pose-correction polytope are certified; continuous operating-set coverage requires an independently verified Jacobian enclosure.";
end

function lifted = discreteCorrectionCompatibility(main, poseCorrection)
% discreteCorrectionCompatibility: Provide a compatibility summary for
% downstream scripts that previously read the lifted sampled-data design
% field. The values now come from the main full-vertex flow/jump LMI rather
% than from a transition-polytope synthesis step.
%
% Input:
%   main: main joint LMI solution struct
%   poseCorrection: timestamped pose-output polytope struct
%
% Output:
%   lifted: compatibility struct exposing Kd, W, C vertices, and margins
    lifted = struct();
    lifted.Kd = main.Kd;
    lifted.W = main.W;
    lifted.P = main.P;
    lifted.PInput = main.P;
    lifted.C = poseCorrection.vertices;
    lifted.rho = main.rho;
    lifted.refinedP = false;
    lifted.flowMargins = main.flowVertexMargins;
    lifted.diagnostics = main.diagnostics;
    lifted.contractionMargins = main.jumpVertexMargins;
    lifted.maxContractionMargin = max(main.jumpVertexMargins);
    lifted.primaryProofObject = false;
    lifted.description = "Compatibility view of the main discrete jump LMI; no lifted transition-polytope synthesis is used.";
end

function saveDesign(design, cfg)
% saveDesign: Save the synthesized replay observer design to the
% configured output folder, or skip persistence when outputFolder is empty.
%
% Input:
%   design: replay observer design struct
%   cfg: updated replay observer configuration
%
% Output:
%   none
    designCfg = replayDesign(cfg);
    if ~isfield(designCfg, "outputFolder") || isempty(designCfg.outputFolder)
        return;
    end
    outputFolder = char(readString(designCfg, "outputFolder"));
    if ~exist(outputFolder, "dir")
        mkdir(outputFolder);
    end
    saveFileName = readString(designCfg, "saveFileName");
    save(fullfile(outputFolder, char(saveFileName)), "design");
end

function assertPoseYawSpeedBound(cfg)
% assertPoseYawSpeedBound: Validate that the configured operating set
% has a strictly positive speed lower bound before any singular speed,
% yaw-rate, or yaw pose-output Jacobian is constructed or sampled.
%
% Input:
%   cfg: observer configuration struct
%
% Output:
%   none
    design = replayDesign(cfg);
    outputs = poseCorrectionOutputs(design);
    includeYaw = any(lower(strtrim(outputs)) == "yaw");
    extraOutputs = lower(strtrim(string(cfg.observer.extraOutputs(:))));
    includeSingularHighRateOutput = any(ismember(extraOutputs, ["speed"; "yawrate"]));
    if includeYaw || includeSingularHighRateOutput
        minSpeedEpsilon = readSpeedSingularityEpsilon(design);
        minDesignSpeed = min(double(cfg.design.speedBounds(:)));
        assert(minDesignSpeed > minSpeedEpsilon, ...
            "cfg.design.speedBounds must have a strictly positive lower bound greater than %.6g when speed, yawrate, or yaw pose correction Jacobians are enabled.", ...
            minSpeedEpsilon);
    end
end

function ph = constructHighRateOutputPolytope(cfg)
% constructHighRateOutputPolytope: Construct the finite box polytope used to
% over-approximate the scaled mean-value coefficient matrix for
% h(z)-h(zhat). The function samples the compact vehicle operating set,
% computes analytic Jacobian entries for the configured high-rate outputs,
% inflates active derivative intervals, converts them to scaled coefficients,
% and returns all structural-zero-preserving box vertices.
%
% Input:
%   cfg: struct from replayObserverConfig with replayDesign fields
%
% Output:
%   ph: struct with derivative bounds, scaled coefficient bounds, active
%       coefficient locations, and vertices H_i of the polytope P_h
    assert(nargin >= 1, ...
        "constructHighRateOutputPolytope requires an explicit cfg; implicit design defaults are not allowed.");
    assertExplicitReplayConfig(cfg, "constructHighRateOutputPolytope");

    extraOutputs = string(cfg.observer.extraOutputs(:));
    theta = double(cfg.observer.theta);
    scalingExponents = double(cfg.observer.scalingExponents(:)).';
    r = double(cfg.observer.extraGainThetaPower);
    design = replayDesign(cfg);

    heading = linspace(-pi, pi, round(readScalar(design, "headingSamples")));
    speed = linspace(cfg.design.speedBounds(1), cfg.design.speedBounds(2), round(readScalar(design, "speedSamples")));
    yawRate = linspace(-cfg.design.maxAbsYawRate, cfg.design.maxAbsYawRate, round(readScalar(design, "yawRateSamples")));
    [headingGrid, speedGrid, yawRateGrid] = ndgrid(heading, speed, yawRate);
    z2 = speedGrid(:) .* cos(headingGrid(:));
    z5 = speedGrid(:) .* sin(headingGrid(:));
    z3 = -z5 .* yawRateGrid(:);
    z6 = z2 .* yawRateGrid(:);
    gradients = outputGradients(z2, z3, z5, z6, extraOutputs);
    assert(all(isfinite(gradients(:))), ...
        "High-rate output Jacobian samples contain nonfinite values; check cfg.design.speedBounds and singular high-rate outputs.");
    derivativeBounds = computeDerivativeBounds(gradients, readScalar(design, "derivativeInflation"));
    scaledCoefficientBounds = computeScaledCoefficientBounds(derivativeBounds, theta, r, scalingExponents);
    [vertices, vertexValues, activeRows, activeCols] = boxVertices(scaledCoefficientBounds);

    ph = struct();
    ph.extraOutputs = extraOutputs;
    ph.theta = theta;
    ph.r = r;
    ph.scalingExponents = scalingExponents;
    ph.Ttheta = diag(theta .^ scalingExponents);
    ph.headingGrid = heading;
    ph.speedGrid = speed;
    ph.yawRateGrid = yawRate;
    ph.derivativeBounds = derivativeBounds;
    ph.scaledCoefficientBounds = scaledCoefficientBounds;
    ph.vertices = vertices;
    ph.vertexValues = vertexValues;
    ph.activeRows = activeRows;
    ph.activeCols = activeCols;
    ph.numVertices = size(vertices, 3);
    ph.verified = false;
    ph.description = "Sampled and inflated box over-approximation of scaled high-rate output Jacobian.";
end

function gradients = outputGradients(z2, z3, z5, z6, extraOutputs)
% outputGradients: Evaluate analytic Jacobian rows of the configured
% vehicle high-rate output map h(z) on sampled operating-set points.
%
% Input:
%   z2: sampled X velocity component
%   z3: sampled X acceleration component
%   z5: sampled Y velocity component
%   z6: sampled Y acceleration component
%   extraOutputs: string vector naming configured high-rate outputs
%
% Output:
%   gradients: [Ns x m x 6] sampled Jacobian entries
    numSamples = numel(z2);
    numOutputs = numel(extraOutputs);
    gradients = zeros(numSamples, numOutputs, 6);
    speedSquared = z2 .* z2 + z5 .* z5;
    numerator = -z5 .* z3 + z2 .* z6;

    for outputIdx = 1:numOutputs
        outputName = lower(strtrim(extraOutputs(outputIdx)));
        if outputName == "speed"
            gradients(:, outputIdx, 2) = z2 ./ sqrt(speedSquared);
            gradients(:, outputIdx, 5) = z5 ./ sqrt(speedSquared);
        elseif outputName == "orthogonalityconstraint"
            gradients(:, outputIdx, 2) = z3;
            gradients(:, outputIdx, 3) = z2;
            gradients(:, outputIdx, 5) = z6;
            gradients(:, outputIdx, 6) = z5;
        elseif outputName == "yawrate"
            gradients(:, outputIdx, 2) = z6 ./ speedSquared - 2.0 .* z2 .* numerator ./ (speedSquared .* speedSquared);
            gradients(:, outputIdx, 3) = -z5 ./ speedSquared;
            gradients(:, outputIdx, 5) = -z3 ./ speedSquared - 2.0 .* z5 .* numerator ./ (speedSquared .* speedSquared);
            gradients(:, outputIdx, 6) = z2 ./ speedSquared;
        else
            error("Unsupported Bessafa 2026 extra output: %s.", outputName);
        end
    end
end

function derivativeBounds = computeDerivativeBounds(gradients, inflation)
% computeDerivativeBounds: Convert sampled derivative values into lower and
% upper interval bounds while preserving structural zeros exactly.
%
% Input:
%   gradients: [Ns x m x 6] sampled derivative values
%   inflation: nonnegative scalar interval inflation for active entries
%
% Output:
%   derivativeBounds: [m x 6 x 2] derivative lower and upper bounds
    numOutputs = size(gradients, 2);
    derivativeBounds = zeros(numOutputs, 6, 2);
    inflation = max(0.0, double(inflation));
    for outputIdx = 1:numOutputs
        for stateIdx = 1:6
            values = gradients(:, outputIdx, stateIdx);
            lowerValue = min(values);
            upperValue = max(values);
            if max(abs(values)) > 1.0e-12
                lowerValue = lowerValue - inflation;
                upperValue = upperValue + inflation;
            end
            derivativeBounds(outputIdx, stateIdx, 1) = lowerValue;
            derivativeBounds(outputIdx, stateIdx, 2) = upperValue;
        end
    end
end

function scaledCoefficientBounds = computeScaledCoefficientBounds(derivativeBounds, theta, r, scalingExponents)
% computeScaledCoefficientBounds: Convert derivative intervals into intervals
% for the scaled coefficient matrix -theta^(-r-1) Jh Ttheta.
%
% Input:
%   derivativeBounds: [m x 6 x 2] derivative interval bounds
%   theta: high-gain scalar
%   r: relative-degree chain length
%   scalingExponents: [1 x 6] high-gain scaling exponents
%
% Output:
%   scaledCoefficientBounds: [m x 6 x 2] scaled coefficient bounds
    scaledCoefficientBounds = zeros(size(derivativeBounds));
    for stateIdx = 1:6
        scale = -theta .^ (-r - 1.0 + scalingExponents(stateIdx));
        lowerValue = scale .* derivativeBounds(:, stateIdx, 1);
        upperValue = scale .* derivativeBounds(:, stateIdx, 2);
        scaledCoefficientBounds(:, stateIdx, 1) = min(lowerValue, upperValue);
        scaledCoefficientBounds(:, stateIdx, 2) = max(lowerValue, upperValue);
    end
end

function [vertices, vertexValues, activeRows, activeCols] = boxVertices(scaledCoefficientBounds)
% boxVertices: Enumerate all lower/upper endpoint combinations of the
% active scaled coefficients while preserving structural zero entries.
%
% Input:
%   scaledCoefficientBounds: [m x 6 x 2] scaled coefficient bounds
%
% Output:
%   vertices: [m x 6 x Nv] box polytope vertices
%   vertexValues: [Nv x Na] selected active coefficient values
%   activeRows: [Na x 1] active output-row indices
%   activeCols: [Na x 1] active state-column indices
    lowerBounds = scaledCoefficientBounds(:, :, 1);
    upperBounds = scaledCoefficientBounds(:, :, 2);
    activeMask = (abs(lowerBounds - upperBounds) > 1.0e-12) | (abs(lowerBounds) > 1.0e-12);
    [activeRows, activeCols] = find(activeMask);
    numActive = numel(activeRows);
    numVertices = round(2 .^ numActive);
    vertices = zeros(size(lowerBounds, 1), size(lowerBounds, 2), numVertices);
    vertexValues = zeros(numVertices, numActive);

    for vertexIdx = 1:numVertices
        vertexMatrix = zeros(size(lowerBounds));
        for activeIdx = 1:numActive
            rowIdx = activeRows(activeIdx);
            colIdx = activeCols(activeIdx);
            if bitget(vertexIdx - 1, activeIdx) == 0
                coefficient = lowerBounds(rowIdx, colIdx);
            else
                coefficient = upperBounds(rowIdx, colIdx);
            end
            vertexValues(vertexIdx, activeIdx) = coefficient;
            vertexMatrix(rowIdx, colIdx) = coefficient;
        end
        vertices(:, :, vertexIdx) = vertexMatrix;
    end
end

function poseCorrection = constructPoseCorrectionPolytope(cfg, ph)
% constructPoseCorrectionPolytope: Construct the conservative
% timestamped pose-output Jacobian enclosure used by the discrete correction
% part of the main replay LMI. X and Y rows are exact scaled position
% outputs, while yaw is represented by an interval box over the scaled
% heading Jacobian on the configured positive-speed operating set.
%
% Input:
%   cfg: observer configuration struct
%   ph: high-rate output polytope struct containing theta and scaling data
%
% Output:
%   poseCorrection: struct with output names, vertices C_j, and bounds
    design = replayDesign(cfg);
    outputs = poseCorrectionOutputs(design);
    includeYaw = any(lower(strtrim(outputs)) == "yaw");
    minDesignSpeed = min(double(cfg.design.speedBounds(:)));
    if includeYaw
        minSpeedEpsilon = readSpeedSingularityEpsilon(design);
        assert(minDesignSpeed > minSpeedEpsilon, ...
            "cfg.design.speedBounds must have a strictly positive lower bound greater than %.6g when yaw pose correction is enabled.", ...
            minSpeedEpsilon);
    end

    bounds = poseCorrectionBounds(outputs, ph, minDesignSpeed);
    [vertices, vertexValues, activeRows, activeCols] = boxVertices(bounds);

    poseCorrection = struct();
    poseCorrection.outputs = outputs;
    poseCorrection.vertices = vertices;
    poseCorrection.vertexValues = vertexValues;
    poseCorrection.activeRows = activeRows;
    poseCorrection.activeCols = activeCols;
    poseCorrection.bounds = bounds;
    poseCorrection.numVertices = size(vertices, 3);
    poseCorrection.minDesignSpeed = minDesignSpeed;
    poseCorrection.description = "Conservative scaled Jacobian enclosure for timestamped [X,Y,yaw] pose-correction jumps.";
end

function outputs = poseCorrectionOutputs(design)
% poseCorrectionOutputs: Read and normalize the configured timestamped
% pose-correction output channels used in the discrete jump LMI.
%
% Input:
%   design: replay-design parameter struct
%
% Output:
%   outputs: [p x 1] string vector of pose-correction channel names
    assert(isfield(design, "poseCorrectionOutputs") && ~isempty(design.poseCorrectionOutputs), ...
        "cfg.replayDesign.poseCorrectionOutputs is required; implicit design defaults are not allowed.");
    outputs = string(design.poseCorrectionOutputs(:));
    outputs = lower(strtrim(outputs));
    assert(all(ismember(outputs, ["x", "y", "yaw"])), ...
        "cfg.replayDesign.poseCorrectionOutputs may contain only x, y, and yaw.");
    assert(numel(unique(outputs)) == numel(outputs), ...
        "cfg.replayDesign.poseCorrectionOutputs must not repeat channels.");
    assert(all(ismember(["x", "y"], outputs)), ...
        "cfg.replayDesign.poseCorrectionOutputs must include x and y.");
    canonicalOutputs = ["x"; "y"; "yaw"];
    assert(all(outputs == canonicalOutputs(1:numel(outputs))), ...
        "cfg.replayDesign.poseCorrectionOutputs must be ordered as [x; y] or [x; y; yaw].");
end

function bounds = poseCorrectionBounds(outputs, ph, minDesignSpeed)
% poseCorrectionBounds: Build lower and upper interval bounds for the
% scaled timestamped pose-output Jacobian rows used in the discrete jump LMI.
%
% Input:
%   outputs: [p x 1] pose-correction channel names
%   ph: high-rate output polytope struct containing theta and scaling data
%   minDesignSpeed: positive minimum speed when yaw is enabled
%
% Output:
%   bounds: [p x 6 x 2] lower and upper Jacobian interval bounds
    bounds = zeros(numel(outputs), 6, 2);
    for outputIdx = 1:numel(outputs)
        outputName = lower(strtrim(outputs(outputIdx)));
        if outputName == "x"
            bounds(outputIdx, 1, :) = 1.0;
        elseif outputName == "y"
            bounds(outputIdx, 4, :) = 1.0;
        elseif outputName == "yaw"
            yawBound = ph.theta ./ minDesignSpeed;
            bounds(outputIdx, 2, 1) = -yawBound;
            bounds(outputIdx, 2, 2) = yawBound;
            bounds(outputIdx, 5, 1) = -yawBound;
            bounds(outputIdx, 5, 2) = yawBound;
        end
    end
end

function main = solveReplayMainLmi(ph, poseCorrection, cfg)
% solveReplayMainLmi: Solve the strict-replay observer main LMI
% with common P, high-rate correction decision Z = P N, and timestamped pose
% correction decision W = P Kd. Flow constraints are enforced over every
% high-rate output vertex, and pose-jump constraints are selected so the
% worst-case jump and flow composition is a strict net contraction over every
% conservative pose-output Jacobian vertex.
%
% Input:
%   ph: struct returned by constructHighRateOutputPolytope
%   poseCorrection: struct returned by constructPoseCorrectionPolytope
%   cfg: struct from replayObserverConfig
%
% Output:
%   main: struct with A, P, Z, N, W, Kd, rho, diagnostics, and LMI margins
    assert(nargin >= 3, ...
        "solveReplayMainLmi requires an explicit cfg; implicit design defaults are not allowed.");
    assertExplicitReplayConfig(cfg, "solveReplayMainLmi");

    assert(exist("sdpvar", "file") == 2, "YALMIP must be on the MATLAB path before solving the replay main LMI.");

    design = replayDesign(cfg);
    n = 6;
    q = size(ph.vertices, 1);
    p = size(poseCorrection.vertices, 1);
    A = vehicleA();
    aH = readScalar(design, "flowAh");
    pFloor = readScalar(design, "pFloor");
    pCeiling = readScalar(design, "pCeiling");
    strictnessEpsilon = readScalar(design, "strictnessEpsilon");
    jumpStrictnessEpsilon = readScalar(design, "jumpStrictnessEpsilon");
    rhoCandidates = sort(unique(readVector(design, "rhoCandidates")));
    intervalMax = readScalar(design, "transitionMaxInterval");
    maxAllowedRho = exp(-ph.theta .* aH .* intervalMax);
    options = sdpsettings('solver', char(string(readString(design, "solver"))), ...
        'verbose', double(readScalar(design, "verbose")), ...
        'dualize', double(readScalar(design, "dualize")));
    solved = false;
    selectedRho = NaN;
    selectedP = [];
    selectedZ = [];
    selectedW = [];
    selectedDiagnostics = [];
    for rhoIdx = 1:numel(rhoCandidates)
        rho = rhoCandidates(rhoIdx);
        if rho >= maxAllowedRho
            continue;
        end
        P = sdpvar(n, n, 'symmetric');
        Z = sdpvar(n, q, 'full');
        W = sdpvar(n, p, 'full');
        constraints = P >= pFloor .* eye(n);
        if isfinite(pCeiling)
            constraints = [constraints, P <= pCeiling .* eye(n)];
        end
        for vertexIdx = 1:size(ph.vertices, 3)
            H = ph.vertices(:, :, vertexIdx);
            lmiMatrix = P * A + Z * H;
            constraints = [constraints, lmiMatrix + lmiMatrix.' <= aH .* P - strictnessEpsilon .* eye(n)];
        end
        for vertexIdx = 1:size(poseCorrection.vertices, 3)
            C = poseCorrection.vertices(:, :, vertexIdx);
            affineTerm = P - W * C;
            lmiBlock = [rho .* P, affineTerm.'; affineTerm, P];
            constraints = [constraints, lmiBlock >= jumpStrictnessEpsilon .* eye(2 .* n)];
        end
        objectiveWeight = readScalar(design, "flowObjectiveWeight");
        objective = objectiveWeight .* trace(P);
        diagnostics = optimize(constraints, objective, options);
        selectedDiagnostics = diagnostics;
        if diagnostics.problem == 0
            solved = true;
            selectedRho = rho;
            selectedP = double(P);
            selectedZ = double(Z);
            selectedW = double(W);
            break;
        end
    end
    if isempty(selectedDiagnostics)
        selectedDiagnostics = struct("info", "No configured rho candidate satisfies the requested flow/jump design constraints.");
    end
    assert(solved, "Replay main LMI solve failed for all configured rho candidates. Net-contraction rho limit is %.6g. Last solver message: %s", ...
        maxAllowedRho, selectedDiagnostics.info);

    NValue = selectedP \ selectedZ;
    KdValue = selectedP \ selectedW;
    flowMargins = replayFlowLmiMargins(A, selectedP, selectedZ, ph.vertices, aH);
    jumpMargins = poseJumpLmiMargins(selectedP, selectedW, poseCorrection.vertices, selectedRho);

    main = struct();
    main.A = A;
    main.P = selectedP;
    main.Z = selectedZ;
    main.N = NValue;
    main.W = selectedW;
    main.Kd = KdValue;
    main.aH = aH;
    main.rho = selectedRho;
    main.theta = ph.theta;
    main.extraGainThetaPower = ph.r;
    main.maxPoseCorrectionInterval = intervalMax;
    main.maxAllowedJumpRhoForNetContraction = maxAllowedRho;
    main.netContractionFactorBound = selectedRho .* exp(ph.theta .* aH .* intervalMax);
    main.netContractionCertified = main.netContractionFactorBound < 1.0;
    main.diagnostics = selectedDiagnostics;
    main.minPEigenvalue = min(eig(selectedP));
    main.maxPEigenvalue = max(eig(selectedP));
    main.flowVertexMargins = flowMargins;
    main.vertexMargins = flowMargins;
    main.jumpVertexMargins = jumpMargins;
    main.numCertifiedHighRateVertices = size(ph.vertices, 3);
    main.certifiedHighRateVertexIdx = (1:size(ph.vertices, 3)).';
    main.fullHighRateVertexCertified = main.numCertifiedHighRateVertices == ph.numVertices;
    main.primaryProofObject = "common-flow-and-discrete-jump-LMI";
    assert(main.fullHighRateVertexCertified, "The main replay LMI must certify every high-rate output polytope vertex.");
    assert(main.netContractionCertified, ...
        "The selected jump contraction and worst-case flow growth do not compose to net contraction.");
end

function margins = replayFlowLmiMargins(A, P, Z, vertices, aH)
% replayFlowLmiMargins: Evaluate the largest eigenvalue margin of
% every solved flow LMI vertex after numerical optimization.
%
% Input:
%   A: [6 x 6] triangular flow matrix
%   P: [6 x 6] Lyapunov matrix value
%   Z: [6 x m] gain decision value
%   vertices: [m x 6 x Nv] H_i vertex matrices
%   aH: scalar flow growth bound
%
% Output:
%   margins: [Nv x 1] maximum eigenvalue of He(PA + ZH_i) - a_h P
    margins = zeros(size(vertices, 3), 1);
    for vertexIdx = 1:size(vertices, 3)
        H = vertices(:, :, vertexIdx);
        residual = P * A + Z * H;
        residual = residual + residual.' - aH .* P;
        margins(vertexIdx) = max(eig(0.5 .* (residual + residual.')));
    end
end

function margins = poseJumpLmiMargins(P, W, vertices, rho)
% poseJumpLmiMargins: Evaluate the largest eigenvalue residual of the
% discrete timestamped pose-correction contraction LMI at each pose-output
% Jacobian vertex.
%
% Input:
%   P: [6 x 6] Lyapunov matrix
%   W: [6 x p] timestamped correction decision value
%   vertices: [p x 6 x Nv] pose-output Jacobian vertices
%   rho: scalar jump contraction factor
%
% Output:
%   margins: [Nv x 1] maximum eigenvalue residuals
    margins = zeros(size(vertices, 3), 1);
    for vertexIdx = 1:size(vertices, 3)
        C = vertices(:, :, vertexIdx);
        Kd = P \ W;
        thetaMatrix = eye(6) - Kd * C;
        residual = thetaMatrix.' * P * thetaMatrix - rho .* P;
        margins(vertexIdx) = max(eig(0.5 .* (residual + residual.')));
    end
end

function A = vehicleA()
% vehicleA: Return the two-chain triangular A matrix for the Bessafa
% transformed vehicle coordinates.
%
% Input:
%   none
%
% Output:
%   A: [6 x 6] triangular vehicle matrix
    A3 = [0.0, 1.0, 0.0; 0.0, 0.0, 1.0; 0.0, 0.0, 0.0];
    A = blkdiag(A3, A3);
end

function transition = constructTransitionPolytope(ph, flow, cfg)
% constructTransitionPolytope: Construct a diagnostic sampled
% transition matrix over-approximation after the main replay LMI has already
% certified the design. This transition polytope is not a primary proof
% object and may sample a subset of H_i vertices only for secondary
% validation and engineering inspection.
%
% Input:
%   ph: struct returned by constructHighRateOutputPolytope
%   flow: main LMI solution struct containing A, N, P, and theta
%   cfg: struct from replayObserverConfig
%
% Output:
%   transition: struct with sampled Phi vertices, source H vertices, interval
%       samples, inflation level, and verification metadata
    assert(nargin >= 3, ...
        "constructTransitionPolytope requires an explicit cfg; implicit design defaults are not allowed.");
    assertExplicitReplayConfig(cfg, "constructTransitionPolytope");

    design = replayDesign(cfg);
    intervalMin = readScalar(design, "transitionMinInterval");
    intervalMax = readScalar(design, "transitionMaxInterval");
    numTimeSamples = max(1, round(readScalar(design, "transitionTimeSamples")));
    maxHVertices = max(1, round(readScalar(design, "transitionMaxHVertices")));
    inflation = max(0.0, readScalar(design, "transitionInflation"));

    hVertexIdx = selectedVertexIndices(size(ph.vertices, 3), maxHVertices);
    intervalSamples = linspace(intervalMin, intervalMax, numTimeSamples);
    baseVertices = sampleTransitionVertices(ph, flow, hVertexIdx, intervalSamples);
    vertices = inflateTransitionVertices(baseVertices, inflation);

    transition = struct();
    transition.vertices = vertices;
    transition.baseVertices = baseVertices;
    transition.hVertexIdx = hVertexIdx;
    transition.intervalSamples = intervalSamples(:);
    transition.inflation = inflation;
    transition.verified = readLogical(design, "transitionVerified");
    transition.diagnosticOnly = true;
    transition.primaryProofObject = false;
    transition.description = "Diagnostic sampled transition matrix polytope; the certified design uses the full-vertex flow/jump main LMI, not this sampled set.";
    transition.numVertices = size(vertices, 3);
end

function vertices = sampleTransitionVertices(ph, flow, hVertexIdx, intervalSamples)
% sampleTransitionVertices: Compute transition matrices for selected
% H_i vertices and timestamp intervals using constant-vertex scaled flow
% systems.
%
% Input:
%   ph: high-rate output polytope struct
%   flow: flow design struct containing A, N, and theta
%   hVertexIdx: selected H vertex indices
%   intervalSamples: sampled timestamp intervals
%
% Output:
%   vertices: [6 x 6 x Nv] sampled transition matrices
    numVertices = numel(hVertexIdx) .* numel(intervalSamples);
    vertices = zeros(6, 6, numVertices);
    vertexCounter = 0;
    for hIdx = 1:numel(hVertexIdx)
        H = ph.vertices(:, :, hVertexIdx(hIdx));
        F = flow.theta .* (flow.A + flow.N * H);
        for intervalIdx = 1:numel(intervalSamples)
            vertexCounter = vertexCounter + 1;
            vertices(:, :, vertexCounter) = expm(F .* intervalSamples(intervalIdx));
        end
    end
    assert(all(isfinite(vertices(:))), "Constructed transition vertices contain nonfinite values.");
end

function vertices = inflateTransitionVertices(baseVertices, inflation)
% inflateTransitionVertices: Add optional entry-axis perturbation
% vertices around sampled transition matrices. With zero inflation the
% sampled vertices are returned unchanged.
%
% Input:
%   baseVertices: [6 x 6 x Nv] sampled transition matrices
%   inflation: nonnegative scalar entry perturbation magnitude
%
% Output:
%   vertices: [6 x 6 x Nout] transition vertices
    if inflation <= 0.0
        vertices = baseVertices;
        return;
    end

    n = size(baseVertices, 1);
    numBase = size(baseVertices, 3);
    vertices = zeros(n, n, numBase .* (1 + 2 .* n .* n));
    writeIdx = 1;
    for baseIdx = 1:numBase
        vertices(:, :, writeIdx) = baseVertices(:, :, baseIdx);
        writeIdx = writeIdx + 1;
        for rowIdx = 1:n
            for colIdx = 1:n
                perturbation = zeros(n, n);
                perturbation(rowIdx, colIdx) = inflation;
                vertices(:, :, writeIdx) = baseVertices(:, :, baseIdx) + perturbation;
                vertices(:, :, writeIdx + 1) = baseVertices(:, :, baseIdx) - perturbation;
                writeIdx = writeIdx + 2;
            end
        end
    end
end

function hVertexIdx = selectedVertexIndices(numVertices, maxVertices)
% selectedVertexIndices: Select an evenly spaced deterministic subset
% of H vertices for transition sampling when the full box is too large.
%
% Input:
%   numVertices: total number of H vertices
%   maxVertices: maximum selected vertex count
%
% Output:
%   hVertexIdx: selected positive integer vertex indices
    if maxVertices >= numVertices
        hVertexIdx = (1:numVertices).';
    else
        hVertexIdx = unique(round(linspace(1, numVertices, maxVertices))).';
    end
end

function design = replayDesign(cfg)
% replayDesign: Return the required replayDesign field from the explicit
% observer configuration used by the replay observer design workflow.
%
% Input:
%   cfg: observer configuration struct
%
% Output:
%   design: replay-design parameter struct
    assert(isstruct(cfg) && isfield(cfg, "replayDesign") && ...
        ~isempty(cfg.replayDesign) && isstruct(cfg.replayDesign), ...
        "cfg.replayDesign is required; implicit design defaults are not allowed.");
    design = cfg.replayDesign;
end

function value = readSpeedSingularityEpsilon(design)
% readSpeedSingularityEpsilon: Read the replay-design speed lower-bound
% epsilon used for singular speed, yaw-rate, and yaw-output Jacobian checks.
% The legacy pose-yaw field remains accepted as a compatibility alias.
%
% Input:
%   design: replay-design parameter struct
%
% Output:
%   value: positive speed lower-bound epsilon
    if isfield(design, "speedSingularityMinSpeedEpsilon") && ~isempty(design.speedSingularityMinSpeedEpsilon)
        value = readScalar(design, "speedSingularityMinSpeedEpsilon");
    else
        value = readScalar(design, "poseYawMinSpeedEpsilon");
    end
    assert(value >= 0.0, "cfg.replayDesign.speedSingularityMinSpeedEpsilon must be nonnegative.");
end

function value = readScalar(source, fieldName)
% readScalar: Read one required numeric scalar replay-design field and
% fail when the field is absent, empty, nonnumeric, nonscalar, or NaN.
%
% Input:
%   source: struct to inspect
%   fieldName: string scalar field name
%
% Output:
%   value: selected scalar value
    fieldName = char(string(fieldName));
    assert(isstruct(source) && isfield(source, fieldName) && ~isempty(source.(fieldName)), ...
        "cfg.replayDesign.%s is required; implicit design defaults are not allowed.", ...
        fieldName);
    candidate = source.(fieldName);
    assert(isnumeric(candidate) && isreal(candidate) && isscalar(candidate), ...
        "cfg.replayDesign.%s must be a real numeric scalar.", fieldName);
    value = double(candidate);
    assert(~isnan(value), "cfg.replayDesign.%s must not be NaN.", fieldName);
end

function value = readString(source, fieldName)
% readString: Read one required nonempty string scalar replay-design
% field and fail when it is absent or invalid.
%
% Input:
%   source: struct to inspect
%   fieldName: string scalar field name
%
% Output:
%   value: selected string scalar
    fieldName = char(string(fieldName));
    assert(isstruct(source) && isfield(source, fieldName) && ~isempty(source.(fieldName)), ...
        "cfg.replayDesign.%s is required; implicit design defaults are not allowed.", ...
        fieldName);
    value = string(source.(fieldName));
    assert(isscalar(value) && ~ismissing(value) && strlength(value) > 0, ...
        "cfg.replayDesign.%s must be a nonempty string scalar.", fieldName);
end

function value = readLogical(source, fieldName)
% readLogical: Read one required logical scalar replay-design field and
% fail when it is absent, empty, nonscalar, or not equivalent to true/false.
%
% Input:
%   source: struct to inspect
%   fieldName: string scalar field name
%
% Output:
%   value: selected logical scalar
    fieldName = char(string(fieldName));
    assert(isstruct(source) && isfield(source, fieldName) && ~isempty(source.(fieldName)), ...
        "cfg.replayDesign.%s is required; implicit design defaults are not allowed.", ...
        fieldName);
    candidate = source.(fieldName);
    if islogical(candidate)
        assert(isscalar(candidate), "cfg.replayDesign.%s must be a logical scalar.", fieldName);
        value = candidate;
    else
        assert(isnumeric(candidate) && isreal(candidate) && isscalar(candidate) && isfinite(candidate) && ...
            (double(candidate) == 0.0 || double(candidate) == 1.0), ...
            "cfg.replayDesign.%s must be a logical scalar or numeric 0/1.", ...
            fieldName);
        value = logical(candidate);
    end
end

function value = readVector(source, fieldName)
% readVector: Read one required finite numeric replay-design vector and
% fail when it is absent, empty, nonnumeric, or contains nonfinite values.
%
% Input:
%   source: struct to inspect
%   fieldName: string scalar field name
%
% Output:
%   value: row vector of selected numeric values
    fieldName = char(string(fieldName));
    assert(isstruct(source) && isfield(source, fieldName) && ~isempty(source.(fieldName)), ...
        "cfg.replayDesign.%s is required; implicit design defaults are not allowed.", ...
        fieldName);
    candidate = source.(fieldName);
    assert(isnumeric(candidate) && isreal(candidate) && isvector(candidate), ...
        "cfg.replayDesign.%s must be a real numeric vector.", fieldName);
    value = double(candidate(:)).';
    assert(~isempty(value) && all(isfinite(value)), ...
        "cfg.replayDesign.%s must be a nonempty finite numeric vector.", ...
        fieldName);
end
