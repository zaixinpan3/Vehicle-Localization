function design = designImprovedObserverGains(cfg)
% designImprovedObserverGains Synthesize the seven-state robust HGO gains.
% A cutting-plane loop solves only active vertex LMIs, while every iteration
% searches the complete output/f/heading-weight Cartesian product. The final
% design is accepted only after exhaustive verification of the complete
% Schur-form LMI from the improved observer derivation.

    arguments
        cfg (1, 1) struct
    end

    assert(exist("sdpvar", "file") == 2, ...
        "YALMIP must be on the MATLAB path before improved-observer synthesis.");
    validateSynthesisConfiguration(cfg);
    certificateData = buildImprovedObserverCertificateData(cfg);
    invariantGain = double(cfg.observer.invariantGain);
    assert(isequal(size(invariantGain), [7, 4]) && all(isfinite(invariantGain(:))), ...
        "cfg.observer.invariantGain must be a finite [7 x 4] matrix.");
    assert(norm(invariantGain, "fro") > 0.0, ...
        "cfg.observer.invariantGain must be nonzero so the invariant channel is implemented.");

    selectedCombinations = initialCombinations(certificateData);
    attempts = repmat(emptyAttempt(), double(cfg.synthesis.maximumCuttingPlaneIterations), 1);
    solved = false;
    finalP = [];
    finalY = [];
    candidateLambda = NaN;

    for iteration = 1:double(cfg.synthesis.maximumCuttingPlaneIterations)
        solution = solveActiveLmIs(selectedCombinations, invariantGain, certificateData, cfg);
        attempts(iteration).iteration = iteration;
        attempts(iteration).activeConstraintCount = size(selectedCombinations, 1);
        attempts(iteration).solverProblem = solution.problem;
        attempts(iteration).solverInfo = solution.info;
        attempts(iteration).candidateLambda = solution.lambda;
        if solution.problem ~= 0
            error("Improved-observer LMI failed at cutting-plane iteration %d: %s", iteration, solution.info);
        end

        [worstMargin, worstCombination] = findWorstCoreCombination( ...
            solution.P, solution.Y, invariantGain, solution.lambda, certificateData);
        attempts(iteration).worstExhaustiveMargin = worstMargin;
        attempts(iteration).worstCombination = worstCombination;
        if worstMargin <= double(cfg.synthesis.violationTolerance)
            solved = true;
            finalP = solution.P;
            finalY = solution.Y;
            candidateLambda = solution.lambda;
            attempts = attempts(1:iteration);
            break;
        end
        assert(~ismember(worstCombination, selectedCombinations, "rows"), ...
            "The active LMI remains violated beyond solver tolerance; tighten solver settings.");
        selectedCombinations = [selectedCombinations; worstCombination]; %#ok<AGROW>
    end
    assert(solved, "The improved-observer cutting-plane design reached its iteration limit.");

    certifiedLambda = double(cfg.synthesis.lambdaSafetyFactor) .* candidateLambda;
    [worstCoreMargin, worstCoreCombination] = findWorstCoreCombination( ...
        finalP, finalY, invariantGain, certifiedLambda, certificateData);
    assert(worstCoreMargin < 0.0, ...
        "The safeguarded certificate lambda did not leave a strict core-LMI margin.");
    decisionNormSquared = norm(finalY, 2).^2;
    schurScale = double(cfg.synthesis.schurSafetyFactor) .* ...
        max(decisionNormSquared, eps) ./ (-worstCoreMargin);
    X = schurScale .* eye(3);

    design = struct();
    design.P = finalP;
    design.K = finalP \ finalY;
    design.N = invariantGain;
    design.Ytranspose = finalY;
    design.Ztranspose = finalP * invariantGain;
    design.X = X;
    design.lambda = certifiedLambda;
    design.candidateLambda = candidateLambda;
    design.theta = double(cfg.observer.theta);
    design.sigma = double(cfg.observer.sigma);
    design.scalingExponents = double(cfg.observer.scalingExponents(:));
    design.selectedCombinations = selectedCombinations;
    design.cuttingPlaneAttempts = attempts;
    design.certificateVertexCounts = [certificateData.outputVertexCount, ...
        certificateData.omegaVertexCount, certificateData.fVertexCount];
    design.worstCoreMargin = worstCoreMargin;
    design.worstCoreCombination = worstCoreCombination;
    design.knownInputIncludedExactly = true;
    design.invariantGainMode = "fixed-nonzero";
    design.cfg = cfg;

    verification = verifyImprovedObserverDesign(design, cfg);
    design.verification = verification;
    design.certified = verification.certified;
    assert(design.certified, ...
        "Exhaustive verification rejected the synthesized improved-observer design.");
    saveDesignIfRequested(design, cfg);
end

function solution = solveActiveLmIs(combinations, invariantGain, data, cfg)
% solveActiveLmIs Solve one cutting-plane relaxation.
    yalmip('clear');
    stateCount = 7;
    P = sdpvar(stateCount, stateCount, 'symmetric');
    Ytranspose = sdpvar(stateCount, 3, 'full');
    lambda = sdpvar(1, 1);
    pFloor = double(cfg.synthesis.minimumPEigenvalue);
    normalizationTrace = double(cfg.synthesis.normalizationTrace);
    constraints = [P >= pFloor .* eye(stateCount), trace(P) == normalizationTrace, ...
        lambda >= double(cfg.synthesis.minimumLambda), ...
        lambda <= double(cfg.synthesis.maximumLambda)];
    strictness = double(cfg.synthesis.strictnessEpsilon);
    for combinationIdx = 1:size(combinations, 1)
        outputIdx = combinations(combinationIdx, 1);
        omegaIdx = combinations(combinationIdx, 2);
        fIdx = combinations(combinationIdx, 3);
        systemMatrix = data.A + data.fVertices(:, :, fIdx) + ...
            invariantGain * data.outputVertices(:, :, outputIdx);
        affineMatrix = P * systemMatrix - ...
            Ytranspose * data.omegaVertices(:, :, omegaIdx) * data.Cb;
        residual = affineMatrix + affineMatrix.' + lambda .* eye(stateCount);
        constraints = [constraints, residual <= -strictness .* eye(stateCount)]; %#ok<AGROW>
    end

    gainWeight = double(cfg.synthesis.gainDecisionWeight);
    objective = -lambda + gainWeight .* sum(Ytranspose(:).^2);
    options = sdpsettings('solver', char(string(cfg.synthesis.solver)), ...
        'verbose', double(cfg.synthesis.verbose), ...
        'dualize', double(cfg.synthesis.dualize), 'cachesolvers', 1);
    diagnostics = optimize(constraints, objective, options);

    solution = struct();
    solution.problem = diagnostics.problem;
    solution.info = string(diagnostics.info);
    solution.P = [];
    solution.Y = [];
    solution.lambda = NaN;
    if diagnostics.problem == 0
        solution.P = double(P);
        solution.Y = double(Ytranspose);
        solution.lambda = double(lambda);
    end
end

function [worstMargin, worstCombination] = findWorstCoreCombination(P, Ytranspose, invariantGain, lambda, data)
% findWorstCoreCombination Exhaustively maximize the seven-state residual.
    worstMargin = -Inf;
    worstCombination = [1, 1, 1];
    identity = eye(7);
    for outputIdx = 1:data.outputVertexCount
        outputContribution = invariantGain * data.outputVertices(:, :, outputIdx);
        for omegaIdx = 1:data.omegaVertexCount
            measurementContribution = Ytranspose * data.omegaVertices(:, :, omegaIdx) * data.Cb;
            for fIdx = 1:data.fVertexCount
                systemMatrix = data.A + data.fVertices(:, :, fIdx) + outputContribution;
                affineMatrix = P * systemMatrix - measurementContribution;
                residual = affineMatrix + affineMatrix.' + lambda .* identity;
                margin = max(real(eig(0.5 .* (residual + residual.'))));
                if margin > worstMargin
                    worstMargin = margin;
                    worstCombination = [outputIdx, omegaIdx, fIdx];
                end
            end
        end
    end
end

function combinations = initialCombinations(data)
% initialCombinations Seed opposite output and known-input corners.
    combinations = [1, 1, 1; ...
        data.outputVertexCount, data.omegaVertexCount, data.fVertexCount; ...
        max(1, floor(data.outputVertexCount ./ 2.0)), 1, ceil(data.fVertexCount ./ 2.0)];
    combinations = unique(combinations, "rows", "stable");
end

function attempt = emptyAttempt()
% emptyAttempt Return one initialized iteration record.
    attempt = struct("iteration", 0, "activeConstraintCount", 0, ...
        "solverProblem", NaN, "solverInfo", "", "candidateLambda", NaN, ...
        "worstExhaustiveMargin", NaN, "worstCombination", [NaN, NaN, NaN]);
end

function saveDesignIfRequested(design, cfg)
% saveDesignIfRequested Save only when an explicit output folder was given.
    outputFolder = string(cfg.synthesis.outputFolder);
    if strlength(outputFolder) == 0
        return;
    end
    assert(isfolder(outputFolder), ...
        "cfg.synthesis.outputFolder must name an existing folder.");
    outputPath = fullfile(outputFolder, string(cfg.synthesis.saveFileName));
    save(outputPath, "design");
end

function validateSynthesisConfiguration(cfg)
% validateSynthesisConfiguration Check scaling and convex-program settings.
    assert(isfield(cfg, "synthesis") && isstruct(cfg.synthesis), ...
        "cfg.synthesis is required.");
    theta = double(cfg.observer.theta);
    sigma = double(cfg.observer.sigma);
    assert(isscalar(theta) && isfinite(theta) && theta > sigma, ...
        "cfg.observer.theta must be finite and strictly greater than sigma.");
    positiveFields = ["normalizationTrace", "minimumPEigenvalue", "minimumLambda", ...
        "maximumLambda", "strictnessEpsilon", "violationTolerance", ...
        "lambdaSafetyFactor", "schurSafetyFactor", "maximumCuttingPlaneIterations"];
    for fieldName = positiveFields
        value = double(cfg.synthesis.(fieldName));
        assert(isscalar(value) && isfinite(value) && value > 0.0, ...
            "cfg.synthesis.%s must be positive and finite.", fieldName);
    end
    assert(cfg.synthesis.lambdaSafetyFactor < 1.0, ...
        "cfg.synthesis.lambdaSafetyFactor must be smaller than one.");
    assert(7.0 .* cfg.synthesis.minimumPEigenvalue < cfg.synthesis.normalizationTrace, ...
        "The P eigenvalue floor must leave room for the trace normalization.");
    gainWeight = double(cfg.synthesis.gainDecisionWeight);
    assert(isscalar(gainWeight) && isfinite(gainWeight) && gainWeight >= 0.0, ...
        "cfg.synthesis.gainDecisionWeight must be nonnegative and finite.");
end
