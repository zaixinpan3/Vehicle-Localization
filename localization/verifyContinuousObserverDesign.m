function verification = verifyContinuousObserverDesign(design, cfg)
% verifyContinuousObserverDesign Exhaustively check the final ten-order LMI.
% Every combination of the 13-parameter h box, the two heading-weight
% endpoints, and the four exact-known-input vertices is evaluated with the
% recovered numerical matrices. No optimization toolbox is needed.

    arguments
        design (1, 1) struct
        cfg (1, 1) struct
    end

    validateDesign(design);
    data = buildImprovedObserverCertificateData(cfg);
    P = double(design.P);
    K = double(design.K);
    N = double(design.N);
    X = double(design.X);
    lambda = double(design.lambda);
    Ytranspose = P * K;
    identity = eye(7);
    worstBlockMargin = -Inf;
    worstSchurMargin = -Inf;
    worstCombination = [1, 1, 1];
    checkedCount = 0;

    for outputIdx = 1:data.outputVertexCount
        outputContribution = N * data.outputVertices(:, :, outputIdx);
        for omegaIdx = 1:data.omegaVertexCount
            measurementContribution = Ytranspose * data.omegaVertices(:, :, omegaIdx) * data.Cb;
            for fIdx = 1:data.fVertexCount
                systemMatrix = data.A + data.fVertices(:, :, fIdx) + outputContribution;
                affineMatrix = P * systemMatrix - measurementContribution;
                topLeft = affineMatrix + affineMatrix.' + lambda .* identity;
                block = [topLeft, Ytranspose; Ytranspose.', -X];
                blockMargin = max(real(eig(0.5 .* (block + block.'))));
                schurMatrix = topLeft + Ytranspose * (X \ Ytranspose.');
                schurMargin = max(real(eig(0.5 .* (schurMatrix + schurMatrix.'))));
                checkedCount = checkedCount + 1;
                if blockMargin > worstBlockMargin
                    worstBlockMargin = blockMargin;
                    worstCombination = [outputIdx, omegaIdx, fIdx];
                end
                worstSchurMargin = max(worstSchurMargin, schurMargin);
            end
        end
    end

    tolerance = max(1.0e-10, double(cfg.synthesis.violationTolerance));
    verification = struct();
    verification.outputVertexCount = data.outputVertexCount;
    verification.omegaVertexCount = data.omegaVertexCount;
    verification.fVertexCount = data.fVertexCount;
    verification.checkedVertexCount = checkedCount;
    verification.worstBlockMargin = worstBlockMargin;
    verification.worstSchurMargin = worstSchurMargin;
    verification.worstCombination = worstCombination;
    verification.minimumPEigenvalue = min(real(eig(0.5 .* (P + P.'))));
    verification.minimumXEigenvalue = min(real(eig(0.5 .* (X + X.'))));
    verification.invariantGainNorm = norm(N, "fro");
    verification.certified = verification.minimumPEigenvalue > tolerance && ...
        verification.minimumXEigenvalue > tolerance && ...
        verification.invariantGainNorm > 0.0 && ...
        worstBlockMargin < -tolerance && worstSchurMargin < -tolerance && ...
        checkedCount == data.totalVertexCount;
end

function validateDesign(design)
% validateDesign Check every matrix required by the online observer and LMI.
    requiredFields = ["P", "K", "N", "X", "lambda"];
    for fieldName = requiredFields
        assert(isfield(design, fieldName), "design.%s is required.", fieldName);
    end
    assert(isequal(size(design.P), [7, 7]) && all(isfinite(design.P(:))), ...
        "design.P must be a finite [7 x 7] matrix.");
    assert(isequal(size(design.K), [7, 3]) && all(isfinite(design.K(:))), ...
        "design.K must be a finite [7 x 3] matrix.");
    assert(isequal(size(design.N), [7, 4]) && all(isfinite(design.N(:))), ...
        "design.N must be a finite [7 x 4] matrix.");
    assert(isequal(size(design.X), [3, 3]) && all(isfinite(design.X(:))), ...
        "design.X must be a finite [3 x 3] matrix.");
    assert(isscalar(design.lambda) && isfinite(design.lambda) && design.lambda > 0.0, ...
        "design.lambda must be positive and finite.");
end
