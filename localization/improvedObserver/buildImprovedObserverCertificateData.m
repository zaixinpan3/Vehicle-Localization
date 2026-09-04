function data = buildImprovedObserverCertificateData(cfg)
% buildImprovedObserverCertificateData Build every robust-LMI vertex family.
% The output-output mean-value box contains the 13 coefficients stated in
% the improved observer derivation. The known-input model term is included
% exactly through a four-vertex box in track-angle rate and its square.

    arguments
        cfg (1, 1) struct
    end

    validateConfiguration(cfg);
    stateCount = 7;
    outputCount = 4;
    sigma = double(cfg.observer.sigma);
    exponents = double(cfg.observer.scalingExponents(:));

    chain = [0.0, 1.0, 0.0; 0.0, 0.0, 1.0; 0.0, 0.0, 0.0];
    data = struct();
    data.A = blkdiag(chain, chain, 0.0);
    data.B = zeros(stateCount, 2);
    data.B(3, 1) = 1.0;
    data.B(6, 2) = 1.0;
    data.Bu = zeros(stateCount, 1);
    data.Bu(7) = 1.0;
    data.Cb = zeros(3, stateCount);
    data.Cb(1, 1) = 1.0;
    data.Cb(2, 4) = 1.0;
    data.Cb(3, 7) = 1.0;
    data.Cl = zeros(2, stateCount);
    data.Cl(1, 1) = 1.0;
    data.Cl(2, 4) = 1.0;
    data.scalingExponents = exponents;
    data.Tsigma = diag(sigma.^exponents);

    activeCoefficients = outputCoefficientTable(cfg);
    data.outputActiveCoefficients = activeCoefficients;
    data.outputVertices = enumerateOutputVertices(activeCoefficients, exponents, sigma, outputCount, stateCount);
    data.fVertices = buildKnownInputVertices(cfg, data.B, data.Tsigma, sigma, stateCount);
    minimumHeadingWeight = double(cfg.lidar.minimumHeadingWeight);
    data.omegaVertices = cat(3, diag([1.0, 1.0, minimumHeadingWeight]), eye(3));
    data.outputVertexCount = size(data.outputVertices, 3);
    data.fVertexCount = size(data.fVertices, 3);
    data.omegaVertexCount = size(data.omegaVertices, 3);
    data.totalVertexCount = data.outputVertexCount .* data.fVertexCount .* data.omegaVertexCount;
end

function table = outputCoefficientTable(cfg)
% outputCoefficientTable Return [row, column, absolute psi bound].
    maximumSpeed = double(cfg.operating.maximumSpeed);
    maximumAcceleration = double(cfg.operating.maximumAcceleration);
    table = [1, 2, 2.0 .* maximumSpeed; ...
        1, 5, 2.0 .* maximumSpeed; ...
        2, 2, maximumAcceleration; ...
        2, 5, maximumAcceleration; ...
        2, 3, maximumSpeed; ...
        2, 6, maximumSpeed; ...
        3, 2, maximumAcceleration; ...
        3, 5, maximumAcceleration; ...
        3, 6, maximumSpeed; ...
        3, 3, maximumSpeed; ...
        4, 2, 1.0; ...
        4, 5, 1.0; ...
        4, 7, 2.0 .* maximumSpeed];
end

function vertices = enumerateOutputVertices(active, exponents, sigma, outputCount, stateCount)
% enumerateOutputVertices Enumerate the structural-zero-preserving box.
    activeCount = size(active, 1);
    vertexCount = 2^activeCount;
    vertices = zeros(outputCount, stateCount, vertexCount);
    for vertexIdx = 1:vertexCount
        for activeIdx = 1:activeCount
            rowIdx = active(activeIdx, 1);
            columnIdx = active(activeIdx, 2);
            psiBound = active(activeIdx, 3);
            scaledBound = psiBound .* sigma.^(exponents(columnIdx) - 4.0);
            endpointSign = (2.0 .* bitget(vertexIdx - 1, activeIdx)) - 1.0;
            vertices(rowIdx, columnIdx, vertexIdx) = endpointSign .* scaledBound;
        end
    end
end

function vertices = buildKnownInputVertices(cfg, B, scaling, sigma, stateCount)
% buildKnownInputVertices Enclose q and q^2 as independent box parameters.
    maximumRate = double(cfg.operating.maximumTrackAngleRate);
    rateEndpoints = [-maximumRate, maximumRate];
    squaredRateEndpoints = [0.0, maximumRate.^2];
    vertices = zeros(stateCount, stateCount, 4);
    vertexIdx = 0;
    for rateIdx = 1:2
        for squaredRateIdx = 1:2
            vertexIdx = vertexIdx + 1;
            F = zeros(2, stateCount);
            F(1, 2) = squaredRateEndpoints(squaredRateIdx);
            F(1, 6) = -2.0 .* rateEndpoints(rateIdx);
            F(2, 5) = squaredRateEndpoints(squaredRateIdx);
            F(2, 3) = 2.0 .* rateEndpoints(rateIdx);
            vertices(:, :, vertexIdx) = sigma.^(-4.0) .* B * F * scaling;
        end
    end
end

function validateConfiguration(cfg)
% validateConfiguration Check the fields that define the certificate box.
    requiredGroups = ["operating", "observer", "lidar"];
    for groupName = requiredGroups
        assert(isfield(cfg, groupName), "cfg.%s is required.", groupName);
    end
    maximumSpeed = double(cfg.operating.maximumSpeed);
    maximumAcceleration = double(cfg.operating.maximumAcceleration);
    maximumRate = double(cfg.operating.maximumTrackAngleRate);
    sigma = double(cfg.observer.sigma);
    exponents = double(cfg.observer.scalingExponents(:));
    minimumHeadingWeight = double(cfg.lidar.minimumHeadingWeight);
    assert(isscalar(maximumSpeed) && isfinite(maximumSpeed) && maximumSpeed > 0.0, ...
        "cfg.operating.maximumSpeed must be positive and finite.");
    assert(isscalar(maximumAcceleration) && isfinite(maximumAcceleration) && maximumAcceleration > 0.0, ...
        "cfg.operating.maximumAcceleration must be positive and finite.");
    assert(isscalar(maximumRate) && isfinite(maximumRate) && maximumRate > 0.0, ...
        "cfg.operating.maximumTrackAngleRate must be positive and finite.");
    assert(isscalar(sigma) && isfinite(sigma) && sigma >= 1.0, ...
        "cfg.observer.sigma must be finite and at least one.");
    assert(isequal(exponents, [1; 2; 3; 1; 2; 3; 1]), ...
        "cfg.observer.scalingExponents must equal [1;2;3;1;2;3;1].");
    assert(isscalar(minimumHeadingWeight) && isfinite(minimumHeadingWeight) && ...
        minimumHeadingWeight > 0.0 && minimumHeadingWeight <= 1.0, ...
        "cfg.lidar.minimumHeadingWeight must lie in (0, 1].");
end
