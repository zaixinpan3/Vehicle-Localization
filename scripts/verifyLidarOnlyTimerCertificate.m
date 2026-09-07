function result = verifyLidarOnlyTimerCertificate(certificate, fixture, outputFile)
% verifyLidarOnlyTimerCertificate Exhaustively verify a research candidate.
% P is piecewise affine in time since a qualified pose. The first ONTIME
% seconds use the base full-pose gain; later times have no pose correction.
% A reset may occur anywhere in [TMIN,TMAX]. Affine flow and reset conditions
% are checked at every time-segment endpoint and every model-box vertex.
% This checks ideal continuous flows; delay, noise, numerical integration and
% invariance of the operating region require the assumptions in the report.

    arguments
        certificate = ""
        fixture = ""
        outputFile (1, 1) string = ""
    end
    root = fileparts(fileparts(mfilename("fullpath")));
    setupVehicleLocalization();
    folder = fullfile(root, "research", "results", "lidar_information_conditions_20260907");
    if ~isstruct(certificate)
        if strlength(certificate) == 0, certificate = fullfile(folder, "timer_certificate.json"); end
        certificate = jsondecode(fileread(certificate));
    end
    if ~isstruct(fixture)
        if strlength(fixture) == 0, fixture = fullfile(folder, "source_design.json"); end
        fixture = jsondecode(fileread(fixture));
    end
    c = certificate;
    fixture.observer.theta = c.theta;
    fixture.observer.sigma = c.theta;
    b = buildImprovedObserverCertificateData(fixture);
    knots = c.knots(:).';
    P = c.P;
    assert(size(P, 3) == numel(knots) && all(diff(knots) > 0));
    assert(knots(1) == 0 && abs(knots(end) - c.tMax) < 1e-12);
    assert(any(abs(knots - c.onTime) < 1e-12) && any(abs(knots - c.tMin) < 1e-12));
    assert(c.tMin >= c.onTime && c.wMin > 0 && c.wMin <= 1 && c.rate > 0);
    assert(all(isfinite(P), 'all'));
    symmetryError = max(abs(P - permute(P, [2, 1, 3])), [], 'all');
    minP = Inf;
    maxP = -Inf;
    maxJump = -Inf;
    for k = 1:numel(knots)
        eigenvalues = eig((P(:, :, k) + P(:, :, k).') / 2);
        minP = min(minP, min(eigenvalues));
        maxP = max(maxP, max(eigenvalues));
        if knots(k) >= c.tMin - 1e-12
            jump = P(:, :, 1) - P(:, :, k);
            maxJump = max(maxJump, max(eig((jump + jump.') / 2)));
        end
    end
    worst = -Inf;
    offLogNorm = -Inf;
    onLogNorm = -Inf;
    witness = [];
    count = 0;
    weights = unique([c.wMin, 1]);
    for h = 1:b.outputVertexCount
        for f = 1:b.fVertexCount
            off = c.theta * (b.A + b.fVertices(:, :, f) ...
                + c.nFactor * fixture.N * b.outputVertices(:, :, h));
            offLogNorm = max(offLogNorm, max(eig((off + off.') / 2)));
            for w = 1:b.omegaVertexCount
                for xy = weights
                    omega = b.omegaVertices(:, :, w);
                    omega(1, 1) = xy;
                    omega(2, 2) = xy;
                    on = off - c.theta * fixture.K * omega * b.Cb;
                    onLogNorm = max(onLogNorm, max(eig((on + on.') / 2)));
                    for k = 1:numel(knots)-1
                        if knots(k) < c.onTime - 1e-12, A = on; else, A = off; end
                        derivativeP = (P(:, :, k+1) - P(:, :, k)) / (knots(k+1) - knots(k));
                        for endpoint = [k, k+1]
                            Pe = P(:, :, endpoint);
                            Q = A.' * Pe + Pe * A + derivativeP + 2 * c.rate * Pe;
                            value = max(eig((Q + Q.') / 2));
                            count = count + 1;
                            if value > worst
                                worst = value;
                                witness = [h, f, w, xy, k, endpoint];
                            end
                        end
                    end
                end
            end
        end
    end
    result = struct('passed', symmetryError < 1e-12 && minP > 1e-5 ...
        && worst < -1e-6 && maxJump < -1e-6, ...
        'maximumFlowEigenvalue', worst, 'minimumPEigenvalue', minP, ...
        'maximumPEigenvalue', maxP, 'maximumResetEigenvalue', maxJump, ...
        'symmetryError', symmetryError, 'flowInequalityCount', count, ...
        'worstCombination', witness, 'RDecayRatePerSecond', c.rate, ...
        'predictionEuclideanLogNormPerSecond', offLogNorm, ...
        'ideal150msPredictionAmplification', exp(0.15 * offLogNorm), ...
        'onEuclideanLogNormPerSecond', onLogNorm, ...
        'ideal150msReplayTailAmplification', exp(0.15 * offLogNorm ...
            + min(c.onTime, 0.15) * max(0, onLogNorm - offLogNorm)), ...
        'minimumQualifiedIntervalSeconds', c.tMin, 'maximumQualifiedIntervalSeconds', c.tMax, ...
        'pulseDurationSeconds', c.onTime, 'solverStatus', c.status, ...
        'solverOptimalityClaimed', false, 'productionImplemented', false);
    if strlength(outputFile) > 0
        fid = fopen(outputFile, 'w');
        assert(fid >= 0, 'Cannot open verification output.');
        cleanup = onCleanup(@() fclose(fid));
        fprintf(fid, '%s\n', jsonencode(result, PrettyPrint=true));
    end
end
