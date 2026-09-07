function result = analyzeLidarInformationConditions(fixtureFile, outputFolder)
% analyzeLidarInformationConditions Audit the recorded observer's stability.
% This research audit does not change or certify the production observer.
% It enumerates the complete mean-value/known-input box, derives fixed-P
% information thresholds, and checks a physically consistent counterexample.
% The committed JSON fixture permits reproduction without the recorded bag.

    arguments
        fixtureFile (1, 1) string = ""
        outputFolder (1, 1) string = ""
    end
    root = fileparts(fileparts(mfilename("fullpath")));
    setupVehicleLocalization();
    if strlength(fixtureFile) == 0
        fixtureFile = fullfile(root, "research", "results", ...
            "lidar_information_conditions_20260907", "source_design.json");
    end
    d = jsondecode(fileread(fixtureFile));
    b = buildImprovedObserverCertificateData(d);
    P = d.P;
    Y = P * d.K;
    noise = Y * (d.X \ Y.');
    position = [1, 4];
    hidden = [2, 3, 5, 6, 7];
    originalThreshold = -Inf;
    originalHiddenMaximum = -Inf;
    gnssCoreMaximum = -Inf;
    outageCoreMaximum = -Inf;
    for h = 1:b.outputVertexCount
        for f = 1:b.fVertexCount
            A = b.A + b.fVertices(:, :, f) + d.N * b.outputVertices(:, :, h);
            Q = P * A + A.' * P + d.lambda * eye(7) + noise;
            for w = 1:b.omegaVertexCount
                omega = b.omegaVertices(:, :, w);
                feedback = Y * omega * b.Cb;
                gnssCoreMaximum = max(gnssCoreMaximum, largest(Q - feedback - feedback.'));
                omega(1:2, 1:2) = 0;
                feedback = Y * omega * b.Cb;
                M = Q - feedback - feedback.';
                originalHiddenMaximum = max(originalHiddenMaximum, largest(M(hidden, hidden)));
                threshold = schurThreshold(M, position, hidden);
                originalThreshold = max(originalThreshold, threshold);
                outageCoreMaximum = max(outageCoreMaximum, largest(M - 2 * b.Cl.' * b.Cl));
            end
        end
    end

    % Rebuild at the actual theta, removing the original decay/noise budget.
    d.observer.sigma = d.observer.theta;
    b = buildImprovedObserverCertificateData(d);
    theta = d.observer.theta;
    R = chol(P);
    weights = [0, 1, 2, 4, 8, 16, 32, 64, 128];
    rates = -Inf(size(weights));
    offRate = -Inf;
    gnssRate = -Inf;
    homogeneousThreshold = -Inf;
    homogeneousHiddenMaximum = -Inf;
    boxSpectralAbscissa = -Inf;
    for h = 1:b.outputVertexCount
        for f = 1:b.fVertexCount
            A = b.A + b.fVertices(:, :, f) + d.N * b.outputVertices(:, :, h);
            Q = P * A + A.' * P;
            offRate = max(offRate, theta * largest(R.' \ Q / R));
            fullLidar = A - d.K * diag([0, 0, 1]) * b.Cb - (P \ b.Cl.') * b.Cl;
            boxSpectralAbscissa = max(boxSpectralAbscissa, theta * max(real(eig(fullLidar))));
            for w = 1:b.omegaVertexCount
                omega = b.omegaVertices(:, :, w);
                feedback = Y * omega * b.Cb;
                gnssRate = max(gnssRate, theta * largest(R.' \ (Q - feedback - feedback.') / R));
                omega(1:2, 1:2) = 0;
                feedback = Y * omega * b.Cb;
                M = Q - feedback - feedback.';
                homogeneousHiddenMaximum = max(homogeneousHiddenMaximum, largest(M(hidden, hidden)));
                homogeneousThreshold = max(homogeneousThreshold, schurThreshold(M, position, hidden));
                for k = 1:numel(weights)
                    weighted = M - 2 * weights(k) * (b.Cl.' * b.Cl);
                    rates(k) = max(rates(k), theta * largest(R.' \ weighted / R));
                end
            end
        end
    end
    fractions = offRate ./ (offRate - rates);
    fractions(rates >= 0) = NaN;
    modeRates = table(weights.', rates.', fractions.', ...
        'VariableNames', {'effectiveXYWeight', 'maximumVRatePerSecond', 'sufficientOnFraction'});

    % Exact straight motion is a constant error-system equilibrium in moving
    % coordinates. Finite differences call the production channel evaluator.
    speed = 16;
    psi = -3 * pi / 4;
    z = [0; speed * cos(psi); 0; 0; speed * sin(psi); 0; psi];
    sample = struct('longitudinalSpeed', speed, 'lateralVelocity', 0, ...
        'longitudinalAcceleration', 0, 'lateralAcceleration', 0, ...
        'yawRate', 0, 'sideSlipAngle', 0, 'sideSlipAngleRate', 0);
    T = b.Tsigma;
    H = zeros(4, 7);
    H(1, [2, 5]) = 2 * z([2, 5]);
    H(2, [3, 6]) = z([2, 5]);
    H(3, [3, 6]) = [-z(5), z(2)];
    H(4, [2, 5, 7]) = [-sin(psi), cos(psi), -speed];
    J = theta * T * (b.A - d.K * diag([0, 0, 1]) * b.Cb ...
        - (P \ b.Cl.') * b.Cl) / T - T * d.N / theta^3 * H;
    finiteDifferenceSteps = [1e-4, 1e-5, 1e-6];
    finiteDifferenceError = zeros(size(finiteDifferenceSteps));
    finiteDifferenceAbscissa = zeros(size(finiteDifferenceSteps));
    for k = 1:numel(finiteDifferenceSteps)
        step = finiteDifferenceSteps(k);
        Jfd = zeros(7);
        for j = 1:7
            offset = zeros(7, 1);
            offset(j) = step;
            Jfd(:, j) = (errorDerivative(offset) - errorDerivative(-offset)) / (2 * step);
        end
        finiteDifferenceError(k) = norm(Jfd - J, 'fro');
        finiteDifferenceAbscissa(k) = max(real(eig(Jfd)));
    end
    counterexample = table(finiteDifferenceSteps.', finiteDifferenceError.', ...
        finiteDifferenceAbscissa.', 'VariableNames', ...
        {'step', 'jacobianFrobeniusError', 'spectralAbscissaPerSecond'});
    result = struct('originalFixedPThreshold', originalThreshold, ...
        'originalHiddenMaximumEigenvalue', originalHiddenMaximum, ...
        'originalGnssCoreMaximumEigenvalue', gnssCoreMaximum, ...
        'originalOutageUnitLidarCoreMaximumEigenvalue', outageCoreMaximum, ...
        'actualThetaHomogeneousThreshold', homogeneousThreshold, ...
        'actualThetaHiddenMaximumEigenvalue', homogeneousHiddenMaximum, ...
        'allPoseOffVRatePerSecond', offRate, 'continuousGnssAndHeadingVRatePerSecond', gnssRate, ...
        'outageUnitLidarBoxSpectralAbscissaPerSecond', boxSpectralAbscissa, ...
        'physicalCounterexampleSpeedMps', speed, 'physicalCounterexampleHeadingRad', psi, ...
        'physicalCounterexampleSpectralAbscissaPerSecond', max(real(eig(J))), ...
        'physicalCounterexampleEquilibriumResidual', norm(errorDerivative(zeros(7, 1))), ...
        'largestFiniteDifferenceError', max(finiteDifferenceError), ...
        'robustVertexCount', b.totalVertexCount, 'productionModified', false);
    assert(result.originalGnssCoreMaximumEigenvalue < 0);
    assert(originalThreshold > 1 && homogeneousThreshold > 1);
    assert(result.physicalCounterexampleSpectralAbscissaPerSecond > 0);
    assert(result.physicalCounterexampleEquilibriumResidual < 1e-10);
    assert(max(finiteDifferenceError) < 1e-5);
    if strlength(outputFolder) > 0
        if ~isfolder(outputFolder), mkdir(outputFolder); end
        writetable(modeRates, fullfile(outputFolder, "mode_rates.csv"));
        writetable(counterexample, fullfile(outputFolder, "counterexample.csv"));
        fid = fopen(fullfile(outputFolder, "audit_metrics.json"), 'w');
        assert(fid >= 0, 'Cannot open audit output.');
        cleanup = onCleanup(@() fclose(fid));
        fprintf(fid, '%s\n', jsonencode(result, PrettyPrint=true));
    end

    function value = errorDerivative(error)
        predicted = evaluateImprovedObserverChannels(z + error, sample);
        truth = evaluateImprovedObserverChannels(z, sample);
        value = predicted.modelDerivative - truth.modelDerivative ...
            - T * d.K * diag([0, 0, 1]) * b.Cb * error ...
            - T * (P \ b.Cl.') * b.Cl * error ...
            + T * d.N / theta^3 * predicted.invariantInnovation;
    end
end

function value = largest(matrix)
    value = max(eig((matrix + matrix.') / 2));
end

function value = schurThreshold(matrix, position, hidden)
    if largest(matrix(hidden, hidden)) >= 0
        value = Inf;
        return;
    end
    value = largest(matrix(position, position) - matrix(position, hidden) * ...
        (matrix(hidden, hidden) \ matrix(hidden, position))) / 2;
end
