function certificate = designLidarOnlyTimerCertificate(fixtureFile, options)
% designLidarOnlyTimerCertificate Synthesize an aperiodic LiDAR-only candidate.
% Requires YALMIP and SeDuMi already on the MATLAB path. This research routine
% uses the recorded base pose gain, optionally scales the invariant gain,
% and DOES NOT install the candidate in the production observer. A solver
% status alone never establishes feasibility: verify every recovered matrix.

    arguments
        fixtureFile (1, 1) string = ""
        options.InvariantScale (1, 1) double {mustBeNonnegative} = 0.1
        options.MinimumXYWeight (1, 1) double {mustBePositive} = 1
        options.MinimumInterval (1, 1) double {mustBePositive} = 0.08
        options.MaximumInterval (1, 1) double {mustBePositive} = 0.11
        options.MaximumIterations (1, 1) double {mustBeInteger, mustBePositive} = 25
    end
    root = fileparts(fileparts(mfilename("fullpath")));
    setupVehicleLocalization();
    if strlength(fixtureFile) == 0
        fixtureFile = fullfile(root, "research", "results", ...
            "lidar_information_conditions_20260907", "source_design.json");
    end
    assert(exist('sdpvar', 'file') ~= 0 && exist('sedumi', 'file') ~= 0, ...
        'Add YALMIP and SeDuMi to the path before synthesis.');
    d = jsondecode(fileread(fixtureFile));
    d.observer.sigma = d.observer.theta;
    b = buildImprovedObserverCertificateData(d);
    theta = d.observer.theta;
    knots = unique([0:0.01:options.MaximumInterval, 0.03, ...
        options.MinimumInterval, options.MaximumInterval]);
    assert(options.MinimumInterval >= 0.03 && options.MaximumInterval >= options.MinimumInterval);
    assert(options.MinimumXYWeight <= 1);
    n = numel(knots);
    rate = 0.01;
    weights = [options.MinimumXYWeight, 1];
    active = [1, 1, 1, 1; b.outputVertexCount, b.fVertexCount, 2, 2; 4000, 2, 1, 1];
    certificate = struct('passed', false, 'productionImplemented', false);
    for iteration = 1:options.MaximumIterations
        yalmip('clear');
        P = sdpvar(7, 7, n, 'symmetric');
        margin = sdpvar(1);
        constraints = [trace(P(:, :, 1)) == 7, margin >= 1e-7, margin <= 1];
        for k = 1:n
            constraints = [constraints, P(:, :, k) >= 1e-5 * eye(7)]; %#ok<AGROW>
            if knots(k) >= options.MinimumInterval - 1e-12
                constraints = [constraints, P(:, :, 1) <= P(:, :, k) - 1e-4 * eye(7)]; %#ok<AGROW>
            end
        end
        for j = 1:size(active, 1)
            h = active(j, 1); f = active(j, 2); w = active(j, 3); iw = active(j, 4);
            off = theta * (b.A + b.fVertices(:, :, f) ...
                + options.InvariantScale * d.N * b.outputVertices(:, :, h));
            omega = b.omegaVertices(:, :, w);
            omega(1, 1) = weights(iw); omega(2, 2) = weights(iw);
            on = off - theta * d.K * omega * b.Cb;
            for k = 1:n-1
                if knots(k) < 0.03 - 1e-12, A = on; else, A = off; end
                derivativeP = (P(:, :, k+1) - P(:, :, k)) / (knots(k+1) - knots(k));
                for endpoint = [k, k+1]
                    Pe = P(:, :, endpoint);
                    Q = A.' * Pe + Pe * A + derivativeP + 2 * rate * Pe;
                    constraints = [constraints, Q <= -margin * eye(7)]; %#ok<AGROW>
                end
            end
        end
        solution = optimize(constraints, -margin, sdpsettings('solver', 'sedumi', 'verbose', 0));
        certificate.status = solution.problem;
        certificate.active = active;
        % Numerical-difficulty status may contain a usable feasible point.
        % Accept it only after the same independent exhaustive checks.
        if solution.problem ~= 0 && solution.problem ~= 4, return; end
        certificate.P = value(P);
        if any(~isfinite(certificate.P), 'all'), return; end
        certificate.knots = knots;
        certificate.theta = theta;
        certificate.nFactor = options.InvariantScale;
        certificate.wMin = options.MinimumXYWeight;
        certificate.tMin = options.MinimumInterval;
        certificate.tMax = options.MaximumInterval;
        certificate.onTime = 0.03;
        certificate.rate = rate;
        certificate.margin = value(margin);
        certificate.iteration = iteration;
        check = verifyLidarOnlyTimerCertificate(certificate, d);
        certificate.verification = check;
        certificate.passed = check.passed;
        fprintf('Timer iteration %d, solver %d, full-box flow maximum %.9g\n', ...
            iteration, solution.problem, check.maximumFlowEigenvalue);
        if check.passed, return; end
        witness = check.worstCombination;
        iw = find(abs(weights - witness(4)) < 1e-12, 1);
        newCombination = [witness(1:3), iw];
        if ismember(newCombination, active, 'rows'), return; end
        active = [active; newCombination]; %#ok<AGROW>
    end
end
