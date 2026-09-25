classdef lateralObserverTest < matlab.unittest.TestCase
% lateralObserverTest: Tests of the LPV lateral-velocity observer. The model
% and scheduling tests check the algebra that makes the polytopic
% representation exact; the observer tests run against a stored design so
% they need no SDP solver; the synthesis test re-runs the LMI and therefore
% requires YALMIP.

    properties (Access = private)
        StoredDesign
    end

    methods (TestClassSetup)
        function addProjectPaths(testCase)
        % addProjectPaths: Put the vehicleLocalization modules on the path and
        % load the stored observer design.
        %
        % Input:
        %   testCase: matlab.unittest.TestCase instance
        %
        % Output:
        %   none
            projectFolder = fileparts(fileparts(mfilename("fullpath")));
            run(fullfile(projectFolder, "setupVehicleLocalization.m"));
            loaded = load(fullfile(projectFolder, "tests", "reference", "lateralObserverDesign.mat"));
            testCase.StoredDesign = loaded.design;
            current=lateralObserverConfig();
            testCase.StoredDesign.cfg.hybrid=current.hybrid;
            testCase.StoredDesign.cfg.outputPoint=current.outputPoint;
        end
    end

    methods (Test)
        function modelMatchesHandDerivedBicycleModel(testCase)
        % modelMatchesHandDerivedBicycleModel: The affine model reproduces the
        % textbook 2-DOF lateral bicycle matrices, and the first row of A and C
        % differ exactly by the centripetal term, which is the identity
        % vydot = ay - Vx r.
            cfg = lateralObserverConfig();
            vehicle = cfg.vehicle;
            model = lateralBicycleModel(vehicle);
            longitudinalSpeed = 15.0;
            [A, C] = evaluateLateralModel(model, [longitudinalSpeed; 1.0 ./ longitudinalSpeed]);

            stiffnessSum = vehicle.frontCorneringStiffness + vehicle.rearCorneringStiffness;
            stiffnessMoment = (vehicle.lr .* vehicle.rearCorneringStiffness) - ...
                (vehicle.lf .* vehicle.frontCorneringStiffness);
            stiffnessSecondMoment = (vehicle.lf.^2 .* vehicle.frontCorneringStiffness) + ...
                (vehicle.lr.^2 .* vehicle.rearCorneringStiffness);
            expectedA = [-stiffnessSum ./ (vehicle.mass .* longitudinalSpeed), ...
                (stiffnessMoment ./ (vehicle.mass .* longitudinalSpeed)) - longitudinalSpeed; ...
                stiffnessMoment ./ (vehicle.yawInertia .* longitudinalSpeed), ...
                -stiffnessSecondMoment ./ (vehicle.yawInertia .* longitudinalSpeed)];

            testCase.verifyEqual(A, expectedA, AbsTol=1.0e-12);
            testCase.verifyEqual(A(1, :), C(1, :) - [0.0, longitudinalSpeed], AbsTol=1.0e-12);
            testCase.verifyEqual(C(2, :), [0.0, 1.0], AbsTol=1.0e-12);
            testCase.verifyEqual(model.D(1), vehicle.frontCorneringStiffness ./ vehicle.mass, AbsTol=1.0e-12);
        end

        function schedulingTriangleCoversTheSpeedRange(testCase)
        % schedulingTriangleCoversTheSpeedRange: The barycentric coordinates of
        % every scheduling point in the speed range are nonnegative and sum to
        % one, which is exactly the statement that the triangle contains the
        % curve rho(Vx) = [Vx; 1/Vx].
            cfg = lateralObserverConfig();
            polytope = buildSchedulingPolytope(cfg.scheduling.speedRange);
            speeds = linspace(cfg.scheduling.speedRange(1), cfg.scheduling.speedRange(2), 401);
            for speed = speeds
                alpha = schedulingCoordinates(polytope, speed);
                testCase.verifyGreaterThanOrEqual(alpha, -1.0e-12 .* ones(3, 1));
                testCase.verifyEqual(sum(alpha), 1.0, AbsTol=1.0e-12);
                testCase.verifyEqual(polytope.vertices * alpha, [speed; 1.0 ./ speed], AbsTol=1.0e-10);
            end
        end

        function polytopicReconstructionIsExact(testCase)
        % polytopicReconstructionIsExact: Because A and C are affine in rho,
        % their values equal the convex combination of the vertex values with
        % the same barycentric coordinates.
            cfg = lateralObserverConfig();
            model = lateralBicycleModel(cfg.vehicle);
            polytope = buildSchedulingPolytope(cfg.scheduling.speedRange);
            vertexA = zeros(2, 2, 3);
            vertexC = zeros(2, 2, 3);
            for vertexIdx = 1:3
                [vertexA(:, :, vertexIdx), vertexC(:, :, vertexIdx)] = ...
                    evaluateLateralModel(model, polytope.vertices(:, vertexIdx));
            end

            for speed = linspace(cfg.scheduling.speedRange(1), cfg.scheduling.speedRange(2), 51)
                alpha = schedulingCoordinates(polytope, speed);
                [A, C] = evaluateLateralModel(model, [speed; 1.0 ./ speed]);
                blendedA = (alpha(1) .* vertexA(:, :, 1)) + (alpha(2) .* vertexA(:, :, 2)) + (alpha(3) .* vertexA(:, :, 3));
                blendedC = (alpha(1) .* vertexC(:, :, 1)) + (alpha(2) .* vertexC(:, :, 2)) + (alpha(3) .* vertexC(:, :, 3));
                testCase.verifyEqual(blendedA, A, AbsTol=1.0e-10);
                testCase.verifyEqual(blendedC, C, AbsTol=1.0e-10);
            end
        end

        function storedDesignSatisfiesTheOriginalCertificate(testCase)
        % storedDesignSatisfiesTheOriginalCertificate: The stored vertex gains
        % satisfy the pre-convexification conditions of the proposition at
        % every grid speed: negative definite Lyapunov derivative, H2 quantity
        % below mu, and a stable error matrix. The check is repeated here with
        % the blended gain so the stored margins are not taken on trust.
            design = testCase.StoredDesign;
            testCase.verifyTrue(design.certified);
            testCase.verifyLessThan(design.maxCertificateMargin, 0);
            testCase.verifyLessThan(design.maxErrorEigenvalueRealPart, 0);
            testCase.verifyLessThan(max(design.h2Values), design.h2Bound);
            testCase.verifyEqual(size(design.vertexGains), [2, 2, 3]);
            testCase.verifyTrue(all(isfinite(design.vertexGains(:))));

            P = design.lyapunovMatrix;
            testCase.verifyGreaterThan(min(eig((P + P.') ./ 2.0)), 0);
            for point = design.grid.points.'
                L = scheduleLateralObserverGain(design, point.speed);
                closedLoop = point.A - (L * point.C);
                certificate = (P * closedLoop) + (closedLoop.' * P) + design.errorWeight + ...
                    (design.tau .* (P * P)) + ((design.lipschitzConstant.^2 ./ design.tau) .* eye(2));
                testCase.verifyLessThan(max(eig((certificate + certificate.') ./ 2.0)), 0);
                testCase.verifyLessThan(trace(L.' * P * L), design.h2Bound);
            end
        end

        function scheduledGainIsTheBarycentricBlendOfTheVertexGains(testCase)
        % scheduledGainIsTheBarycentricBlendOfTheVertexGains: The online gain
        % is the vertex gain itself at the two speed vertices, the barycentric
        % blend of the three vertex gains at an interior speed, and the nearer
        % vertex gain when the speed leaves the designed range.
            design = testCase.StoredDesign;
            polytope = design.polytope;
            vertexGains = design.vertexGains;

            atMinimum = scheduleLateralObserverGain(design, polytope.speedRange(1));
            testCase.verifyEqual(atMinimum, vertexGains(:, :, 1), AbsTol=1.0e-10);
            atMaximum = scheduleLateralObserverGain(design, polytope.speedRange(2));
            testCase.verifyEqual(atMaximum, vertexGains(:, :, 2), AbsTol=1.0e-10);

            interiorSpeed = 12.3;
            alpha = schedulingCoordinates(polytope, interiorSpeed);
            expected = (alpha(1) .* vertexGains(:, :, 1)) + (alpha(2) .* vertexGains(:, :, 2)) + ...
                (alpha(3) .* vertexGains(:, :, 3));
            testCase.verifyEqual(scheduleLateralObserverGain(design, interiorSpeed), expected, AbsTol=1.0e-12);
            testCase.verifyGreaterThan(min(alpha), 0, "The interior speed should use all three vertices.");

            belowRange = scheduleLateralObserverGain(design, polytope.speedRange(1) - 5.0);
            testCase.verifyEqual(belowRange, atMinimum, AbsTol=1.0e-12);
            aboveRange = scheduleLateralObserverGain(design, polytope.speedRange(2) + 5.0);
            testCase.verifyEqual(aboveRange, atMaximum, AbsTol=1.0e-12);
        end

        function scheduledGainIsAffineInTheSchedulingParameter(testCase)
        % scheduledGainIsAffineInTheSchedulingParameter: Because the blend uses
        % the barycentric coordinates of rho = [Vx; 1/Vx], the gain is affine
        % in rho: the blended gain along the speed range equals the least
        % squares affine fit L0 + rho1 L1 + rho2 L2 to numerical precision.
            design = testCase.StoredDesign;
            speeds = linspace(design.polytope.speedRange(1), design.polytope.speedRange(2), 41);
            regressors = [ones(numel(speeds), 1), speeds(:), 1.0 ./ speeds(:)];
            gains = zeros(numel(speeds), 4);
            for speedIdx = 1:numel(speeds)
                L = scheduleLateralObserverGain(design, speeds(speedIdx));
                gains(speedIdx, :) = L(:).';
            end
            coefficients = regressors \ gains;
            residual = gains - (regressors * coefficients);
            testCase.verifyLessThan(max(abs(residual(:))), 1.0e-10);
        end

        function observerConvergesFromAWrongInitialState(testCase)
        % observerConvergesFromAWrongInitialState: On the synthetic scenario the
        % observer recovers the lateral velocity from a deliberately wrong
        % initial state and settles well below the peak of the signal.
            design = testCase.StoredDesign;
            result = simulateLateralObserverScenario(design, design.cfg);

            testCase.verifyGreaterThan(result.metrics.peakLateralVelocity, 0.2);
            testCase.verifyLessThan(result.metrics.settledLateralVelocityRmse, ...
                0.05 .* result.metrics.peakLateralVelocity);
            testCase.verifyLessThan(result.metrics.settledYawRateRmse, 0.01);
            testCase.verifyTrue(all(isfinite(result.estimate.state(:))));
        end

        function observerToleratesLipschitzBoundedNonlinearity(testCase)
        % observerToleratesLipschitzBoundedNonlinearity: With a nonlinearity
        % whose Lipschitz constant respects the design bound, the observer still
        % converges, which is what the Young-inequality term of the certificate
        % is there to guarantee.
            design = testCase.StoredDesign;
            cfg = design.cfg;
            nonlinearGain = 0.3;
            cfg.observer.nonlinearity = @(x) [nonlinearGain .* sin(x(2)); -0.2 .* sin(x(1))];
            testCase.assertLessThanOrEqual(nonlinearGain, design.lipschitzConstant);

            result = simulateLateralObserverScenario(design, cfg);
            testCase.verifyLessThan(result.metrics.settledLateralVelocityRmse, ...
                0.05 .* result.metrics.peakLateralVelocity);
            testCase.verifyTrue(all(isfinite(result.estimate.state(:))));
        end

        function sideSlipRateIsConsistentWithTheSideSlipAngle(testCase)
        % sideSlipRateIsConsistentWithTheSideSlipAngle: The persistent
        % side-slip interface state's rate agrees with a finite difference of
        % its reported angle. This is the signal the ego-state observer
        % consumes as part of the track-angle rate.
            design = testCase.StoredDesign;
            result = simulateLateralObserverScenario(design, design.cfg);
            estimate = result.estimate;

            sampleTime = design.cfg.simulation.sampleTime;
            numericalRate = gradient(estimate.sideSlipAngle, sampleTime);
            settled = estimate.time >= 0.25 .* design.cfg.simulation.tFinal;
            rateDeviation = estimate.sideSlipAngleRate(settled) - numericalRate(settled);

            testCase.verifyLessThan(sqrt(mean(rateDeviation.^2)), ...
                0.05 .* max(abs(estimate.sideSlipAngleRate(settled))));
            testCase.verifyTrue(all(isfinite(estimate.sideSlipAngleRate)));
        end

        function speedRateIsReconstructedFromTheSpecificForce(testCase)
        % speedRateIsReconstructedFromTheSpecificForce: The observer treats the
        % longitudinal measurement as the inertial specific force and rebuilds
        % the speed derivative as ax + vy r, which enters the side-slip rate.
            design = testCase.StoredDesign;
            result = simulateLateralObserverScenario(design, design.cfg);

            expectedRate = result.measurements.longitudinalAcceleration + ...
                (result.estimate.lateralVelocity .* result.measurements.yawRate);
            testCase.verifyEqual(result.estimate.longitudinalSpeedRate, expectedRate, AbsTol=1.0e-12);
            testCase.verifyGreaterThan(max(abs(result.estimate.longitudinalSpeedRate - ...
                result.measurements.longitudinalAcceleration)), 0, ...
                "The scenario should exercise a nonzero vy*r correction.");
        end

        function outputPointTransportFollowsRigidBodyKinematics(testCase)
        % outputPointTransportFollowsRigidBodyKinematics: The exported lateral
        % velocity is the master state transported to the declared output
        % point, vyOutput = vyObserver - forwardOffset*r, and the side-slip
        % interface tracks the direction of that output-point velocity. A zero
        % offset exports the observer point unchanged.
            design = testCase.StoredDesign;
            cfg = design.cfg;
            measurements = lateralObserverTest.constantSpeedMeasurements(6.0, 12.0);
            measurements.yawRate(:) = 0.20;
            measurements.lateralAcceleration(:) = 12.0 .* 0.20;

            cfg.outputPoint = struct("forwardOffsetM", 0.0, "identifier", "observer-point");
            atObserver = runLateralVelocityObserver(measurements, design, cfg);
            testCase.verifyEqual(atObserver.lateralVelocity, atObserver.observerPointLateralVelocity, AbsTol=0);

            cfg.outputPoint = struct("forwardOffsetM", 2.5, "identifier", "test-point");
            atOutput = runLateralVelocityObserver(measurements, design, cfg);
            testCase.verifyEqual(atOutput.observerPointLateralVelocity, atObserver.observerPointLateralVelocity, AbsTol=0);
            testCase.verifyEqual(atOutput.lateralVelocity, ...
                atOutput.observerPointLateralVelocity - 2.5 .* measurements.yawRate, AbsTol=1.0e-12);
            settled = measurements.time >= 4.0;
            expectedAngle = atan2(atOutput.lateralVelocity(settled), measurements.longitudinalSpeed(settled));
            testCase.verifyEqual(atOutput.sideSlipAngle(settled), expectedAngle, AbsTol=2.0e-3);
            testCase.verifyGreaterThan(min(abs(atOutput.sideSlipAngle(settled) - atObserver.sideSlipAngle(settled))), 0.03);
            testCase.verifyEqual(atOutput.outputPoint.forwardOffsetM, 2.5);
        end

        function missingOutputPointConfigurationIsRejected(testCase)
        % missingOutputPointConfigurationIsRejected: A configuration without
        % the output-point group is an obsolete contract, not a zero offset.
            design = testCase.StoredDesign;
            cfg = rmfield(design.cfg, "outputPoint");
            measurements = lateralObserverTest.constantSpeedMeasurements(1.0, 8.0);
            testCase.verifyError(@() runLateralVelocityObserver(measurements, design, cfg), ...
                "VehicleLocalization:MissingOutputPointConfiguration");
        end

        function invalidSideSlipSpeedZeroesTheOutputs(testCase)
        % invalidSideSlipSpeedZeroesTheOutputs: Below the valid side-slip speed
        % the persistent interface tracks zero while the common lateral state
        % remains free to converge continuously.
            design = testCase.StoredDesign;
            cfg = design.cfg;
            numSamples = 200;
            measurements = struct();
            measurements.time = (0:(numSamples - 1)).' .* 0.01;
            measurements.steeringAngle = zeros(numSamples, 1);
            measurements.longitudinalSpeed = 0.25 .* cfg.hybrid.sideSlip.validSpeed .* ones(numSamples, 1);
            measurements.longitudinalAcceleration = zeros(numSamples, 1);
            measurements.lateralAcceleration = 0.2 .* ones(numSamples, 1);
            measurements.yawRate = 0.05 .* ones(numSamples, 1);

            estimate = runLateralVelocityObserver(measurements, design, cfg);

            testCase.verifyTrue(all(~estimate.diagnostics.sideSlipCommandValid));
            testCase.verifyEqual(estimate.sideSlipAngle, zeros(numSamples, 1), AbsTol=0);
            testCase.verifyEqual(estimate.sideSlipAngleRate, zeros(numSamples, 1), AbsTol=0);
            testCase.verifyTrue(all(isfinite(estimate.state(:))));
        end

        function zeroSpeedNeverEvaluatesTheReciprocalModel(testCase)
        % zeroSpeedNeverEvaluatesTheReciprocalModel Corrupt the reciprocal
        % coefficients and prove that the zero-speed execution path does not
        % access them.
            design = testCase.StoredDesign;
            design.model.A2(:) = NaN;
            design.model.C2(:) = NaN;
            cfg = lateralObserverConfig();
            cfg.observer.initialState = [1.0; 0.2];
            cfg.hybrid.initialMasterState = [1.0; 0.0];
            measurements = testCase.zeroMotionMeasurements(2.0);

            estimate = runLateralVelocityObserver(measurements, design, cfg);

            testCase.verifyFalse(any(estimate.diagnostics.dynamicModelEvaluated));
            testCase.verifyTrue(all(isfinite(estimate.state), "all"));
            testCase.verifyTrue(all(isfinite(estimate.dynamicState), "all"));
            testCase.verifyEqual(estimate.lateralVelocity(1), 1.0, AbsTol=0.0);
            testCase.verifyLessThan(abs(estimate.lateralVelocity(end)), 0.05);
            testCase.verifyEqual(estimate.sideSlipAngle, ...
                zeros(size(estimate.sideSlipAngle)), AbsTol=0.0);
            testCase.verifyEqual(estimate.sideSlipAngleRate, ...
                zeros(size(estimate.sideSlipAngleRate)), AbsTol=0.0);
        end

        function abruptValidityLossFadesTheStoredCorrection(testCase)
        % abruptValidityLossFadesTheStoredCorrection A one-edge validity loss
        % removes the raw target immediately but not the injected correction.
            cfg = lateralObserverConfig();
            cfg.observer.initialState = [0.0; 0.0];
            cfg.hybrid.initialMasterState = [1.0; 0.0];
            measurements = testCase.constantSpeedMeasurements(3.0, 10.0);
            measurements.dynamicValid = measurements.time < 1.5;
            firstInvalid = find(~measurements.dynamicValid, 1, "first");

            estimate = runLateralVelocityObserver(measurements, testCase.StoredDesign, cfg);

            testCase.verifyFalse(estimate.diagnostics.dynamicModelEvaluated(firstInvalid));
            testCase.verifyEqual(estimate.correctionTarget.dynamic(firstInvalid, :), ...
                zeros(1, 2), AbsTol=0.0);
            testCase.verifyGreaterThan(norm( ...
                estimate.correctionInjection.dynamic(firstInvalid, :)), 1.0e-6);
            testCase.verifyLessThan(norm(estimate.correctionInjection.dynamic(end, :)), ...
                norm(estimate.correctionInjection.dynamic(firstInvalid, :)));
            testCase.verifyLessThan(abs(estimate.lateralVelocity(firstInvalid) - ...
                estimate.lateralVelocity(firstInvalid - 1)), 1.0e-3);
            testCase.verifyTrue(all(isfinite(estimate.sideSlipAngleRate)));
        end

        function stopGoModesShareOneContinuousOutputPath(testCase)
        % stopGoModesShareOneContinuousOutputPath Exercise both speed edges,
        % the stationary detector, and a deliberately mismatched hidden state.
            cfg = lateralObserverConfig();
            cfg.observer.initialState = [0.0; 0.10];
            cfg.hybrid.initialMasterState = [0.80; 0.0];
            measurements = testCase.stopGoMeasurements();

            estimate = runLateralVelocityObserver(measurements, testCase.StoredDesign, cfg);
            transitionStep = [0.0; abs(diff(estimate.lateralVelocity))];
            transitionRateStep = [0.0; abs(diff(estimate.sideSlipAngleRate))];

            testCase.verifyTrue(any(estimate.mode == "stationary"));
            testCase.verifyTrue(any(estimate.mode == "crawl"));
            testCase.verifyTrue(any(estimate.mode == "dynamic"));
            testCase.verifyGreaterThanOrEqual(nnz(estimate.modeChanged), 4);
            testCase.verifyLessThan(max(transitionStep(estimate.modeChanged)), 0.10);
            testCase.verifyLessThan(max(transitionRateStep(estimate.modeChanged)), 0.10);
            testCase.verifyTrue(all(isfinite(estimate.state), "all"));
            testCase.verifyTrue(all(isfinite(estimate.sideSlipAngleRate)));
        end

        function synthesisReproducesTheStoredDesign(testCase)
        % synthesisReproducesTheStoredDesign: Re-running the LMI synthesis
        % reproduces the stored vertex gains. Needs YALMIP and an SDP solver.
            testCase.assumeTrue(exist("sdpvar", "file") == 2, ...
                "The lateral observer synthesis test needs YALMIP and an SDP solver on the MATLAB path.");
            design = testCase.StoredDesign;
            resolved = designLateralObserverGains(design.cfg);

            testCase.verifyEqual(resolved.tau, design.tau);
            testCase.verifyEqual(resolved.h2Bound, design.h2Bound, RelTol=1.0e-4);
            testCase.verifyEqual(resolved.vertexGains, design.vertexGains, AbsTol=1.0e-4);
            testCase.verifyEqual(resolved.lyapunovMatrix, design.lyapunovMatrix, AbsTol=1.0e-4);
            testCase.verifyTrue(resolved.certified);
        end
    end

    methods (Static, Access = private)
        function measurements = zeroMotionMeasurements(finalTime)
        % zeroMotionMeasurements Build a stationary public-interface input.
            time = (0:0.01:finalTime).';
            sampleCount = numel(time);
            measurements = struct("time", time, ...
                "steeringAngle", zeros(sampleCount, 1), ...
                "longitudinalSpeed", zeros(sampleCount, 1), ...
                "longitudinalAcceleration", zeros(sampleCount, 1), ...
                "lateralAcceleration", zeros(sampleCount, 1), ...
                "yawRate", zeros(sampleCount, 1));
        end

        function measurements = constantSpeedMeasurements(finalTime, speed)
        % constantSpeedMeasurements Build a straight constant-speed input.
            time = (0:0.01:finalTime).';
            sampleCount = numel(time);
            measurements = struct("time", time, ...
                "steeringAngle", zeros(sampleCount, 1), ...
                "longitudinalSpeed", speed .* ones(sampleCount, 1), ...
                "longitudinalAcceleration", zeros(sampleCount, 1), ...
                "lateralAcceleration", zeros(sampleCount, 1), ...
                "yawRate", zeros(sampleCount, 1));
        end

        function measurements = stopGoMeasurements()
        % stopGoMeasurements Build dwell, smooth launch, cruise, and stop.
            sampleTime = 0.01;
            time = (0:sampleTime:6.0).';
            riseFraction = min(max((time - 0.50) ./ 1.50, 0.0), 1.0);
            fallFraction = min(max((time - 4.00) ./ 1.50, 0.0), 1.0);
            rise = 6.0 .* riseFraction.^5 - 15.0 .* riseFraction.^4 + ...
                10.0 .* riseFraction.^3;
            fall = 6.0 .* fallFraction.^5 - 15.0 .* fallFraction.^4 + ...
                10.0 .* fallFraction.^3;
            speed = 8.0 .* rise .* (1.0 - fall);
            sampleCount = numel(time);
            measurements = struct("time", time, ...
                "steeringAngle", zeros(sampleCount, 1), ...
                "longitudinalSpeed", speed, ...
                "longitudinalAcceleration", gradient(speed, sampleTime), ...
                "lateralAcceleration", zeros(sampleCount, 1), ...
                "yawRate", zeros(sampleCount, 1));
        end
    end
end
