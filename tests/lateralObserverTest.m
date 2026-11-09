classdef lateralObserverTest < matlab.unittest.TestCase
% lateralObserverTest: Tests of the LPV lateral-velocity observer. The model
% and scheduling tests check the algebra that makes the polytopic
% representation exact. Observer tests synthesize the current MnCAV design
% once per class and require YALMIP and an SDP solver.

    properties (Access = private)
        CurrentDesign
    end

    methods (TestClassSetup)
        function addProjectPaths(testCase)
        % addProjectPaths: Put the vehicleLocalization modules on the path and
        % synthesize the current MnCAV observer design.
        %
        % Input:
        %   testCase: matlab.unittest.TestCase instance
        %
        % Output:
        %   none
            projectFolder = fileparts(fileparts(mfilename("fullpath")));
            run(fullfile(projectFolder, "setupVehicleLocalization.m"));
            testCase.CurrentDesign = designLateralObserverGains(lateralObserverConfig());
        end
    end

    properties (TestParameter)
        integrationMethod = struct('midpoint', "exponentialMidpoint", 'rk4', "rk4", 'euler', "euler")
    end

    methods (Test)
        function virtualObservationDrivesEveryIntegrator(testCase, integrationMethod)
            design = testCase.CurrentDesign;
            cfg = design.cfg;
            cfg.observer.integrationMethod = integrationMethod;
            measurements = testCase.constantSpeedMeasurements(.02, 12);
            baseline = runLateralVelocityObserver(measurements, design, cfg);
            measurements.lateralAcceleration(:) = .1;
            actual = runLateralVelocityObserver(measurements, design, cfg);
            measurements.yawRate(:) = NaN; % Gyro is not a lateral innovation.
            ignoredGyro = runLateralVelocityObserver(measurements, design, cfg);
            testCase.verifyGreaterThan(norm(actual.state - baseline.state), 1e-6);
            testCase.verifyEqual(ignoredGyro.state, actual.state, AbsTol=0);
            testCase.verifyEqual(actual.innovation(1), 1.2, AbsTol=1e-12);
        end

        function missingLateralAccelerationIsRejected(testCase)
            measurements = rmfield(testCase.constantSpeedMeasurements(.1, 12), 'lateralAcceleration');
            testCase.verifyError(@() runLateralVelocityObserver( ...
                measurements, testCase.CurrentDesign, testCase.CurrentDesign.cfg), ...
                'VehicleLocalization:MissingLateralMeasurement');
        end

        function scheduledMetricCertifiesPositionAndRateDomain(testCase)
            design = testCase.CurrentDesign;
            testCase.verifySize(design.vertexY, [2, 1, 3]);
            testCase.verifySize(design.vertexLyapunovMatrices, [2, 2, 3]);
            testCase.verifyEqual(design.tau, 0);
            testCase.verifyLessThan(testCase.maximumCertificateEigenvalue(design), 0);
            testCase.verifyLessThan(design.maximumCertificateEigenvalue, 0);
            testCase.verifyEqual(design.certificateDomain.numVertexPairs, 6);
            testCase.verifyEqual(design.certificateDomain.numRateVertices, 4);
            testCase.verifyFalse(isfield(design, 'stiffnessUncertainty'));
            testCase.verifyFalse(isfield(design.cfg.synthesis, 'stiffnessRelativeErrorBound'));
        end

        function obsoleteTwoOutputGainsAreRejected(testCase)
            design = testCase.CurrentDesign;
            design.vertexGains = zeros(2, 2, 3);
            testCase.verifyError(@() runLateralVelocityObserver( ...
                testCase.constantSpeedMeasurements(.1, 10), design, design.cfg), ...
                'VehicleLocalization:ObsoleteLateralDesign');
        end

        function obsoleteNoiseScalesAreRejected(testCase)
            cfg = testCase.CurrentDesign.cfg;
            cfg.synthesis.measurementErrorScale = [.3; .02];
            testCase.verifyError(@() designLateralObserverGains(cfg), ...
                'VehicleLocalization:ObsoleteLateralConfiguration');
        end

        function linearModelNeedsNoYoungMultiplier(testCase)
            cfg = testCase.CurrentDesign.cfg;
            cfg.synthesis.lipschitzConstant = 0;
            design = designLateralObserverGains(cfg);
            testCase.verifyTrue(design.solverSolved);
            testCase.verifyEqual(design.tau, 0, AbsTol=0);
            testCase.verifyLessThan(testCase.maximumCertificateEigenvalue(design), 0);
        end

        function coordinateTransportPrecedesAlgebraicSideslip(testCase)
            cfg=testCase.CurrentDesign.cfg;
            measurements=testCase.constantSpeedMeasurements(1,10);
            nominal=runLateralVelocityObserver(measurements,testCase.CurrentDesign,cfg);
            transform=repmat([1,0,0,1,1,2],numel(measurements.time),1);
            actual=runLateralVelocityObserver(measurements,testCase.CurrentDesign,cfg,transform);
            angle=atan2(2,11);t=measurements.time;
            testCase.verifyEqual(actual.state,nominal.state,AbsTol=0);
            testCase.verifyEqual(actual.innovation,nominal.innovation,AbsTol=0);
            testCase.verifyEqual(actual.velocityValid,nominal.velocityValid);
            testCase.verifyEqual(actual.longitudinalSpeed,11*ones(size(t)));
            testCase.verifyEqual(actual.lateralVelocity,2*ones(size(t)));
            testCase.verifyEqual(actual.sideSlipAngle,angle*ones(size(t)),AbsTol=1e-12);
            testCase.verifyEqual(actual.sideSlipAngleRate,zeros(size(t)),AbsTol=1e-12);
        end
        function identityOutputMapPreservesTheOriginalInterface(testCase)
            cfg=testCase.CurrentDesign.cfg;m=testCase.stopGoMeasurements();
            a=runLateralVelocityObserver(m,testCase.CurrentDesign,cfg);
            b=runLateralVelocityObserver(m,testCase.CurrentDesign,cfg,repmat([1,0,0,1,0,0],numel(m.time),1));
            testCase.verifyEqual(a,b);
        end
        function modelMatchesHandDerivedBicycleModel(testCase)
            cfg = lateralObserverConfig();
            vehicle = cfg.vehicle;
            model = lateralObserverSupport.lateralBicycleModel(vehicle);
            speed = 15;
            [A, C, D] = lateralObserverSupport.evaluateLateralModel(model, [speed; 1/speed]);
            total = vehicle.frontCorneringStiffness + vehicle.rearCorneringStiffness;
            moment = vehicle.lr*vehicle.rearCorneringStiffness - vehicle.lf*vehicle.frontCorneringStiffness;
            second = vehicle.lf^2*vehicle.frontCorneringStiffness + vehicle.lr^2*vehicle.rearCorneringStiffness;
            expectedA = [-total/(vehicle.mass*speed), moment/(vehicle.mass*speed) - speed; ...
                moment/(vehicle.yawInertia*speed), -second/(vehicle.yawInertia*speed)];
            testCase.verifyEqual(A, expectedA, AbsTol=1e-12);
            testCase.verifyEqual(C, [-total/vehicle.mass, moment/vehicle.mass], AbsTol=1e-12);
            testCase.verifyEqual(D, speed*vehicle.frontCorneringStiffness/vehicle.mass, AbsTol=1e-12);
            state = [.4; -.12];
            steering = .03;
            derivative = A*state + model.B*steering;
            ay = derivative(1) + speed*state(2);
            testCase.verifyEqual(speed*ay, C*state + D*steering, AbsTol=1e-12);
        end

        function knownSteeringFeedthroughCancelsAtZeroState(testCase, integrationMethod)
            design = testCase.CurrentDesign;
            cfg = design.cfg;
            cfg.observer.integrationMethod = integrationMethod;
            measurements = testCase.constantSpeedMeasurements(.01, 17);
            measurements.steeringAngle(:) = .03;
            measurements.lateralAcceleration(:) = design.model.B(1)*.03;
            estimate = runLateralVelocityObserver(measurements, design, cfg);
            testCase.verifyEqual(estimate.innovation(1), 0, AbsTol=1e-12);
            testCase.verifyEqual(estimate.observerPointLateralVelocityRate(1), design.model.B(1)*.03, AbsTol=1e-12);
            testCase.verifyTrue(all(isfinite(estimate.state), 'all'));
        end

        function obsoleteStiffnessDesignIsRejected(testCase)
            design = testCase.CurrentDesign;
            design.certificateMethod = "yaw-rate-stiffness-regressor-iss-v1";
            testCase.verifyError(@() runLateralVelocityObserver( ...
                testCase.constantSpeedMeasurements(.1, 10), design, design.cfg), ...
                'VehicleLocalization:ObsoleteLateralDesign');
        end

        function physicalPlantMinusObserverSatisfiesDissipation(testCase)
            design = testCase.CurrentDesign;
            speeds = linspace(design.polytope.speedRange(1), design.polytope.speedRange(2), 31);
            worst = -Inf;
            for idx = 1:numel(speeds)
                speed = speeds(idx);
                [A, C, D] = lateralObserverSupport.evaluateLateralModel(design.model, [speed; 1/speed]);
                state = [.4*sin(idx); .12*cos(idx)];
                error = [.2*cos(2*idx); .03*sin(2*idx)];
                estimate = state - error;
                delta = .025*sin(.3*idx);
                noise = .1*cos(.7*idx);
                plantRate = A*state + design.model.B*delta;
                measuredAy = plantRate(1) + speed*state(2) + noise;
                for rate = design.certificateDomain.speedRateRange
                    [L, P, Pdot] = lateralObserverSupport.scheduleLateralObserverGain(design, speed, rate);
                    observerRate = A*estimate + design.model.B*delta + ...
                        L*(speed*measuredAy - C*estimate - D*delta);
                    xi = noise/design.measurementErrorScale;
                    residual = 2*error.'*P*(plantRate - observerRate) + error.'*Pdot*error + ...
                        2*design.decayRate*(error.'*P*error) - design.issGainLimit^2*xi^2;
                    worst = max(worst, residual);
                end
            end
            testCase.verifyLessThanOrEqual(worst, 1e-8);
        end

        function schedulingTriangleCoversTheSpeedRange(testCase)
        % schedulingTriangleCoversTheSpeedRange: The barycentric coordinates of
        % every scheduling point in the speed range are nonnegative and sum to
        % one, which is exactly the statement that the triangle contains the
        % curve rho(Vx) = [Vx; 1/Vx].
            cfg = lateralObserverConfig();
            polytope = lateralObserverSupport.buildSchedulingPolytope(cfg.scheduling.speedRange);
            speeds = linspace(cfg.scheduling.speedRange(1), cfg.scheduling.speedRange(2), 401);
            for speed = speeds
                alpha = lateralObserverSupport.schedulingCoordinates(polytope, speed);
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
            model = lateralObserverSupport.lateralBicycleModel(cfg.vehicle);
            polytope = lateralObserverSupport.buildSchedulingPolytope(cfg.scheduling.speedRange);
            vertexA = zeros(2, 2, 3);
            vertexC = zeros(1, 2, 3);
            for vertexIdx = 1:3
                [vertexA(:, :, vertexIdx), vertexC(:, :, vertexIdx)] = ...
                    lateralObserverSupport.evaluateLateralModel(model, polytope.vertices(:, vertexIdx));
            end

            for speed = linspace(cfg.scheduling.speedRange(1), cfg.scheduling.speedRange(2), 51)
                alpha = lateralObserverSupport.schedulingCoordinates(polytope, speed);
                [A, C] = lateralObserverSupport.evaluateLateralModel(model, [speed; 1.0 ./ speed]);
                blendedA = (alpha(1) .* vertexA(:, :, 1)) + (alpha(2) .* vertexA(:, :, 2)) + (alpha(3) .* vertexA(:, :, 3));
                blendedC = (alpha(1) .* vertexC(:, :, 1)) + (alpha(2) .* vertexC(:, :, 2)) + (alpha(3) .* vertexC(:, :, 3));
                testCase.verifyEqual(blendedA, A, AbsTol=1.0e-10);
                testCase.verifyEqual(blendedC, C, AbsTol=1.0e-10);
            end
        end

        function scheduledGainRecoversFromBlendedMetricAndY(testCase)
            design = testCase.CurrentDesign;
            speed = 12.3;
            alpha = lateralObserverSupport.schedulingCoordinates(design.polytope, speed);
            expectedP = sum(design.vertexLyapunovMatrices.*reshape(alpha, 1, 1, 3), 3);
            expectedY = sum(design.vertexY.*reshape(alpha, 1, 1, 3), 3);
            [L, P] = lateralObserverSupport.scheduleLateralObserverGain(design, speed);
            testCase.verifyEqual(P, expectedP, AbsTol=1e-12);
            testCase.verifyEqual(P*L, expectedY, AbsTol=1e-12);
            atMinimum = lateralObserverSupport.scheduleLateralObserverGain(design, design.polytope.speedRange(1));
            atMaximum = lateralObserverSupport.scheduleLateralObserverGain(design, design.polytope.speedRange(2));
            testCase.verifyEqual(atMinimum, design.vertexGains(:, :, 1), AbsTol=1e-10);
            testCase.verifyEqual(atMaximum, design.vertexGains(:, :, 2), AbsTol=1e-10);
            testCase.verifyEqual(lateralObserverSupport.scheduleLateralObserverGain(design, 0), atMinimum, AbsTol=0);
            testCase.verifyEqual(lateralObserverSupport.scheduleLateralObserverGain(design, 35), atMaximum, AbsTol=0);
        end

        function scheduledMetricDerivativeMatchesTheChainRule(testCase)
            design = testCase.CurrentDesign;
            design.vertexLyapunovMatrices = cat(3, [2,.1;.1,3], [5,.2;.2,2], [3,-.1;-.1,4]);
            speed = 12.3;
            rate = 1.7;
            step = 1e-5;
            [~, ~, Pdot] = lateralObserverSupport.scheduleLateralObserverGain(design, speed, rate);
            [~, before] = lateralObserverSupport.scheduleLateralObserverGain(design, speed - step*rate);
            [~, after] = lateralObserverSupport.scheduleLateralObserverGain(design, speed + step*rate);
            testCase.verifyGreaterThan(norm(Pdot), .01);
            testCase.verifyEqual(Pdot, (after - before)/(2*step), AbsTol=1e-8);
        end

        function measuredInputResidualIncludesVirtualOutputAndFeedthrough(testCase)
            design = testCase.CurrentDesign;
            speed = 17;
            measuredSpeed = 16.5;
            delta = .03;
            measuredDelta = .028;
            state = [.4; -.12];
            estimate = [.3; -.1];
            error = state - estimate;
            noise = .1;
            model = design.model;
            A = lateralObserverSupport.evaluateLateralModel(model, [speed; 1/speed]);
            [Am, C, Dm] = lateralObserverSupport.evaluateLateralModel(model, [measuredSpeed; 1/measuredSpeed]);
            L = lateralObserverSupport.scheduleLateralObserverGain(design, measuredSpeed);
            plantRate = A*state + model.B*delta;
            ay = plantRate(1) + speed*state(2) + noise;
            observerRate = Am*estimate + model.B*measuredDelta + ...
                L*(measuredSpeed*ay - C*estimate - Dm*measuredDelta);
            residual = (A - Am)*state + model.B*(delta - measuredDelta) - ...
                L*((measuredSpeed/speed - 1)*C*state + measuredSpeed*model.B(1)*(delta - measuredDelta));
            expected = (Am - L*C)*error - measuredSpeed*L*noise + residual;
            testCase.verifyEqual(plantRate - observerRate, expected, AbsTol=1e-12);
        end

        function causalSpeedRateDiagnosticsDoNotUseSpecificForce(testCase)
            measurements = testCase.constantSpeedMeasurements(.03, 10);
            measurements.longitudinalSpeed = [10;10.02;10.08;10.02];
            measurements.longitudinalAcceleration(:) = 50;
            estimate = runLateralVelocityObserver(measurements, testCase.CurrentDesign, testCase.CurrentDesign.cfg);
            testCase.verifyEqual(estimate.scheduledSpeedRate, [0;2;6;-6], AbsTol=1e-10);
            testCase.verifyEqual(estimate.diagnostics.schedulingRateWithinBounds, [false;true;false;false]);
            testCase.verifyTrue(all(estimate.velocityValid));
        end

        function observerConvergesFromAWrongInitialState(testCase)
        % observerConvergesFromAWrongInitialState: On the synthetic scenario the
        % observer recovers the lateral velocity from a deliberately wrong
        % initial state and settles well below the peak of the signal.
            design = testCase.CurrentDesign;
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
            cfg = testCase.CurrentDesign.cfg;
            cfg.synthesis.lipschitzConstant = 0.5;
            design = designLateralObserverGains(cfg);
            nonlinearGain = 0.3;
            cfg.observer.nonlinearity = @(x) [nonlinearGain .* sin(x(2)); -0.2 .* sin(x(1))];
            testCase.assertLessThanOrEqual(nonlinearGain, design.lipschitzConstant);

            result = simulateLateralObserverScenario(design, cfg);
            testCase.verifyLessThan(result.metrics.settledLateralVelocityRmse, ...
                0.05 .* result.metrics.peakLateralVelocity);
            testCase.verifyTrue(all(isfinite(result.estimate.state(:))));
        end

        function scheduledGainMustNotBlendVertexGains(testCase)
            design = testCase.CurrentDesign;
            % An artificial schedule makes an incorrect gain blend detectable
            % independently of the optimizer's particular solution.
            design.vertexLyapunovMatrices = cat(3, eye(2), 2*eye(2), 3*eye(2));
            design.vertexY = cat(3, [1;2], [-2;1], [3;-1]);
            design.vertexGains = cat(3, [1;2], [-1;.5], [1;-1/3]);
            alpha = lateralObserverSupport.schedulingCoordinates(design.polytope, 12.3);
            [L, P] = lateralObserverSupport.scheduleLateralObserverGain(design, 12.3);
            blendL = sum(design.vertexGains.*reshape(alpha, 1, 1, 3), 3);
            expectedY = sum(design.vertexY.*reshape(alpha, 1, 1, 3), 3);
            testCase.verifyEqual(P*L, expectedY, AbsTol=1e-12);
            testCase.verifyGreaterThan(norm(L - blendL), .01);
        end

        function sideSlipRateIsConsistentWithTheSideSlipAngle(testCase)
        % sideSlipRateIsConsistentWithTheSideSlipAngle: The algebraic
        % sideslip derivative agrees approximately with a finite difference of
        % its reported angle. This is the signal the ego-state observer
        % consumes as part of the track-angle rate.
            design = testCase.CurrentDesign;
            cfg = design.cfg;
            cfg.simulation.yawRateNoiseStd = 0;
            cfg.simulation.lateralAccelerationNoiseStd = 0;
            result = simulateLateralObserverScenario(design, cfg);
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
            design = testCase.CurrentDesign;
            result = simulateLateralObserverScenario(design, design.cfg);

            expectedRate = result.measurements.longitudinalAcceleration + ...
                (result.estimate.observerPointLateralVelocity .* result.estimate.yawRate);
            testCase.verifyEqual(result.estimate.longitudinalSpeedRate, expectedRate, AbsTol=1.0e-12);
            testCase.verifyGreaterThan(max(abs(result.estimate.longitudinalSpeedRate - ...
                result.measurements.longitudinalAcceleration)), 0, ...
                "The scenario should exercise a nonzero vy*r correction.");
        end

        function outputPointTransportFollowsRigidBodyKinematics(testCase)
        % outputPointTransportFollowsRigidBodyKinematics: The exported lateral
        % velocity is the LPV state transported to the declared output
        % point, vyOutput = vyObserver - forwardOffset*r, and the side-slip
        % interface equals the direction of that output-point velocity. A zero
        % offset exports the observer point unchanged.
            design = testCase.CurrentDesign;
            cfg = design.cfg;
            cfg.observer.initialState = [0;.20];
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
                atOutput.observerPointLateralVelocity - 2.5 .* atOutput.yawRate, AbsTol=1.0e-12);
            settled = true(size(measurements.time));
            expectedAngle = atan2(atOutput.lateralVelocity(settled), measurements.longitudinalSpeed(settled));
            testCase.verifyEqual(atOutput.sideSlipAngle(settled), expectedAngle, AbsTol=1.0e-12);
            testCase.verifyGreaterThan(abs(atOutput.sideSlipAngle(1) - atObserver.sideSlipAngle(1)), 0.03);
            testCase.verifyEqual(atOutput.outputPoint.forwardOffsetM, 2.5);
        end

        function missingOutputPointConfigurationIsRejected(testCase)
        % missingOutputPointConfigurationIsRejected: A configuration without
        % the output-point group is an obsolete contract, not a zero offset.
            design = testCase.CurrentDesign;
            cfg = rmfield(design.cfg, "outputPoint");
            measurements = lateralObserverTest.constantSpeedMeasurements(1.0, 8.0);
            testCase.verifyError(@() runLateralVelocityObserver(measurements, design, cfg), ...
                "VehicleLocalization:MissingOutputPointConfiguration");
        end

        function unavailableLpvWithdrawsSideslip(testCase)
        % Outside the LPV domain angle and rate are zero placeholders and the held
        % velocity is explicitly unavailable to the global observer.
            design = testCase.CurrentDesign;
            cfg = design.cfg;
            numSamples = 200;
            measurements = struct();
            measurements.time = (0:(numSamples - 1)).' .* 0.01;
            measurements.steeringAngle = zeros(numSamples, 1);
            measurements.longitudinalSpeed = 0.25 .* cfg.scheduling.speedRange(1) .* ones(numSamples, 1);
            measurements.longitudinalAcceleration = zeros(numSamples, 1);
            measurements.lateralAcceleration = 0.2 .* ones(numSamples, 1);
            measurements.yawRate = 0.05 .* ones(numSamples, 1);

            estimate = runLateralVelocityObserver(measurements, design, cfg);

            testCase.verifyTrue(all(~estimate.diagnostics.sideSlipValid));
            testCase.verifyEqual(estimate.sideSlipAngle, zeros(numSamples, 1), AbsTol=0);
            testCase.verifyEqual(estimate.sideSlipAngleRate, zeros(numSamples, 1), AbsTol=0);
            testCase.verifyTrue(all(isfinite(estimate.state(:))));
        end

        function zeroSpeedNeverEvaluatesTheReciprocalModel(testCase)
        % zeroSpeedNeverEvaluatesTheReciprocalModel Corrupt the reciprocal
        % coefficients and prove that the zero-speed execution path does not
        % access them.
            design = testCase.CurrentDesign;
            design.model.A2(:) = NaN;
            cfg = lateralObserverConfig();
            cfg.observer.initialState = [1.0; 0.2];
            measurements = testCase.zeroMotionMeasurements(2.0);

            estimate = runLateralVelocityObserver(measurements, design, cfg);

            testCase.verifyFalse(any(estimate.diagnostics.dynamicModelEvaluated));
            testCase.verifyTrue(all(isfinite(estimate.state), "all"));
            testCase.verifyEqual(estimate.observerPointLateralVelocity(1), 1.0, AbsTol=0.0);
            testCase.verifyEqual(estimate.lateralVelocity, ...
                (1-cfg.outputPoint.forwardOffsetM*.2)*ones(size(estimate.time)), AbsTol=0.0);
            testCase.verifyEqual(estimate.sideSlipAngle, ...
                zeros(size(estimate.sideSlipAngle)), AbsTol=0.0);
            testCase.verifyEqual(estimate.sideSlipAngleRate, ...
                zeros(size(estimate.sideSlipAngleRate)), AbsTol=0.0);
        end

        function outsideDomainHoldsLpvWithoutIntegratingImu(testCase)
        % Out-of-domain acceleration and steering must not create a second
        % velocity estimator or evaluate reciprocal-speed coefficients.
            design = testCase.CurrentDesign;
            design.model.A2(:) = NaN;
            cfg = design.cfg;
            cfg.observer.initialState = [0.4; 0.0];
            for speed = [0.0, 0.5*cfg.scheduling.speedRange(1), cfg.scheduling.speedRange(2)+5.0]
                measurements = testCase.constantSpeedMeasurements(2.0, speed);
                measurements.steeringAngle(:) = 0.3;
                measurements.lateralAcceleration(:) = 0.1;
                estimate = runLateralVelocityObserver(measurements, design, cfg);
                testCase.verifyEqual(estimate.state, ...
                    repmat([0.4, 0.0], numel(estimate.time), 1), AbsTol=0);
                testCase.verifyFalse(any(estimate.velocityValid));
                testCase.verifyFalse(any(estimate.diagnostics.dynamicModelEvaluated));
                testCase.verifyTrue(all(estimate.mode == "unavailable"));
            end
        end

        function abruptValidityLossHoldsTheLpvState(testCase)
        % Invalid measurements withdraw the LPV flow and its public aiding.
            cfg = lateralObserverConfig();
            cfg.observer.initialState = [1.0; 0.0];
            measurements = testCase.constantSpeedMeasurements(3.0, 10.0);
            measurements.dynamicValid = measurements.time < 1.5;
            firstInvalid = find(~measurements.dynamicValid, 1, "first");
            estimate = runLateralVelocityObserver(measurements, testCase.CurrentDesign, cfg);
            testCase.verifyEqual(estimate.velocityValid, measurements.dynamicValid);
            testCase.verifyEqual(estimate.state(firstInvalid:end,:), ...
                repmat(estimate.state(firstInvalid,:), numel(estimate.time)-firstInvalid+1, 1), AbsTol=0);
            testCase.verifyEqual(estimate.gainNorm(firstInvalid:end), ...
                zeros(numel(estimate.time)-firstInvalid+1,1), AbsTol=0);
            testCase.verifyTrue(all(isfinite(estimate.sideSlipAngleRate)));
        end

        function stopGoAvailabilityFollowsTheLpvDomain(testCase)
        % Re-entry resumes the same LPV state; no out-of-domain correction is
        % presented as a valid estimate or as a recovery-time certificate.
            cfg = lateralObserverConfig();
            cfg.observer.initialState = [0.80; 0.10];
            measurements = testCase.stopGoMeasurements();
            estimate = runLateralVelocityObserver(measurements, testCase.CurrentDesign, cfg);
            valid = measurements.longitudinalSpeed >= cfg.scheduling.speedRange(1) & ...
                measurements.longitudinalSpeed <= cfg.scheduling.speedRange(2);
            testCase.verifyEqual(estimate.velocityValid, valid);
            testCase.verifyEqual(nnz(estimate.modeChanged), 2);
            testCase.verifyTrue(all(isfinite(estimate.state), "all"));
            testCase.verifyTrue(all(isfinite(estimate.sideSlipAngleRate)));
        end

        function lpvInnovationReachesExportedVelocityWithoutAnotherEstimator(testCase)
        % A one-step correction is the scheduled LPV correction itself,
        % without the former master/injection lag.
            design = testCase.CurrentDesign;
            cfg = design.cfg;
            cfg.observer.integrationMethod = "euler";
            cfg.observer.initialState = [0.0; 0.0];
            measurements = testCase.constantSpeedMeasurements(0.01, 10.0);
            measurements.lateralAcceleration(:) = 0.1; % y = 1 at Vx = 10.
            estimate = runLateralVelocityObserver(measurements, design, cfg);
            gain = lateralObserverSupport.scheduleLateralObserverGain(design, 10.0);
            testCase.verifyEqual(estimate.lateralVelocity(2), 0.01*(gain(1,1)-cfg.outputPoint.forwardOffsetM*gain(2,1)), AbsTol=1e-12);
            testCase.verifyEqual(estimate.state(:,1), estimate.observerPointLateralVelocity);
            testCase.verifyEqual(estimate.stateNames, ["lateralVelocity","yawRate"]);
        end

        function synthesisReproducesTheCurrentDesign(testCase)
        % synthesisReproducesTheCurrentDesign: Re-running the LMI synthesis
        % reproduces the current vertex gains. Needs YALMIP and an SDP solver.
            testCase.assumeTrue(exist("sdpvar", "file") == 2, ...
                "The lateral observer synthesis test needs YALMIP and an SDP solver on the MATLAB path.");
            design = testCase.CurrentDesign;
            resolved = designLateralObserverGains(design.cfg);

            testCase.verifyEqual(resolved.tau, design.tau, RelTol=1e-4);
            testCase.verifyEqual(resolved.certificateMethod, "virtual-ay-scheduled-metric-iss-v1");
            testCase.verifyEqual(resolved.issGain, design.issGain, RelTol=1.0e-4);
            testCase.verifyEqual(resolved.vertexGains, design.vertexGains, AbsTol=1.0e-4);
            testCase.verifyTrue(resolved.solverSolved);
        end
    end

    methods (Static, Access = private)
        function worst = maximumCertificateEigenvalue(design)
        % Recompute the physical inequality, including Pdot and noise scaling.
            worst = -Inf;
            for speed = linspace(design.polytope.speedRange(1), design.polytope.speedRange(2), 101)
                A = lateralObserverSupport.evaluateLateralModel(design.model, [speed; 1/speed]);
                for rate = [design.certificateDomain.speedRateRange, 0]
                    [L, P, Pdot] = lateralObserverSupport.scheduleLateralObserverGain(design, speed, rate);
                    Ae = A - L*design.model.C;
                    noise = -design.measurementErrorScale*speed*P*L;
                    Phi = Pdot + P*Ae + Ae.'*P + 2*design.decayRate*P + ...
                        design.tau*design.lipschitzConstant^2*eye(2);
                    if design.lipschitzConstant == 0
                        block = [Phi, noise; noise.', -design.issGain^2];
                    else
                        nonlinear = P*(eye(2) - speed*L*[1, 0]);
                        block = [Phi, noise, nonlinear; noise.', -design.issGain^2, zeros(1, 2); ...
                            nonlinear.', zeros(2, 1), -design.tau*eye(2)];
                    end
                    worst = max(worst, max(eig((block + block.')/2)));
                end
            end
        end

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
