classdef lateralObserverIntegrationTest < matlab.unittest.TestCase
% lateralObserverIntegrationTest Numerical integration of the same LPV flow.
    properties (Access = private)
        Design
    end
    methods (TestClassSetup)
        function currentDefaultDesign(testCase)
            root = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));
            setupVehicleLocalization;
            cfg = lateralObserverConfig();
            testCase.Design = designLateralObserverGains(cfg);
        end
    end
    methods (Test)
        function constantForcedLpvMatchesAnalyticFlowAt100ms(testCase)
            cfg = testCase.Design.cfg;
            cfg.observer.initialState = [.05;.005];
            m = testCase.measurements((0:.1:1).',1);
            m.steeringAngle(:) = .01;
            m.lateralAcceleration(:) = .1;
            m.yawRate(:) = .01;
            [M,b] = testCase.linearSystem(testCase.Design,1,.01,.1,.01);
            expected = testCase.exactAffine(M,b,cfg.observer.initialState,m.time);
            actual = runLateralVelocityObserver(m,testCase.Design,cfg);
            testCase.verifyEqual(actual.state,expected,AbsTol=1e-12);
            testCase.verifyTrue(all(actual.velocityValid));
        end

        function fastModeDecaysAt100ms(testCase)
            cfg = testCase.Design.cfg;
            [M,~] = testCase.linearSystem(testCase.Design,1,0,0);
            [V,D] = eig(M);
            [~,idx] = min(real(diag(D)));
            cfg.observer.initialState = .01*real(V(:,idx))/norm(real(V(:,idx)));
            actual = runLateralVelocityObserver(testCase.measurements((0:.1:1).',1),testCase.Design,cfg);
            testCase.verifyLessThan(norm(actual.state(2,:)),1e-7);
            testCase.verifyLessThan(norm(actual.state(end,:)),1e-12);
            testCase.verifyLessThanOrEqual(max(vecnorm(actual.state,2,2)),.01+1e-14);
        end

        function constantDirectionHasNoAngleStartupLag(testCase)
            cfg = testCase.Design.cfg;
            m = testCase.measurements((0:.1:2).',1);
            transform = repmat([1,0,0,1,0,.3],numel(m.time),1);
            actual = runLateralVelocityObserver(m,testCase.Design,cfg,transform);
            testCase.verifyEqual(actual.sideSlipAngle, ...
                atan2(.3,1)*ones(size(m.time)),AbsTol=1e-12);
            testCase.verifyEqual(actual.sideSlipAngleRate,zeros(size(m.time)),AbsTol=0);
            testCase.verifyFalse(isfield(cfg,'sideSlip'));
            testCase.verifySize(actual.state,[numel(m.time),2]);
        end

        function zeroSpeedHoldsStateAndWithdrawsSideslip(testCase)
            design = testCase.Design;
            design.model.A2(:) = NaN;
            cfg = design.cfg;
            cfg.observer.initialState = [.4;.02];
            m = testCase.measurements((0:.1:1).',0);
            actual = runLateralVelocityObserver(m,design,cfg);
            testCase.verifyEqual(actual.state,repmat([.4,.02],numel(m.time),1),AbsTol=0);
            testCase.verifyEqual(actual.sideSlipAngle,zeros(size(m.time)),AbsTol=0);
            testCase.verifyEqual(actual.sideSlipAngleRate,zeros(size(m.time)),AbsTol=0);
            testCase.verifyFalse(any(actual.velocityValid));
        end

        function lowerSpeedCrossingPreservesTheUnavailablePart(testCase)
            cfg = testCase.Design.cfg;
            cfg.observer.initialState = [.4;.02];
            coarse = testCase.measurements([0;.1],[0;2]);
            split = testCase.measurements([0;.05;.1],[0;1;2]);
            a = runLateralVelocityObserver(coarse,testCase.Design,cfg);
            b = runLateralVelocityObserver(split,testCase.Design,cfg);
            testCase.verifyEqual(b.state(2,:),cfg.observer.initialState.',AbsTol=0);
            testCase.verifyEqual(a.state(end,:),b.state(end,:),AbsTol=1e-12);
            testCase.verifyEqual(a.sideSlipAngle(end),b.sideSlipAngle(end),AbsTol=1e-12);
        end

        function upperSpeedCrossingHoldsAfterLeavingTheDomain(testCase)
            cfg = testCase.Design.cfg;
            cfg.observer.initialState = [.4;.02];
            coarse = testCase.measurements([0;.1],[29;31]);
            split = testCase.measurements([0;.05;.1],[29;30;31]);
            a = runLateralVelocityObserver(coarse,testCase.Design,cfg);
            b = runLateralVelocityObserver(split,testCase.Design,cfg);
            testCase.verifyEqual(b.state(3,:),b.state(2,:),AbsTol=0);
            testCase.verifyEqual(a.state(end,:),b.state(end,:),AbsTol=1e-12);
            testCase.verifyFalse(a.velocityValid(end));
        end

        function unavailableEndpointsDoNotInjectAtAnIsolatedValidSample(testCase)
            cfg = testCase.Design.cfg;
            cfg.observer.initialState = [.4;.02];
            m = testCase.measurements([0;.1;.2;.3],2);
            m.dynamicValid = [true;false;true;true];
            actual = runLateralVelocityObserver(m,testCase.Design,cfg);
            testCase.verifyEqual(actual.state(1:3,:),repmat([.4,.02],3,1),AbsTol=0);
            testCase.verifyGreaterThan(norm(actual.state(4,:)-actual.state(3,:)),.01);
            testCase.verifyEqual(actual.velocityValid,m.dynamicValid);
        end

        function singularLinearPartIntegratesAConstantForcing(testCase)
            design = testCase.Design;
            design.model.A0(:) = 0;
            design.model.A1(:) = 0;
            design.model.A2(:) = 0;
            design.vertexGains(:) = 0;
            design.vertexY(:) = 0;
            cfg = design.cfg;
            cfg.observer.initialState = [.4;.02];
            m = testCase.measurements([0;.1;.2],2);
            m.steeringAngle(:) = .01;
            actual = runLateralVelocityObserver(m,design,cfg);
            expected = cfg.observer.initialState.' + m.time*(design.model.B*.01).';
            testCase.verifyEqual(actual.state,expected,AbsTol=1e-12);
        end

        function copiedNonlinearityRetainsSecondOrderConvergence(testCase)
            cfg = testCase.Design.cfg;
            cfg.observer.initialState = [.05;.005];
            N = [.2,.1;-.1,.1];
            cfg.observer.nonlinearity = @(x) N*x;
            [M,b] = testCase.linearSystem(testCase.Design,10,.01,.1,.01);
            L = lateralObserverSupport.scheduleLateralObserverGain(testCase.Design, 10);
            nonlinearMap = eye(2) - 10*L(:,1)*[1,0];
            coarse = testCase.measurements((0:.1:1).',10);
            coarse.steeringAngle(:) = .01;
            coarse.lateralAcceleration(:) = .1;
            coarse.yawRate(:) = .01;
            fine = testCase.measurements((0:.05:1).',10);
            fine.steeringAngle(:) = .01;
            fine.lateralAcceleration(:) = .1;
            fine.yawRate(:) = .01;
            a = runLateralVelocityObserver(coarse,testCase.Design,cfg);
            z = runLateralVelocityObserver(fine,testCase.Design,cfg);
            expected = testCase.exactAffine(M+nonlinearMap*N,b,cfg.observer.initialState,coarse.time);
            coarseError = norm(a.state-expected,'fro');
            fineError = norm(z.state(1:2:end,:)-expected,'fro');
            testCase.verifyLessThan(fineError,.35*coarseError);
            testCase.verifyLessThan(coarseError,1e-3);
        end

        function branchCutDoesNotCreateAnAngularRateImpulse(testCase)
            cfg = testCase.Design.cfg;
            m = testCase.measurements([0;.1;.2],2);
            transform = [-ones(3,1),zeros(3,2),ones(3,1),zeros(3,1),[.01;-.01;-.03]];
            actual = runLateralVelocityObserver(m,testCase.Design,cfg,transform);
            testCase.verifyGreaterThan(actual.sideSlipAngle(1),3);
            testCase.verifyLessThan(actual.sideSlipAngle(2),-3);
            testCase.verifyEqual(actual.sideSlipAngleRate(2:3), ...
                .4./(4+[.01;.03].^2),AbsTol=1e-12);
        end

        function changingAffineMapIncludesMatrixAndTranslationRates(testCase)
            m = testCase.measurements([0;.07;.2;.31],10);
            t = m.time;
            transform = [ones(4,1),zeros(4,1),.2*t,ones(4,1),t,3*t];
            actual = runLateralVelocityObserver(m,testCase.Design,testCase.Design.cfg,transform);
            testCase.verifyEqual(actual.longitudinalSpeed,10+t,AbsTol=1e-12);
            testCase.verifyEqual(actual.lateralVelocity,5*t,AbsTol=1e-12);
            testCase.verifyEqual(actual.longitudinalSpeedRate,[0;1;1;1],AbsTol=1e-12);
            testCase.verifyEqual(actual.lateralVelocityRate,[0;5;5;5],AbsTol=1e-12);
            testCase.verifyEqual(actual.sideSlipAngleRate(2:end), ...
                50./((10+t(2:end)).^2+25*t(2:end).^2),AbsTol=1e-12);
        end

        function pointTransportUsesTheObserverYawDerivative(testCase)
            design = testCase.heldDesign(testCase.Design);
            design.model.B = [0;.5];
            cfg = design.cfg;
            cfg.outputPoint.forwardOffsetM = 2;
            m = testCase.measurements([0;.07;.2;.31],10);
            m.steeringAngle(:) = 1;
            % Gyro samples deliberately disagree: transport must use the
            % estimated rigid-body state, including its analytic derivative.
            m.yawRate(:) = 7;
            actual = runLateralVelocityObserver(m,design,cfg);
            testCase.verifyEqual(actual.lateralVelocity,-m.time,AbsTol=1e-12);
            testCase.verifyEqual(actual.lateralVelocityRate,-ones(4,1),AbsTol=1e-12);
            testCase.verifyEqual(actual.yawRateRate,.5*ones(4,1),AbsTol=1e-12);
            testCase.verifyEqual(actual.sideSlipAngleRate, ...
                -10./(100+m.time.^2),AbsTol=1e-12);
        end

        function outputCancellationWithdrawsAidingWithoutHoldingLpv(testCase)
            m = testCase.measurements([0;.1;.2],10);
            transform = repmat([1,0,0,1,-10,0],3,1);
            actual = runLateralVelocityObserver(m,testCase.Design,testCase.Design.cfg,transform);
            testCase.verifyFalse(any(actual.velocityValid));
            testCase.verifyTrue(all(actual.diagnostics.dynamicModelEvaluated));
            testCase.verifyEqual(actual.sideSlipAngle,zeros(3,1),AbsTol=0);
            testCase.verifyEqual(actual.sideSlipAngleRate,zeros(3,1),AbsTol=0);
        end

        function appendedMeasurementsCannotChangeExportedPast(testCase)
            m = testCase.measurements([0;.1;.2;.3],10);
            m.yawRate = [.01;.03;.02;.1];
            transform = [ones(4,1),zeros(4,2),ones(4,1),m.time,2*m.time];
            short = testCase.prefix(m,3);
            a = runLateralVelocityObserver(short,testCase.Design,testCase.Design.cfg,transform(1:3,:));
            b = runLateralVelocityObserver(m,testCase.Design,testCase.Design.cfg,transform);
            testCase.verifyEqual(a.sideSlipAngle,b.sideSlipAngle(1:3),AbsTol=0);
            testCase.verifyEqual(a.sideSlipAngleRate,b.sideSlipAngleRate(1:3),AbsTol=0);
        end

        function obsoleteFilterSettingsAreRejected(testCase)
            cfg = testCase.Design.cfg;
            cfg.sideSlip = struct('naturalFrequency',12);
            m = testCase.measurements([0;.1],10);
            testCase.verifyError(@() runLateralVelocityObserver(m,testCase.Design,cfg), ...
                'VehicleLocalization:ObsoleteLateralConfiguration');
        end

    end
    methods (Static, Access = private)
        function design = heldDesign(design)
            design.model.A0(:) = 0;
            design.model.A1(:) = 0;
            design.model.A2(:) = 0;
            design.vertexGains(:) = 0;
            design.vertexY(:) = 0;
        end
        function m = prefix(m,n)
            names = fieldnames(m);
            for k = 1:numel(names)
                m.(names{k}) = m.(names{k})(1:n,:);
            end
        end
        function m = measurements(t,v)
            n = numel(t);
            m = struct('time',t,'longitudinalSpeed',v+zeros(n,1), ...
                'steeringAngle',zeros(n,1),'longitudinalAcceleration',zeros(n,1), ...
                'lateralAcceleration',zeros(n,1),'yawRate',zeros(n,1));
        end
        function [M,b] = linearSystem(design,v,delta,ay,gyro)
            if nargin < 5, gyro = 0; end
            [A,C,D] = lateralObserverSupport.evaluateLateralModel(design.model,[v;1/v]);
            L = lateralObserverSupport.scheduleLateralObserverGain(design,v);
            M = A-L*C;
            b = design.model.B*delta+L*([v*ay;gyro]-D*delta);
        end
        function x = exactAffine(M,b,x0,t)
            equilibrium = -(M\b);
            x = zeros(numel(t),2);
            for k = 1:numel(t)
                x(k,:) = (equilibrium+expm(t(k)*M)*(x0-equilibrium)).';
            end
        end
    end
end
