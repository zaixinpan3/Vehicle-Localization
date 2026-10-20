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
            [M,b] = testCase.linearSystem(testCase.Design,1,.01,[.1;.01]);
            expected = testCase.exactAffine(M,b,cfg.observer.initialState,m.time);
            actual = runLateralVelocityObserver(m,testCase.Design,cfg);
            testCase.verifyEqual(actual.state,expected,AbsTol=1e-12);
            testCase.verifyTrue(all(actual.velocityValid));
        end

        function fastModeDecaysAt100ms(testCase)
            cfg = testCase.Design.cfg;
            [M,~] = testCase.linearSystem(testCase.Design,1,0,[0;0]);
            [V,D] = eig(M);
            [~,idx] = min(real(diag(D)));
            cfg.observer.initialState = .01*real(V(:,idx))/norm(real(V(:,idx)));
            actual = runLateralVelocityObserver(testCase.measurements((0:.1:1).',1),testCase.Design,cfg);
            testCase.verifyLessThan(norm(actual.state(2,:)),1e-7);
            testCase.verifyLessThan(norm(actual.state(end,:)),1e-12);
            testCase.verifyLessThanOrEqual(max(vecnorm(actual.state,2,2)),.01+1e-14);
        end

        function constantAngleCommandMatchesAnalyticFilterAt100ms(testCase)
            cfg = testCase.Design.cfg;
            cfg.sideSlip.initialState = [0;0];
            m = testCase.measurements((0:.1:2).',1);
            transform = repmat([1,0,0,1,0,.3],numel(m.time),1);
            actual = runLateralVelocityObserver(m,testCase.Design,cfg,transform);
            command = atan2(.3,1);
            w = cfg.sideSlip.naturalFrequency;
            testCase.verifyEqual(actual.sideSlipAngle, ...
                command*(1-(1+w*m.time).*exp(-w*m.time)),AbsTol=1e-12);
            testCase.verifyEqual(actual.sideSlipAngleRate, ...
                command*w^2*m.time.*exp(-w*m.time),AbsTol=1e-12);
        end

        function zeroSpeedHoldsStateAndRelaxesAngleWithoutReciprocalModel(testCase)
            design = testCase.Design;
            design.model.A2(:) = NaN;
            design.model.C2(:) = NaN;
            cfg = design.cfg;
            cfg.observer.initialState = [.4;.02];
            cfg.sideSlip.initialState = [.1;0];
            m = testCase.measurements((0:.1:1).',0);
            actual = runLateralVelocityObserver(m,design,cfg);
            w = cfg.sideSlip.naturalFrequency;
            testCase.verifyEqual(actual.state,repmat([.4,.02],numel(m.time),1),AbsTol=0);
            testCase.verifyEqual(actual.sideSlipAngle,.1*(1+w*m.time).*exp(-w*m.time),AbsTol=1e-12);
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
            [M,b] = testCase.linearSystem(testCase.Design,10,.01,[.1;.01]);
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
            expected = testCase.exactAffine(M+N,b,cfg.observer.initialState,coarse.time);
            coarseError = norm(a.state-expected,'fro');
            fineError = norm(z.state(1:2:end,:)-expected,'fro');
            testCase.verifyLessThan(fineError,.35*coarseError);
            testCase.verifyLessThan(coarseError,1e-3);
        end

        function wrappedAngleUsesTheNearbyTarget(testCase)
            cfg = testCase.Design.cfg;
            cfg.sideSlip.initialState = [pi-.01;0];
            m = testCase.measurements([0;.1],1);
            transform = repmat([-1,0,0,1,0,-.01],2,1);
            actual = runLateralVelocityObserver(m,testCase.Design,cfg,transform);
            testCase.verifyGreaterThan(actual.sideSlipAngle(2),actual.sideSlipAngle(1));
            testCase.verifyLessThan(actual.sideSlipAngle(2)-actual.sideSlipAngle(1),.03);
        end
    end
    methods (Static, Access = private)
        function m = measurements(t,v)
            n = numel(t);
            m = struct('time',t,'longitudinalSpeed',v+zeros(n,1), ...
                'steeringAngle',zeros(n,1),'longitudinalAcceleration',zeros(n,1), ...
                'lateralAcceleration',zeros(n,1),'yawRate',zeros(n,1));
        end
        function [M,b] = linearSystem(design,v,delta,y)
            [A,C] = lateralObserverSupport.evaluateLateralModel(design.model,[v;1/v]);
            L = lateralObserverSupport.scheduleLateralObserverGain(design,v);
            M = A-L*C;
            b = (design.model.B-L*design.model.D)*delta+L*y;
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
