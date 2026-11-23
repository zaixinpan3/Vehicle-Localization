classdef squaredSpeedChannelTest < matlab.unittest.TestCase
% Direct squared-speed feedback and its positive-domain LMI contribution.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function isolatedSquaredSpeedCorrectionIsRadial(testCase)
            [cfg,design,packet,state]=fixture();dt=.1;
            actual=localizationSupport.correct(state,packet,dt,design,cfg);
            factor=1+2*dt*(64-25)/(1+4*dt*25);
            expected=state;expected([2,5])=factor*state([2,5]);
            testCase.verifyEqual(actual,expected,AbsTol=1e-12);
            testCase.verifyGreaterThan(norm(actual([2,5])),5);
            % A frozen squared-speed Jacobian may overshoot the speed target.
            testCase.verifyLessThan(abs(norm(actual([2,5]))^2-64),abs(norm(state([2,5]))^2-64));
        end
        function speedMagnitudeFeedbackDoesNotUseAHeadingProjection(testCase)
            [cfg,design,packet,state]=fixture();
            a=localizationSupport.correct(state,packet,.1,design,cfg);
            changed=state;changed(7)=state(7)+.8;
            b=localizationSupport.correct(changed,packet,.1,design,cfg);
            testCase.verifyEqual(b([1:6,8]),a([1:6,8]),AbsTol=1e-12);
            testCase.verifyEqual(b(7),changed(7),AbsTol=0);
        end
        function zeroVelocityIsFiniteWithoutInventingAnEscapeDirection(testCase)
            [cfg,design,packet,~]=fixture();state=zeros(8,1);
            actual=localizationSupport.correct(state,packet,.1,design,cfg);
            testCase.verifyTrue(all(isfinite(actual)));
            testCase.verifyEqual(actual,state,AbsTol=0);
        end
    end
end

function [cfg,design,packet,state]=fixture()
    cfg=mncavFullObserverConfig();
    design=struct('theta',1,'P',eye(8),'toPhysical',eye(8));
    packet=localizationSupport.measurements(8,0,0,0,true,false,cfg);
    packet.valid(4)=false;
    state=[0;3;0;0;4;0;.4;0];
end
