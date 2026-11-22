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
        function isolatedSquaredSpeedUsesItsFixedColumn(testCase)
            [cfg,design,packet,state]=fixture();
            actual=localizationSupport.correct(state,packet,.1,design,cfg);
            expected=state+.1*design.M(:,1)*(64-25);
            testCase.verifyEqual(actual,expected,AbsTol=1e-12);
        end
        function speedMagnitudeFeedbackDoesNotUseAHeadingProjection(testCase)
            [cfg,design,packet,state]=fixture();
            a=localizationSupport.correct(state,packet,.1,design,cfg);
            changed=state;changed(7)=state(7)+.8;
            b=localizationSupport.correct(changed,packet,.1,design,cfg);
            testCase.verifyEqual(b([1:6,8]),a([1:6,8]),AbsTol=1e-12);
            testCase.verifyEqual(b(7),changed(7),AbsTol=0);
        end
        function removingMatchedWeightsLeavesNoHiddenSpeedCorrection(testCase)
            [cfg,design,packet,state]=fixture();design.M(:,1)=0;
            actual=localizationSupport.correct(state,packet,.1,design,cfg);
            testCase.verifyEqual(actual,state,AbsTol=0);
        end
        function zeroVelocityUsesTheSameFixedColumn(testCase)
            [cfg,design,packet,~]=fixture();state=zeros(8,1);
            actual=localizationSupport.correct(state,packet,.1,design,cfg);
            testCase.verifyTrue(all(isfinite(actual)));
            testCase.verifyEqual(actual,state+.1*64*design.M(:,1),AbsTol=1e-12);
        end
    end
end

function [cfg,design,packet,state]=fixture()
    cfg=mncavFullObserverConfig();
    M=zeros(8,4);M(2,1)=.15;design=struct('M',M,'toPhysical',eye(8));
    packet=localizationSupport.measurements(8,0,0,0,true,false,cfg);
    state=[0;3;0;0;4;0;.4;0];
end
