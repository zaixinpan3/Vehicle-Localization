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
            [cfg,design,packet,state]=fixture();dt=.1;s=cfg.iss.maximumSpeed;
            actual=invariantObserverSupport.correct(state,packet,dt,design,cfg);
            factor=1+dt*2*(64-25)/(2*s^2)/(1+dt*2*25/s^2);
            expected=state;expected([2,5])=factor*state([2,5]);
            testCase.verifyEqual(actual,expected,AbsTol=1e-12);
            testCase.verifyGreaterThan(norm(actual([2,5])),5);
            testCase.verifyLessThan(norm(actual([2,5])),8);
        end
        function speedMagnitudeFeedbackDoesNotUseAHeadingProjection(testCase)
            [cfg,design,packet,state]=fixture();
            a=invariantObserverSupport.correct(state,packet,.1,design,cfg);
            changed=state;changed(7)=state(7)+.8;packet.beta=packet.beta-.3;
            b=invariantObserverSupport.correct(changed,packet,.1,design,cfg);
            testCase.verifyEqual(b([1:6,8]),a([1:6,8]),AbsTol=1e-12);
            testCase.verifyEqual(b(7),changed(7),AbsTol=0);
        end
        function removingMatchedWeightsLeavesNoHiddenSpeedCorrection(testCase)
            [cfg,design,packet,state]=fixture();design.auxiliaryWeights(:)=0;
            actual=invariantObserverSupport.correct(state,packet,.1,design,cfg);
            testCase.verifyEqual(actual,state,AbsTol=0);
        end
        function zeroVelocityIsFiniteWithoutInventingAnEscapeDirection(testCase)
            [cfg,design,packet,~]=fixture();state=zeros(8,1);
            actual=invariantObserverSupport.correct(state,packet,.1,design,cfg);
            testCase.verifyTrue(all(isfinite(actual)));
            testCase.verifyEqual(actual,state,AbsTol=0);
        end
        function bernsteinControlsRetainTheSquaredSpeedDissipation(testCase)
            [actual,expected]=speedControlDifference();
            testCase.verifyEqual(actual,expected,AbsTol=1e-12);
        end
    end
end

function [cfg,design,packet,state]=fixture()
    cfg=mncavFullObserverConfig();
    design=struct('theta',1,'P',eye(8),'toPhysical',eye(8), ...
        'auxiliaryWeights',[2;0;0],'gyroGainNormalized',1);
    packet=invariantObserverSupport.measurements(8,0,0,0,0,.2,true,false,false,cfg);
    state=[0;3;0;0;4;0;.4;0];
end

function [actual,expected]=speedControlDifference()
    cfg=mncavFullObserverConfig();cfg.invariantObserver.redundancy.speedPartitions=1;
    cfg.invariantObserver.redundancy.exactGnssLeverEnclosure=false;
    w=sensorRedundancySupport.witness(cfg,eye(8),1,1,ones(3,1),1);
    [~,withSpeed]=sensorRedundancySupport.blocks(w);
    w.speedSquaredGain=0;[~,withoutSpeed]=sensorRedundancySupport.blocks(w);
    u=.37;weights=[(1-u)^2,2*u*(1-u),u^2];actual=zeros(8);
    % Tensor ordering is speed, course rate, tangential acceleration; with
    % one GNSS sensor control, fixed q/b use speed controls 1,10,19.
    ids=[1,10,19];
    for j=1:3,actual=actual+weights(j)*(withSpeed{ids(j)}-withoutSpeed{ids(j)});end
    speed=(1-u)*w.domain.minimumCourseSpeed+u*w.inputs.maximumSpeed;
    expected=zeros(8);expected(2,2)=-2*(speed/w.inputs.maximumSpeed)^2;
end
