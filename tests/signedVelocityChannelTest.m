classdef signedVelocityChannelTest < matlab.unittest.TestCase
% Signed longitudinal feedback on the world-frame state.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function signedLongitudinalUpdateFromRest(testCase)
            cfg=mncavFullObserverConfig();
            design=struct('theta',1,'P',eye(8),'toPhysical',eye(8));
            dt=.1; angle=.4;
            state=zeros(8,1);state(7)=angle;
            for speed=[-8,8]
                packet=localizationSupport.measurements(speed,0,NaN,NaN,true,false,cfg);
                actual=localizationSupport.correct(state,packet,dt,design,cfg);
                expected=state;
                expected([2,5])=dt/(1+dt)*speed*[cos(angle);sin(angle)];
                testCase.verifyEqual(actual,expected,AbsTol=1e-12);
            end
        end
    end
end
