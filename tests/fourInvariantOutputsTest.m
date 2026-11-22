classdef fourInvariantOutputsTest < matlab.unittest.TestCase
% Four local invariant coordinates and their coupled dot/cross information.
    properties (TestParameter)
        localAngle={0,.3,1.4}
    end
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));setupVehicleLocalization;
        end
    end
    methods (Test)
        function exactlyTheFourRequestedOutputsAreReturned(testCase)
            c=mncavFullObserverConfig();x=[1;3;1;2;4;-2;.2;.3];
            [h,H]=localizationSupport.outputs(x,c);
            expected=[25;-5;-10;-3*sin(.2)+4*cos(.2)];
            testCase.verifyEqual(h,expected,AbsTol=1e-13);
            testCase.verifySize(H,[4,8]);
            testCase.verifyEqual(H(:,[1,4]),zeros(4,2),AbsTol=0);
        end
        function changingRateDoesNotChangeAdditionalOutputs(testCase)
            c=mncavFullObserverConfig();x=[0;12;.6;0;0;1.2;-.1;.03];
            y=x;y(8)=y(8)+.2;
            a=localizationSupport.outputs(x,c);b=localizationSupport.outputs(y,c);
            testCase.verifyEqual(b,a,AbsTol=0);
        end
        function localQuotientCoordinatesAreRecovered(testCase,localAngle)
            c=mncavFullObserverConfig();speed=12;parallel=.6;normal=1.2;rate=.03;
            x=[0;speed;parallel;0;0;normal;-localAngle;rate];
            h=localizationSupport.outputs(x,c);
            s=sqrt(h(1));
            recovered=[s;h(2:3)/s;asin(h(4)/s)];
            testCase.verifyEqual(recovered,[speed;parallel;normal;localAngle],AbsTol=2e-14);
        end
        function jacobianHasFourIndependentRowsAndTheRotationNullDirection(testCase)
            c=mncavFullObserverConfig();x=[1;12;.6;2;1;1.2;-.1;.03];
            [~,H]=localizationSupport.outputs(x,c);
            orbit=[0;-x(5);-x(6);0;x(2);x(3);1;0];
            testCase.verifyGreaterThan(min(svd(H)),.1);
            testCase.verifyEqual(H*orbit,zeros(4,1),AbsTol=2e-14);
        end
        function orthogonalHeadingBranchLosesLocalRank(testCase)
            c=mncavFullObserverConfig();x=[0;12;.6;0;0;1.2;-pi/2;.03];
            [~,H]=localizationSupport.outputs(x,c);
            testCase.verifyLessThan(min(svd(H)),1e-12);
        end
        function zeroSpeedRemainsFiniteButDoesNotGiveFourCoordinates(testCase)
            c=mncavFullObserverConfig();x=[0;0;.6;0;0;1.2;0;.03];
            [h,H]=localizationSupport.outputs(x,c);
            testCase.verifyTrue(all(isfinite([h;H(:)])));
            testCase.verifyLessThanOrEqual(rank(H),3);
        end
        function missingVelocityWithdrawsEveryAccelerationOutput(testCase)
            c=mncavFullObserverConfig();
            p=localizationSupport.measurements(NaN,NaN,20,-30,false,true,c);
            testCase.verifyEqual(p.valid,false(4,1));
            testCase.verifyEqual(p.value,zeros(4,1),AbsTol=0);
            testCase.verifyFalse(isfield(p,'yawRate'));
        end
        function missingAccelerationKeepsSpeedAndLateralVelocity(testCase)
            c=mncavFullObserverConfig();
            p=localizationSupport.measurements(12,1,NaN,NaN,true,false,c);
            testCase.verifyEqual(p.valid,logical([1;0;0;1]));
            testCase.verifyTrue(all(isfinite(p.value)));
        end
        function accelerationHasNoHiddenProjectionCorrection(testCase)
            c=mncavFullObserverConfig();d=correctionDesign();x=[0;3;.6;0;4;1.2;.2;.03];
            p=localizationSupport.measurements(NaN,NaN,20,-30,false,true,c);
            y=localizationSupport.correct(x,p,.1,d,c);
            testCase.verifyEqual(y,x,AbsTol=0);
        end
        function motionInnovationCorrectsRateThroughTheHeadingMetric(testCase)
            c=mncavFullObserverConfig();d=correctionDesign();
            x=[0;12;0;0;0;0;.1;.03];
            p=localizationSupport.measurements(12,0,0,0,true,true,c);
            p.valid=logical([0;0;0;1]);
            uncoupled=localizationSupport.correct(x,p,.1,d,c);
            d.P(7:8,7:8)=[2,-1;-1,2];
            coupled=localizationSupport.correct(x,p,.1,d,c);
            testCase.verifyEqual(uncoupled(8),x(8),AbsTol=0);
            testCase.verifyLessThan(coupled(7),x(7));
            testCase.verifyLessThan(coupled(8),x(8));
            p.valid(:)=false;
            withdrawn=localizationSupport.correct(x,p,.1,d,c);
            testCase.verifyEqual(withdrawn,x,AbsTol=0);
        end
        function numericalNormalizationPreservesThePhysicalOutputs(testCase)
            c=mncavFullObserverConfig();x=[0;3;1;0;4;-2;.2;.3];
            h=localizationSupport.outputs(x,c);D=localizationSupport.outputNormalization(c,1);
            V=c.iss.maximumSpeed;
            expected=[25/(2*V);-5/V;-10/V;-3*sin(.2)+4*cos(.2)];
            testCase.verifyEqual(D*h,expected,AbsTol=1e-13);
        end
    end
end

function d=correctionDesign()
    d=struct('theta',1,'P',eye(8),'toPhysical',eye(8), ...
        'auxiliaryWeights',ones(3,1));
end
