classdef fiveInvariantOutputsTest < matlab.unittest.TestCase
% Five local invariant coordinates and their coupled dot/cross information.
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
        function exactlyTheFiveRequestedOutputsAreReturned(testCase)
            c=mncavFullObserverConfig();x=[1;3;1;2;4;-2;.2;.3];beta=.1;
            [h,H]=invariantObserverSupport.outputs(x,beta,c);
            expected=[25;-5;-10;-3*sin(.3)+4*cos(.3);.3];
            testCase.verifyEqual(h,expected,AbsTol=1e-13);
            testCase.verifySize(H,[5,8]);
            testCase.verifyEqual(H(:,[1,4]),zeros(5,2),AbsTol=0);
        end
        function changingRateChangesOnlyTheGyroOutput(testCase)
            c=mncavFullObserverConfig();x=[0;12;.6;0;0;1.2;-.1;.03];
            y=x;y(8)=y(8)+.2;
            a=invariantObserverSupport.outputs(x,.1,c);b=invariantObserverSupport.outputs(y,.1,c);
            testCase.verifyEqual(b(1:4),a(1:4),AbsTol=0);
            testCase.verifyEqual(b(5)-a(5),.2,AbsTol=1e-14);
        end
        function localQuotientCoordinatesAreRecovered(testCase,localAngle)
            c=mncavFullObserverConfig();beta=.1;speed=12;parallel=.6;normal=1.2;rate=.03;
            x=[0;speed;parallel;0;0;normal;-beta-localAngle;rate];
            h=invariantObserverSupport.outputs(x,beta,c);
            s=sqrt(h(1));
            recovered=[s;h(2:3)/s;asin(h(4)/s);h(5)];
            testCase.verifyEqual(recovered,[speed;parallel;normal;localAngle;rate],AbsTol=2e-14);
        end
        function jacobianHasFiveIndependentRowsAndTheRotationNullDirection(testCase)
            c=mncavFullObserverConfig();x=[1;12;.6;2;1;1.2;-.1;.03];
            [~,H]=invariantObserverSupport.outputs(x,.1,c);
            orbit=[0;-x(5);-x(6);0;x(2);x(3);1;0];
            testCase.verifyGreaterThan(min(svd(H)),.1);
            testCase.verifyEqual(H*orbit,zeros(5,1),AbsTol=2e-14);
        end
        function orthogonalHeadingBranchLosesLocalRank(testCase)
            c=mncavFullObserverConfig();x=[0;12;.6;0;0;1.2;-pi/2;.03];
            [~,H]=invariantObserverSupport.outputs(x,0,c);
            testCase.verifyLessThan(min(svd(H)),1e-12);
        end
        function zeroSpeedRemainsFiniteButDoesNotGiveFiveCoordinates(testCase)
            c=mncavFullObserverConfig();x=[0;0;.6;0;0;1.2;0;.03];
            [h,H]=invariantObserverSupport.outputs(x,0,c);
            testCase.verifyTrue(all(isfinite([h;H(:)])));
            testCase.verifyLessThanOrEqual(rank(H),3);
        end
        function missingVelocityWithdrawsEveryAccelerationOutput(testCase)
            c=mncavFullObserverConfig();
            p=invariantObserverSupport.measurements(NaN,NaN,20,-30,.1,NaN,false,true,true,c);
            testCase.verifyEqual(p.valid,logical([0;0;0;0;1]));
            testCase.verifyEqual(p.value,[0;0;0;0;.1],AbsTol=0);
            testCase.verifyFalse(isfield(p,'betaRate'));
        end
        function missingAccelerationKeepsSpeedDirectionAndGyro(testCase)
            c=mncavFullObserverConfig();
            p=invariantObserverSupport.measurements(12,1,NaN,NaN,.1,.2,true,false,true,c);
            testCase.verifyEqual(p.valid,logical([1;0;0;1;1]));
            testCase.verifyTrue(all(isfinite(p.value)));
        end
        function accelerationHasNoHiddenProjectionCorrection(testCase)
            c=mncavFullObserverConfig();d=correctionDesign();x=[0;3;.6;0;4;1.2;.2;.03];
            p=invariantObserverSupport.measurements(NaN,NaN,20,-30,NaN,NaN,false,true,false,c);
            y=invariantObserverSupport.correct(x,p,.1,d,c);
            testCase.verifyEqual(y,x,AbsTol=0);
        end
        function fifthOutputAppliesTheGyroInnovationExactlyOnce(testCase)
            c=mncavFullObserverConfig();d=correctionDesign();x=zeros(8,1);dt=.1;
            p=invariantObserverSupport.measurements(NaN,NaN,NaN,NaN,.3,NaN,false,false,true,c);
            y=invariantObserverSupport.correct(x,p,dt,d,c);
            testCase.verifyEqual(y(8),dt*d.gyroGainNormalized*.3/(1+dt*d.gyroGainNormalized),AbsTol=1e-14);
            testCase.verifyEqual(y(1:7),zeros(7,1),AbsTol=0);
        end
        function everySensorFamilyCreditsSquaredSpeed(testCase)
            c=mncavFullObserverConfig();c.invariantObserver.redundancy.speedPartitions=1;
            c.invariantObserver.redundancy.exactGnssLeverEnclosure=false;
            w=sensorRedundancySupport.witness(c,eye(8),1,1,ones(3,1),1);
            [a,b,d]=sensorRedundancySupport.blocks(w);
            w.speedSquaredGain=0;[a0,b0,d0]=sensorRedundancySupport.blocks(w);
            E=eye(8);expected=-2*(w.domain.minimumCourseSpeed/w.inputs.maximumSpeed)^2*(E(:,2)*E(:,2).');
            for pair={{a,a0},{b,b0},{d,d0}}
                p=pair{1};testCase.verifyEqual(p{1}{1}-p{2}{1},expected,AbsTol=2e-12);
            end
        end
        function numericalNormalizationPreservesThePhysicalOutputs(testCase)
            c=mncavFullObserverConfig();x=[0;3;1;0;4;-2;.2;.3];
            h=invariantObserverSupport.outputs(x,.1,c);D=invariantObserverSupport.outputNormalization(c,1);
            V=c.iss.maximumSpeed;
            expected=[25/(2*V);-5/V;-10/V;-3*sin(.3)+4*cos(.3);c.invariantObserver.headingLengthM*.3];
            testCase.verifyEqual(D*h,expected,AbsTol=1e-13);
        end
    end
end

function d=correctionDesign()
    d=struct('theta',1,'P',eye(8),'toPhysical',eye(8), ...
        'auxiliaryWeights',ones(3,1),'gyroGainNormalized',2);
end
