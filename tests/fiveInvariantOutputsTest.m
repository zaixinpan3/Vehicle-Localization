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
            [h,H]=invariantObserverSupport.outputs(x,beta,c);V=c.iss.maximumSpeed;
            expected=[25/(2*V);-5/V;-10/V;-3*sin(.3)+4*cos(.3);c.invariantObserver.headingLengthM*.3];
            testCase.verifyEqual(h,expected,AbsTol=1e-13);
            testCase.verifySize(H,[5,8]);
            testCase.verifyEqual(H(:,[1,4]),zeros(5,2),AbsTol=0);
        end
        function changingRateChangesOnlyTheGyroOutput(testCase)
            c=mncavFullObserverConfig();x=[0;12;.6;0;0;1.2;-.1;.03];
            y=x;y(8)=y(8)+.2;
            a=invariantObserverSupport.outputs(x,.1,c);b=invariantObserverSupport.outputs(y,.1,c);
            testCase.verifyEqual(b(1:4),a(1:4),AbsTol=0);
            testCase.verifyEqual(b(5)-a(5),.2*c.invariantObserver.headingLengthM,AbsTol=1e-14);
        end
        function localQuotientCoordinatesAreRecovered(testCase,localAngle)
            c=mncavFullObserverConfig();beta=.1;speed=12;parallel=.6;normal=1.2;rate=.03;
            x=[0;speed;parallel;0;0;normal;-beta-localAngle;rate];
            h=invariantObserverSupport.outputs(x,beta,c);
            s=sqrt(2*c.iss.maximumSpeed*h(1));
            recovered=[s;c.iss.maximumSpeed*h(2:3)/s;asin(h(4)/s);h(5)/c.invariantObserver.headingLengthM];
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
            testCase.verifyEqual(p.value,[0;0;0;0;.1*c.invariantObserver.headingLengthM],AbsTol=0);
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
        function pairedBilinearLowerBoundHoldsOnBothSpeedDomains(testCase)
            minimum=pairBoundMinimum();
            testCase.verifyGreaterThanOrEqual(minimum,-1e-12);
        end
        function gnssLmiUsesThePairedVelocityPenaltyAndSpeedDependentAcceleration(testCase)
            [actual,expected]=bilinearControlDifference();
            testCase.verifyEqual(actual,expected,AbsTol=2e-12);
        end
    end
end

function d=correctionDesign()
    d=struct('theta',1,'P',eye(8),'toPhysical',eye(8), ...
        'auxiliaryWeights',ones(3,1),'gyroGainNormalized',2);
end

function minimum=pairBoundMinimum()
    stream=RandStream('mt19937ar','Seed',2605);minimum=Inf;E=eye(8);
    Da=E(:,[3,6])*E(:,[3,6]).';Dv=E(:,[2,5])*E(:,[2,5]).';
    configs={mncavFullObserverConfig(),mncavSmallOffsetObserverConfig()};
    for i=1:numel(configs)
        c=configs{i};V=c.iss.maximumSpeed;A=c.iss.maximumAcceleration;t=c.invariantObserver.theta;
        ell=c.invariantObserver.headingLengthM;eta=c.invariantObserver.redundancy.bilinearSplit;
        B=diag(t.^[1,2,3,1,2,3,1,2]./[ones(1,6),ell,ell]);
        for j=1:100
            speed=c.invariantObserver.redundancy.minimumCourseSpeed+rand(stream)*(V-c.invariantObserver.redundancy.minimumCourseSpeed);
            angle=2*pi*rand(stream);a=A*rand(stream)*[cos(angle);sin(angle)];
            x=[0;speed;a(1);0;0;a(2);0;.03];[~,H]=invariantObserverSupport.outputs(x,0,c);
            pair=t^-3*H(2:3,:)*B;
            lower=(1-eta)*(speed/V)^2*Da-(1/eta-1)*(A/(t*V))^2*Dv;
            minimum=min(minimum,min(eig(pair.'*pair-lower)));
        end
    end
end

function [actual,expected]=bilinearControlDifference()
    c=mncavFullObserverConfig();c.invariantObserver.redundancy.speedPartitions=1;
    c.invariantObserver.redundancy.exactGnssLeverEnclosure=false;
    w=sensorRedundancySupport.witness(c,eye(8),1,1,ones(3,1),1);
    [~,withPair]=sensorRedundancySupport.blocks(w);
    w.bilinearGain=0;[~,withoutPair]=sensorRedundancySupport.blocks(w);
    u=.37;weights=[(1-u)^2,2*u*(1-u),u^2];actual=zeros(8);
    for j=1:3,actual=actual+weights(j)*(withPair{j}-withoutPair{j});end
    speed=(1-u)*w.domain.minimumCourseSpeed+u*w.inputs.maximumSpeed;
    E=eye(8);eta=w.domain.bilinearSplit;
    expected=-2*(1-eta)*(speed/w.inputs.maximumSpeed)^2*(E(:,[3,6])*E(:,[3,6]).') ...
        +2*(1/eta-1)*(w.inputs.maximumAcceleration/(w.inputs.theta*w.inputs.maximumSpeed))^2*(E(:,[2,5])*E(:,[2,5]).');
end
