classdef fullObserverMotionAvailabilityTest < matlab.unittest.TestCase
% Missing motion outputs withdraw feedback rather than supply zero observations.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));setupVehicleLocalization;
        end
    end
    methods (Test)
        function defaultMasksPreserveTheExistingRuntime(testCase)
            [d,l,c]=fixture();a=execute(d,l,c);
            d.highRate.bodyVelocityValid=true(size(l.time));
            d.highRate.bodyAccelerationValid=true(size(l.time));b=execute(d,l,c);
            testCase.verifyEqual(a,b);
        end
        function absentOutputsPreserveFreePrediction(testCase)
            [d,l,c]=fixture();n=numel(l.time);
            d.highRate.bodyVelocityValid=false(n,1);d.highRate.bodyAccelerationValid=false(n,1);
            d.highRate.longitudinalSpeed(:)=NaN;l.lateralVelocity(:)=NaN;
            d.highRate.longitudinalAcceleration(:)=NaN;d.highRate.lateralAcceleration(:)=NaN;
            a=execute(d,l,c);t=l.time;
            expectedVelocity=3+.2*t;expectedPosition=[0;cumsum(diff(t).*expectedVelocity(2:end))];
            testCase.verifyEqual(a.velocity(:,1),expectedVelocity,AbsTol=1e-12);
            testCase.verifyEqual(a.position(:,1),expectedPosition,AbsTol=1e-12);
            testCase.verifyEqual(a.diagnostics.packetCounts.bodyVelocityValid,0);
            testCase.verifyFalse(a.diagnostics.observedMotionEnvelopeSatisfied);
        end
        function withdrawnPayloadCannotInfluenceAnyState(testCase)
            [d,l,c]=fixture();n=numel(l.time);
            d.highRate.bodyVelocityValid=false(n,1);d.highRate.bodyAccelerationValid=false(n,1);
            a=execute(d,l,c);
            d.highRate.longitudinalSpeed(:)=1e8;l.lateralVelocity(:)=NaN;
            d.highRate.longitudinalAcceleration(:)=NaN;d.highRate.lateralAcceleration(:)=-1e8;
            b=execute(d,l,c);testCase.verifyEqual(a.z,b.z);
        end
        function availableNanStillFailsValidation(testCase)
            [d,l,c]=fixture();d.highRate.longitudinalSpeed(2)=NaN;
            testCase.verifyError(@()execute(d,l,c),'VehicleLocalization:InvalidFullInput');
        end
        function validityRequiresOneBooleanPerFrame(testCase)
            [d,l,c]=fixture();d.highRate.bodyVelocityValid=[true;false];
            testCase.verifyError(@()execute(d,l,c),'VehicleLocalization:InvalidMotionValidity');
        end
    end
end

function [d,l,c]=fixture()
    t=(0:.1:1).';zero=zeros(size(t));
    d=struct('highRate',struct('time',t,'longitudinalSpeed',8+zero, ...
        'longitudinalAcceleration',zero,'lateralAcceleration',zero,'yawRate',zero));
    l=struct('time',t,'lateralVelocity',zero,'sideSlipAngle',zero,'sideSlipAngleRate',zero);
    c=mncavFullObserverConfig();
    c.initialState=[0;3;.2;0;0;0;0];
end

function r=execute(d,l,c)
    r=runFullLocalizationObserver(d,struct(),c,LateralInputs=l);
end
