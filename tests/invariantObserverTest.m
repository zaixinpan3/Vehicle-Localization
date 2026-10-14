classdef invariantObserverTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));setupVehicleLocalization;
        end
    end
    methods (Test)
        function storedIntrinsicCertificateIsRecomputed(testCase)
            cfg=mncavFullObserverConfig();d=invariantObserverSupport.runtime(cfg);v=d.continuousCertificate.verification;
            testCase.verifyTrue(v.certified);testCase.verifyEqual(v.vertices,256);
            testCase.verifyLessThan(v.worstVertexEigenvalue,0);testCase.verifyFalse(v.actualOverlapCertified);
            testCase.verifyTrue(v.jointSensorLmiVerified);testCase.verifyEqual(v.jointSensor.vertices,5888);
            testCase.verifySize(d.P,[8,8]);
        end
        function staleArchitectureInputsCannotReuseTheGain(testCase)
            testCase.verifyError(@()mncavFullObserverConfig(DesignOverride=struct('headingLengthM',20)), ...
                'VehicleLocalization:StaleInvariantDesign');
        end
        function synchronizationWithdrawsAnInvalidBracketEndpoint(testCase)
            [d,l,c]=fixture();t=(.05:.1:.95).';n=numel(t);
            d.highRate.bodyVelocityValid=true(size(l.time));d.highRate.bodyVelocityValid(2)=false;
            d.highRate.bodyAccelerationValid=true(size(l.time));d.highRate.yawRateValid=true(size(l.time));
            d.lidar=struct('time',t,'pose',zeros(n,3),'valid',true(n,1), ...
                'information',repmat(eye(3),1,1,n),'delay',0);
            c.synchronization.motionMaximumBracket=.2;
            [a,~,~]=fullObserverSupport.synchronizeLocalizationInputs(d,l,c);
            testCase.verifyEqual(a.highRate.bodyVelocityValid,[false;false;true(n-2,1)]);
            testCase.verifyTrue(all(a.highRate.bodyAccelerationValid));
            testCase.verifyTrue(all(a.highRate.yawRateValid));
        end
        function synchronizationPreservesTheUpstreamAngleAndRate(testCase)
            [d,l,c]=fixture();t=(.05:.1:.95).';n=numel(t);
            l.sideSlipAngle=.02*l.time.^2;l.sideSlipAngleRate=.04*l.time;
            d.lidar=struct('time',t,'pose',zeros(n,3),'valid',true(n,1), ...
                'information',repmat(eye(3),1,1,n),'delay',0);
            c.synchronization.motionMaximumBracket=.2;
            [~,aligned]=fullObserverSupport.synchronizeLocalizationInputs(d,l,c);
            testCase.verifyEqual(aligned.sideSlipAngle,interp1(l.time,l.sideSlipAngle,t),AbsTol=1e-12);
            testCase.verifyEqual(aligned.sideSlipAngleRate,interp1(l.time,l.sideSlipAngleRate,t),AbsTol=1e-12);
        end
        function auxiliaryDirectionUsesTheSuppliedAngle(testCase)
            [d,l,c]=fixture();baseline=runFullLocalizationObserver(d,struct(),c,LateralInputs=l);
            l.sideSlipAngle(:)=.2;
            supplied=runFullLocalizationObserver(d,struct(),c,LateralInputs=l);
            first=supplied.diagnostics.invariantOutputs{1};
            testCase.verifyEqual(first.predicted(6),-8*sin(c.initialState(7)+.2),AbsTol=1e-12);
            testCase.verifyGreaterThan(norm(supplied.internalState(end,:)-baseline.internalState(end,:)),1e-4);
            testCase.verifyEqual(supplied.lateral.sideSlipAngle,l.sideSlipAngle,AbsTol=1e-12);
        end
        function missingSideSlipAngleCannotBeReconstructed(testCase)
            [d,l,c]=fixture();l=rmfield(l,'sideSlipAngle');
            testCase.verifyError(@()runFullLocalizationObserver(d,struct(),c,LateralInputs=l), ...
                'VehicleLocalization:AlignedLateralRequired');
        end
        function availableSideSlipAngleMustBeFinite(testCase)
            [d,l,c]=fixture();l.sideSlipAngle(2)=NaN;
            testCase.verifyError(@()runFullLocalizationObserver(d,struct(),c,LateralInputs=l), ...
                'VehicleLocalization:InvalidFullInput');
        end
        function withdrawnAnglePayloadCannotAffectTheEstimate(testCase)
            [d,l,c]=fixture();d.highRate.bodyVelocityValid=false(size(l.time));
            baseline=runFullLocalizationObserver(d,struct(),c,LateralInputs=l);
            l.sideSlipAngle(:)=NaN;
            poisoned=runFullLocalizationObserver(d,struct(),c,LateralInputs=l);
            testCase.verifyEqual(poisoned.internalState,baseline.internalState,AbsTol=1e-12);
        end
        function stoppedPacketKeepsTheContinuousUpstreamAngle(testCase)
            c=mncavFullObserverConfig();
            packet=invariantObserverSupport.measurements(0,0,0,0,.2,.3,-.1,true,true,true,c);
            testCase.verifyEqual(packet.beta,.3,AbsTol=1e-12);
            testCase.verifyEqual(packet.betaRate,-.1,AbsTol=1e-12);
            x=[0;.1;0;0;.2;0;0;0];design=invariantObserverSupport.runtime(c);
            corrected=invariantObserverSupport.correct(x,packet,.1,design,c);
            testCase.verifyTrue(all(isfinite(corrected)));
        end
        function projectedSpeedCorrectionUsesTheSuppliedAngle(testCase)
            c=mncavFullObserverConfig();design=invariantObserverSupport.runtime(c);
            design.auxiliaryWeights(:)=0;
            packet=invariantObserverSupport.measurements(8,0,0,0,0,.3,0,true,true,false,c);
            x=[0;5;0;0;0;0;0;0];dt=.1;
            corrected=invariantObserverSupport.correct(x,packet,dt,design,c);
            direction=[cos(.3);sin(.3)];gain=design.theta*design.speedGainNormalized;
            expected=x([2,5])+dt*gain/(1+dt*gain)*direction*(8-direction.'*x([2,5]));
            testCase.verifyEqual(corrected([2,5]),expected,AbsTol=1e-12);
        end
        function storedDesignVerificationRequiresNoSdpSolver(testCase)
            originalPath=path;cleanup=onCleanup(@()path(originalPath));
            root=fileparts(fileparts(mfilename('fullpath')));restoredefaultpath;addpath(root);setupVehicleLocalization;
            testCase.verifyEqual(exist('sdpvar','file'),0);
            c=mncavFullObserverConfig();v=invariantObserverSupport.verify(c.invariantDesign,c);
            testCase.verifyTrue(v.certified);
        end
        function imuCannotDrivePredictionWithAuxiliaryFeedbackWithdrawn(testCase)
            [d,l,c]=fixture();c.initialState(8)=.05;
            d.highRate.yawRateValid=false(size(l.time));
            d.highRate.bodyVelocityValid=false(size(l.time));
            d.highRate.bodyAccelerationValid=false(size(l.time));
            a=runFullLocalizationObserver(d,struct(),c,LateralInputs=l);
            d.highRate.yawRate(:)=-10;d.highRate.longitudinalAcceleration(:)=30;d.highRate.lateralAcceleration(:)=-40;
            b=runFullLocalizationObserver(d,struct(),c,LateralInputs=l);
            testCase.verifyEqual(a.internalState,b.internalState);
            testCase.verifyEqual(a.diagnostics.intrinsicPrediction(2,7),c.initialState(7)+.05*l.time(2),AbsTol=1e-12);
            testCase.verifyFalse(a.diagnostics.intrinsicPredictionUsesImu);
        end
        function gyroEntersAsAnInnovationOnAnEstimatedState(testCase)
            [d,l,c]=fixture();c.initialState(:)=0;d.highRate.longitudinalSpeed(:)=NaN;l.lateralVelocity(:)=NaN;
            d.highRate.bodyVelocityValid=false(size(l.time));d.highRate.bodyAccelerationValid=false(size(l.time));
            d.highRate.yawRate(:)=.2;
            a=runFullLocalizationObserver(d,struct(),c,LateralInputs=l);
            testCase.verifyEqual(a.diagnostics.intrinsicPrediction(2,7:8),[0,0]);
            testCase.verifyGreaterThan(a.yawRate(2),0);testCase.verifyLessThan(a.yawRate(2),.2);
            testCase.verifyGreaterThan(a.headingUnwrapped(end),0);
            testCase.verifyFalse(a.diagnostics.intrinsicPredictionUsesImu);
            testCase.verifyTrue(all(isfinite(a.internalState),'all'));
        end
        function stoppedRotatingVehicleHasNoSpeedDivision(testCase)
            [d,l,c]=fixture();c.initialState(:)=0;d.highRate.longitudinalSpeed(:)=0;d.highRate.yawRate(:)=.2;
            a=runFullLocalizationObserver(d,struct(),c,LateralInputs=l);
            testCase.verifyEqual(a.position,zeros(numel(l.time),2),AbsTol=1e-12);
            testCase.verifyTrue(all(isfinite(a.internalState),'all'));
        end
        function informativePoseRecoversUnmeasuredVelocity(testCase)
            t=(0:.2:12).';zero=zeros(size(t));n=numel(t);
            d=struct('highRate',struct('time',t,'longitudinalSpeed',nan(n,1), ...
                'bodyVelocityValid',false(n,1),'longitudinalAcceleration',zero, ...
                'lateralAcceleration',zero,'yawRate',zero));
            d.lidar=struct('time',t,'pose',[5*t,zero,zero],'valid',true(n,1), ...
                'information',repmat(1e6*eye(3),1,1,n),'delay',0);
            l=struct('time',t,'lateralVelocity',nan(n,1),'sideSlipAngle',zero,'sideSlipAngleRate',zero);
            c=mncavFullObserverConfig();c.initialState=[.2;0;0;-.1;0;0;.02];
            a=runFullLocalizationObserver(d,struct(),c,LateralInputs=l);
            testCase.verifyLessThan(norm(a.position(end,:)-[60,0]),.02);
            testCase.verifyLessThan(norm(a.velocity(end,:)-[5,0]),.05);
            testCase.verifyEqual(a.diagnostics.packetCounts.bodyVelocityValid,0);
        end
        function simultaneousFrameRotationPreservesEveryInvariant(testCase)
            c=mncavFullObserverConfig();x=[2;8;.2;-1;.4;1.3;.5;.12];angle=.7;
            R=[cos(angle),-sin(angle);sin(angle),cos(angle)];rotated=x;
            for ids={[1,4],[2,5],[3,6]},i=ids{1};rotated(i)=R*x(i);end
            rotated(7)=x(7)+angle;
            a=invariantObserverSupport.outputs(x,.03,.02,c);b=invariantObserverSupport.outputs(rotated,.03,.02,c);
            testCase.verifyEqual(a,b,AbsTol=1e-12);
        end
        function invariantJacobianAgreesWithIndependentFiniteDifferences(testCase)
            c=mncavFullObserverConfig();x=[2;8;.2;-1;.4;1.3;.5;.12];[~,H]=invariantObserverSupport.outputs(x,.03,.02,c);
            numerical=zeros(8,8);step=1e-5;
            for j=1:8
                a=x;b=x;a(j)=a(j)+step;b(j)=b(j)-step;
                numerical(:,j)=(invariantObserverSupport.outputs(a,.03,.02,c)-invariantObserverSupport.outputs(b,.03,.02,c))/(2*step);
            end
            testCase.verifyEqual(H,numerical,AbsTol=1e-8);
        end
        function bodyMeasurementsMatchInvariantsWithoutKnowingGlobalYaw(testCase)
            c=mncavFullObserverConfig();v=[8;.4];a=[.2;1.3];psi=.7;betaRate=.015;
            R=[cos(psi),-sin(psi);sin(psi),cos(psi)];vg=R*v;ag=R*a;
            r=(v(1)*a(2)-v(2)*a(1))/(v.'*v)-betaRate;x=[0;vg(1);ag(1);0;vg(2);ag(2);psi;r];
            packet=invariantObserverSupport.measurements(v(1),v(2),a(1),a(2),r,atan2(v(2),v(1)),betaRate,true,true,true,c);
            y=invariantObserverSupport.outputs(x,packet.beta,betaRate,c);
            testCase.verifyEqual(y,packet.value,AbsTol=1e-12);
        end
    end
end

function [d,l,c]=fixture()
    t=(0:.1:1).';zero=zeros(size(t));
    d=struct('highRate',struct('time',t,'longitudinalSpeed',8+zero, ...
        'longitudinalAcceleration',zero,'lateralAcceleration',zero,'yawRate',zero));
    l=struct('time',t,'lateralVelocity',zero,'sideSlipAngle',zero,'sideSlipAngleRate',zero);
    c=mncavFullObserverConfig();c.initialState=[0;8;0;0;0;0;.1];
end
