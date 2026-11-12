classdef (SharedTestFixtures={freshObserverGainsFixture}) invariantObserverTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));setupVehicleLocalization;
        end
    end
    methods (Test)
        function courseAccelerationUsesOnlyValidPastAndPresentSamples(testCase)
            t=[0;.1;.3;.6;1];rate=.2+.3*t;valid=[true;true;false;true;true];
            rate(3)=NaN;
            [qd,available]=localizationSupport.courseRateDerivative(t,rate,valid);
            testCase.verifyEqual(available,[false;true;false;false;true]);
            testCase.verifyEqual(qd,[0;.3;0;0;.3],AbsTol=1e-12);
        end
        function courseAccelerationEntersTheRuntimePredictor(testCase)
            [data,lateral,c]=fixture();t=lateral.time;
            data.highRate.yawRate=.1+.2*t;lateral.sideSlipAngleRate=.03*t;
            result=runFullLocalizationObserver(data,c,LateralInputs=lateral);
            testCase.verifyEqual(result.diagnostics.courseAngularAcceleration,[0;.23+zeros(numel(t)-1,1)],AbsTol=1e-12);
            testCase.verifyTrue(result.diagnostics.intrinsicPredictionUsesImu);
            data.highRate.courseAngularAcceleration=.4+zeros(size(t));
            explicit=runFullLocalizationObserver(data,c,LateralInputs=lateral);
            testCase.verifyEqual(explicit.diagnostics.courseAngularAcceleration,data.highRate.courseAngularAcceleration);
            testCase.verifyGreaterThan(norm(explicit.diagnostics.intrinsicPrediction-result.diagnostics.intrinsicPrediction),1e-5);
        end
        function explicitCourseAccelerationTracksAnIncreasingTurnRate(testCase)
            h=.001;t=(0:h:2).';speed=12;q0=.05;qd=.1;
            x=[0;speed;0;0;0;speed*q0;0;0];old=x;
            for k=2:numel(t)
                q=q0+qd*t(k);
                x=localizationSupport.predict(x,h,q,qd);
                old=localizationSupport.predict(old,h,q,0);
            end
            angle=q0*t(end)+qd*t(end)^2/2;
            truth=speed*[cos(angle);sin(angle)];
            newError=norm(x([2,5])-truth);oldError=norm(old([2,5])-truth);
            fprintf('TURN-RAMP prediction: velocity error new %.9g, omitted-qdot %.9g m/s\n',newError,oldError);
            testCase.verifyLessThan(newError,.02);
            testCase.verifyLessThan(newError,.02*oldError);
        end
        function courseAccelerationDomainChangeRejectsStaleGains(testCase)
            c=freshObserverTestConfig();c.maximumTrackAngleAcceleration=2*c.maximumTrackAngleAcceleration;
            testCase.verifyError(@()localizationSupport.runtime(c),'VehicleLocalization:StaleInvariantDesign');
        end
        function varyingTurnHasOnlyTangentialSpeedCurvatureResidual(testCase)
            t=.7;step=1e-5;s=12+.4*t+.15*t^2;b=.4+.3*t;
            angle=.2+.1*t+.2*t^2;q=.1+.4*t;qd=.4;
            R=[cos(angle),-sin(angle);sin(angle),cos(angle)];J=[0,-1;1,0];
            v=R*[s;0];a=R*[b;s*q];
            ap=analyticAcceleration(t+step);am=analyticAcceleration(t-step);
            residual=(ap-am)/(2*step)-(q^2*eye(2)+qd*J)*v-2*q*J*a;
            testCase.verifyEqual(residual,R*[.3;0],AbsTol=1e-8);
            x=[0;v(1);a(1);0;v(2);a(2);angle;q];h=1e-7;
            next=localizationSupport.predict(x,h,0,qd);
            testCase.verifyEqual((next([3,6])-a)/h,(q^2*eye(2)+qd*J)*v+2*q*J*a,AbsTol=2e-6);
        end
        function temporaryIntrinsicCertificateIsRecomputed(testCase)
            cfg=freshObserverTestConfig();d=localizationSupport.runtime(cfg);v=d.continuousCertificate.verification;
            testCase.verifyTrue(v.certified);testCase.verifyEqual(v.vertices,128);
            testCase.verifyLessThan(v.worstVertexEigenvalue,0);testCase.verifyFalse(v.actualOverlapCertified);
            testCase.verifyTrue(v.nominalFusionLmiVerified);testCase.verifyEqual(v.nominalFusion.vertices,128);
            testCase.verifyTrue(v.operatingSet.assumedInvariant);
            testCase.verifyFalse(v.operatingSet.invarianceVerified);
            testCase.verifySize(d.P,[8,8]);
        end
        function staleArchitectureInputsCannotReuseTheGain(testCase)
            c=freshObserverTestConfig();c.invariantObserver.headingLengthM=20;
            testCase.verifyError(@()localizationSupport.runtime(c), ...
                'VehicleLocalization:StaleInvariantDesign');
        end
        function headingOnlyMixedCertificateCannotBeReused(testCase)
            c=freshObserverTestConfig();c.invariantDesign.inputs.architecture='intrinsic-vector-iss-v3';
            testCase.verifyError(@()localizationSupport.runtime(c), ...
                'VehicleLocalization:StaleInvariantDesign');
        end
        function parameterFactoryHasNoSolvedGains(testCase)
            c=mncavFullObserverConfig();
            testCase.verifyFalse(isfield(c,'invariantDesign'));
            testCase.verifyEqual(c.gainSource,"unsolved-invariant");
        end
        function publicScriptResolvesEvenWhenAnOldDesignIsSupplied(testCase)
            c=freshObserverTestConfig();c.invariantDesign.kappaL=0;
            [d,ready]=synthesizeMncavInvariantDesign(c);
            testCase.verifyGreaterThan(d.kappaL,0);
            testCase.verifyTrue(d.verification.nominalFusionLmiVerified);
            testCase.verifyEqual(ready.invariantDesign,d);
            testCase.verifyEqual(ready.gainSource,"fresh-lmi");
            testCase.verifyEqual(d.inputs,localizationSupport.invariantInputs(ready));
        end
        function synchronizationWithdrawsAnInvalidBracketEndpoint(testCase)
            [d,l,c]=fixture();t=(.05:.1:.95).';n=numel(t);
            d.highRate.bodyVelocityValid=true(size(l.time));d.highRate.bodyVelocityValid(2)=false;
            d.highRate.bodyAccelerationValid=true(size(l.time));d.highRate.yawRateValid=true(size(l.time));
            d.lidar=struct('time',t,'pose',zeros(n,3),'valid',true(n,1), ...
                'information',repmat(eye(3),1,1,n),'delay',0);
            c.synchronization.motionMaximumBracket=.2;
            [a,~,~]=localizationSupport.synchronizeLocalizationInputs(d,l,c);
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
            [~,aligned]=localizationSupport.synchronizeLocalizationInputs(d,l,c);
            testCase.verifyEqual(aligned.sideSlipAngle,interp1(l.time,l.sideSlipAngle,t),AbsTol=1e-12);
            testCase.verifyEqual(aligned.sideSlipAngleRate,interp1(l.time,l.sideSlipAngleRate,t),AbsTol=1e-12);
        end
        function bodyLateralOutputDoesNotUseTheSuppliedAngle(testCase)
            [d,l,c]=fixture();baseline=runFullLocalizationObserver(d,c,LateralInputs=l);
            l.sideSlipAngle(:)=.2;
            supplied=runFullLocalizationObserver(d,c,LateralInputs=l);
            first=supplied.diagnostics.invariantOutputs{1};
            testCase.verifyEqual(first.predicted(4),-8*sin(c.initialState(7)),AbsTol=1e-12);
            testCase.verifyEqual(supplied.internalState,baseline.internalState,AbsTol=1e-12);
            testCase.verifyEqual(supplied.lateral.sideSlipAngle,l.sideSlipAngle,AbsTol=1e-12);
        end
        function missingSideSlipAngleCannotBeReconstructed(testCase)
            [d,l,c]=fixture();l=rmfield(l,'sideSlipAngle');
            testCase.verifyError(@()runFullLocalizationObserver(d,c,LateralInputs=l), ...
                'VehicleLocalization:AlignedLateralRequired');
        end
        function availableSideSlipAngleMustBeFinite(testCase)
            [d,l,c]=fixture();l.sideSlipAngle(2)=NaN;
            testCase.verifyError(@()runFullLocalizationObserver(d,c,LateralInputs=l), ...
                'VehicleLocalization:InvalidFullInput');
        end
        function withdrawnAnglePayloadCannotAffectTheEstimate(testCase)
            [d,l,c]=fixture();d.highRate.bodyVelocityValid=false(size(l.time));
            baseline=runFullLocalizationObserver(d,c,LateralInputs=l);
            l.sideSlipAngle(:)=NaN;
            poisoned=runFullLocalizationObserver(d,c,LateralInputs=l);
            testCase.verifyEqual(poisoned.internalState,baseline.internalState,AbsTol=1e-12);
        end
        function stoppedPacketHasFourOutputsAndSeparateGyro(testCase)
            c=freshObserverTestConfig();
            packet=localizationSupport.measurements(0,0,0,0,.2,true,true,true,c);
            testCase.verifySize(packet.value,[4,1]);
            testCase.verifyEqual(packet.gyro,.2,AbsTol=1e-12);
            testCase.verifyFalse(isfield(packet,'beta'));
            testCase.verifyFalse(isfield(packet,'betaRate'));
            x=[0;.1;0;0;.2;0;0;0];design=localizationSupport.runtime(c);
            corrected=localizationSupport.correct(x,packet,.1,design,c);
            testCase.verifyTrue(all(isfinite(corrected)));
        end
        function squaredSpeedGainIsActiveAndHasNoScalarReplacement(testCase)
            c=freshObserverTestConfig();design=localizationSupport.runtime(c);
            testCase.verifyGreaterThan(design.auxiliaryWeights(1),0);
            testCase.verifyFalse(isfield(design,'speedGainNormalized'));
            testCase.verifyFalse(isfield(c.invariantDesign,'speedDesign'));
        end
        function nominalResponseReplacesRemovedRegionalDecayFields(testCase)
            c=freshObserverTestConfig();v=c.invariantDesign.verification;
            testCase.verifyFalse(isfield(c.invariantObserver,'minimumDirectRatesPerSecond'));
            testCase.verifyFalse(isfield(c.invariantObserver.fusion,'nominalDecayRatePerSecond'));
            testCase.verifyFalse(isfield(v.feedbackAuthority,'requiredDirectRatesPerSecond'));
            testCase.verifyFalse(isfield(v,'regionalIss'));
            testCase.verifyTrue(v.allAuxiliaryOutputsRequired);
        end
        function eightOutputCertificateCannotBeReused(testCase)
            c=freshObserverTestConfig();c.invariantDesign.inputs.architecture='intrinsic-vector-iss-v5';
            testCase.verifyError(@()localizationSupport.runtime(c), ...
                'VehicleLocalization:StaleInvariantDesign');
        end
        function projectedSpeedCertificateCannotBeReused(testCase)
            c=freshObserverTestConfig();c.invariantDesign.inputs.architecture='intrinsic-vector-iss-v4';
            testCase.verifyError(@()localizationSupport.runtime(c), ...
                'VehicleLocalization:StaleInvariantDesign');
        end
        function disablingSquaredSpeedCannotReuseACertifiedClaim(testCase)
            c=freshObserverTestConfig();c.invariantDesign.auxiliaryWeights(1)=0;
            verification=localizationSupport.verifyInvariantDesign(c.invariantDesign,c);
            testCase.verifyFalse(verification.certified);
        end
        function temporaryDesignVerificationRequiresNoSdpSolver(testCase)
            c=freshObserverTestConfig();
            originalPath=path;cleanup=onCleanup(@()path(originalPath));
            root=fileparts(fileparts(mfilename('fullpath')));restoredefaultpath;addpath(root);setupVehicleLocalization;
            testCase.verifyEqual(exist('sdpvar','file'),0);
            v=localizationSupport.verifyInvariantDesign(c.invariantDesign,c);
            testCase.verifyTrue(v.certified);
        end
        function imuCannotDrivePredictionWithAuxiliaryFeedbackWithdrawn(testCase)
            [d,l,c]=fixture();c.initialState(8)=.05;
            d.highRate.yawRateValid=false(size(l.time));
            d.highRate.bodyVelocityValid=false(size(l.time));
            d.highRate.bodyAccelerationValid=false(size(l.time));
            a=runFullLocalizationObserver(d,c,LateralInputs=l);
            d.highRate.yawRate(:)=-10;d.highRate.longitudinalAcceleration(:)=30;d.highRate.lateralAcceleration(:)=-40;
            b=runFullLocalizationObserver(d,c,LateralInputs=l);
            testCase.verifyEqual(a.internalState,b.internalState);
            testCase.verifyEqual(a.diagnostics.intrinsicPrediction(2,7),c.initialState(7)+.05*l.time(2),AbsTol=1e-12);
            testCase.verifyFalse(a.diagnostics.intrinsicPredictionUsesImu);
        end
        function gyroEntersAsAnInnovationOnAnEstimatedState(testCase)
            [d,l,c]=fixture();c.initialState(:)=0;d.highRate.longitudinalSpeed(:)=NaN;l.lateralVelocity(:)=NaN;
            d.highRate.bodyVelocityValid=false(size(l.time));d.highRate.bodyAccelerationValid=false(size(l.time));
            d.highRate.yawRate(:)=.2;
            a=runFullLocalizationObserver(d,c,LateralInputs=l);
            testCase.verifyEqual(a.diagnostics.intrinsicPrediction(2,7:8),[0,0]);
            testCase.verifyGreaterThan(a.yawRate(2),0);testCase.verifyLessThan(a.yawRate(2),.2);
            testCase.verifyGreaterThan(a.headingUnwrapped(end),0);
            testCase.verifyFalse(a.diagnostics.intrinsicPredictionUsesImu);
            testCase.verifyTrue(all(isfinite(a.internalState),'all'));
        end
        function stoppedRotatingVehicleHasNoSpeedDivision(testCase)
            [d,l,c]=fixture();c.initialState(:)=0;d.highRate.longitudinalSpeed(:)=0;d.highRate.yawRate(:)=.2;
            a=runFullLocalizationObserver(d,c,LateralInputs=l);
            testCase.verifyEqual(a.position,zeros(numel(l.time),2),AbsTol=1e-12);
            testCase.verifyTrue(all(isfinite(a.internalState),'all'));
        end
        function informativePoseRecoversUnmeasuredVelocity(testCase)
            % With all velocity outputs withdrawn, the nominal fusion LMI
            % promises no recovery time. Retain the accuracy target at 20 s.
            t=(0:.2:20).';zero=zeros(size(t));n=numel(t);
            d=struct('highRate',struct('time',t,'longitudinalSpeed',nan(n,1), ...
                'bodyVelocityValid',false(n,1),'longitudinalAcceleration',zero, ...
                'lateralAcceleration',zero,'yawRate',zero));
            d.lidar=struct('time',t,'pose',[5*t,zero,zero],'valid',true(n,1), ...
                'information',repmat(1e6*eye(3),1,1,n),'delay',0);
            l=struct('time',t,'lateralVelocity',nan(n,1),'sideSlipAngle',zero,'sideSlipAngleRate',zero);
            c=freshObserverTestConfig();c.initialState=[.2;0;0;-.1;0;0;.02];
            a=runFullLocalizationObserver(d,c,LateralInputs=l);
            fprintf('POSE-ONLY 20 s: position %.9g m, velocity %.9g m/s\n', ...
                norm(a.position(end,:)-[100,0]),norm(a.velocity(end,:)-[5,0]));
            testCase.verifyLessThan(norm(a.position(end,:)-[100,0]),.02);
            testCase.verifyLessThan(norm(a.velocity(end,:)-[5,0]),.05);
            testCase.verifyEqual(a.diagnostics.packetCounts.bodyVelocityValid,0);
        end
        function simultaneousFrameRotationPreservesEveryInvariant(testCase)
            c=freshObserverTestConfig();x=[2;8;.2;-1;.4;1.3;.5;.12];angle=.7;
            R=[cos(angle),-sin(angle);sin(angle),cos(angle)];rotated=x;
            for ids={[1,4],[2,5],[3,6]},i=ids{1};rotated(i)=R*x(i);end
            rotated(7)=x(7)+angle;
            a=localizationSupport.outputs(x,c);b=localizationSupport.outputs(rotated,c);
            testCase.verifyEqual(a,b,AbsTol=1e-12);
        end
        function invariantJacobianAgreesWithIndependentFiniteDifferences(testCase)
            c=freshObserverTestConfig();x=[2;8;.2;-1;.4;1.3;.5;.12];[~,H]=localizationSupport.outputs(x,c);
            numerical=zeros(4,8);step=1e-5;
            for j=1:8
                a=x;b=x;a(j)=a(j)+step;b(j)=b(j)-step;
                numerical(:,j)=(localizationSupport.outputs(a,c)-localizationSupport.outputs(b,c))/(2*step);
            end
            testCase.verifyEqual(H,numerical,AbsTol=1e-8);
        end
        function bodyMeasurementsMatchInvariantsWithoutKnowingGlobalYaw(testCase)
            c=freshObserverTestConfig();v=[8;.4];a=[.2;1.3];psi=.7;betaRate=.015;
            R=[cos(psi),-sin(psi);sin(psi),cos(psi)];vg=R*v;ag=R*a;
            r=(v(1)*a(2)-v(2)*a(1))/(v.'*v)-betaRate;x=[0;vg(1);ag(1);0;vg(2);ag(2);psi;r];
            packet=localizationSupport.measurements(v(1),v(2),a(1),a(2),r,true,true,true,c);
            y=localizationSupport.outputs(x,c);
            testCase.verifyEqual(y,packet.value,AbsTol=1e-12);
        end
    end
end

function [d,l,c]=fixture()
    t=(0:.1:1).';zero=zeros(size(t));
    d=struct('highRate',struct('time',t,'longitudinalSpeed',8+zero, ...
        'longitudinalAcceleration',zero,'lateralAcceleration',zero,'yawRate',zero));
    l=struct('time',t,'lateralVelocity',zero,'sideSlipAngle',zero,'sideSlipAngleRate',zero);
    c=freshObserverTestConfig();c.initialState=[0;8;0;0;0;0;.1];
end

function a=analyticAcceleration(t)
    s=12+.4*t+.15*t^2;b=.4+.3*t;q=.1+.4*t;angle=.2+.1*t+.2*t^2;
    a=[cos(angle),-sin(angle);sin(angle),cos(angle)]*[b;s*q];
end
