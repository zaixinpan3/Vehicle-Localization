classdef (SharedTestFixtures={freshObserverGainsFixture}) invariantObserverTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));setupVehicleLocalization;
        end
    end
    methods (Test)
        function varyingTurnResidualRetainsUnmeasuredCourseAcceleration(testCase)
            % An independent analytic trajectory: s(t), phi(t) vary smoothly.
            t=.7;s=12+.3*t+.02*t^2;sdot=.3+.04*t;sddot=.04;
            phi=.2+.1*t+.015*t^2;q=.1+.03*t;qdot=.03;
            R=[cos(phi),-sin(phi);sin(phi),cos(phi)];J=[0,-1;1,0];
            v=R*[s;0];a=R*[sdot;s*q];
            jerk=R*[sddot-s*q^2;2*sdot*q+s*qdot];
            residual=jerk-q^2*v-2*q*J*a;
            testCase.verifyEqual(residual,R*[sddot;s*qdot],AbsTol=1e-13);
            testCase.verifyGreaterThan(norm(residual),abs(sddot));
            % Recover the actual implicit predictor drift, not a duplicate model.
            x=[0;v(1);a(1);0;v(2);a(2);phi;q];h=.02;
            xp=localizationSupport.predict(x,h,mncavFullObserverConfig());
            drift=(xp-x)/h;
            testCase.verifyEqual(drift([3,6]),q^2*xp([2,5])+2*q*J*xp([3,6]),AbsTol=1e-12);
        end
        function changeOfOperatingDomainRejectsStaleGains(testCase)
            c=freshObserverTestConfig();
            c.iss.maximumAcceleration=3;
            testCase.verifyError(@()localizationSupport.runtime(c), ...
                'VehicleLocalization:StaleInvariantDesign');
        end
        function temporaryIntrinsicCertificateIsRecomputed(testCase)
            cfg=freshObserverTestConfig();d=localizationSupport.runtime(cfg);v=d.continuousCertificate.verification;
            testCase.verifyTrue(v.certified);testCase.verifyEqual(v.vertices,4);
            testCase.verifyLessThan(v.worstVertexEigenvalue,0);testCase.verifyFalse(v.actualOverlapCertified);
            testCase.verifyTrue(v.nominalFusionLmiVerified);testCase.verifyEqual(v.nominalFusion.vertices,4);
            testCase.verifyTrue(v.operatingSet.assumedInvariant);
            testCase.verifyFalse(v.operatingSet.invarianceVerified);
            testCase.verifySize(d.P,[8,8]);
        end
        function staleArchitectureInputsCannotReuseTheGain(testCase)
            c=freshObserverTestConfig();c.invariantObserver.theta=3;
            testCase.verifyError(@()localizationSupport.runtime(c), ...
                'VehicleLocalization:StaleInvariantDesign');
        end
        function parameterFactoryHasNoSolvedGains(testCase)
            c=mncavFullObserverConfig();
            testCase.verifyFalse(isfield(c,'invariantDesign'));
            testCase.verifyEqual(c.gainSource,"unsolved-invariant");
        end
        function publicScriptResolvesEvenWhenAnOldDesignIsSupplied(testCase)
            c=freshObserverTestConfig();c.invariantDesign.inputs.architecture='invalid';
            [d,ready]=synthesizeMncavInvariantDesign(c);
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
            [a,~,~]=syntheticObserver.align(d,l,c);
            testCase.verifyEqual(a.highRate.bodyVelocityValid,[false;false;true(n-2,1)]);
            testCase.verifyTrue(all(a.highRate.bodyAccelerationValid));
            testCase.verifyTrue(all(a.highRate.yawRateValid));
        end
        function synchronizationRequiresOnlyLateralVelocity(testCase)
            [d,l,c]=fixture();t=(.05:.1:.95).';n=numel(t);
            d.lidar=struct('time',t,'pose',zeros(n,3),'valid',true(n,1), ...
                'information',repmat(eye(3),1,1,n),'delay',0);
            c.synchronization.motionMaximumBracket=.2;
            [~,aligned]=syntheticObserver.align(d,l,c);
            testCase.verifyFalse(isfield(aligned,'sideSlipAngle'));
            testCase.verifyEqual(aligned.lateralVelocity,interp1(l.time,l.lateralVelocity,t),AbsTol=1e-12);
        end
        function bodyLateralOutputRequiresNoAnglePayload(testCase)
            [d,l,c]=fixture();
            testCase.verifyFalse(isfield(l,'sideSlipAngle'));
            supplied=runFullLocalizationObserver(d,c,LateralInputs=l);
            first=supplied.diagnostics.invariantOutputs{1};
            testCase.verifyEqual(first.predicted(2),-8*sin(c.initialState(7)),AbsTol=1e-12);
            testCase.verifyFalse(isfield(supplied.lateral,'sideSlipAngle'));
        end
        function stoppedPacketHasOnlyFourMotionOutputs(testCase)
            c=freshObserverTestConfig();
            packet=localizationSupport.measurements(0,0,0,0,true,true,c);
            testCase.verifySize(packet.value,[4,1]);
            testCase.verifyFalse(isfield(packet,'yawRate'));
            testCase.verifyFalse(isfield(packet,'beta'));
            testCase.verifyFalse(isfield(packet,'betaRate'));
            x=[0;.1;0;0;.2;0;0;0];design=localizationSupport.runtime(c);
            corrected=localizationSupport.correct(x,packet,.1,design,c);
            testCase.verifyTrue(all(isfinite(corrected)));
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
        function upstreamLinearRateFeedbackMovesRateAndHeading(testCase)
            [d,l,c]=fixture();c.initialState(:)=0;d.highRate.longitudinalSpeed(:)=NaN;l.lateralVelocity(:)=NaN;
            d.highRate.bodyVelocityValid=false(size(l.time));d.highRate.bodyAccelerationValid=false(size(l.time));
            l.yawRate(:)=.2;
            a=runFullLocalizationObserver(d,c,LateralInputs=l);
            testCase.verifyEqual(a.diagnostics.intrinsicPrediction(2,7:8),[0,0]);
            dt=[0;diff(l.time)];
            expected=.2*(1-cumprod(1./(1+c.invariantDesign.verification.yawRateGainPerSecond*dt)));
            testCase.verifyEqual(a.yawRate,expected,AbsTol=1e-12);
            gain=a.observer.continuousDesign.linearOutputGain(7,4);
            testCase.verifyEqual(a.headingUnwrapped,cumsum(dt.*(expected+gain*(.2-expected))),AbsTol=1e-12);
            testCase.verifyFalse(a.diagnostics.intrinsicPredictionUsesImu);
            testCase.verifyTrue(all(isfinite(a.internalState),'all'));
        end
        function stoppedRotatingVehicleHasNoSpeedDivision(testCase)
            [d,l,c]=fixture();c.initialState(:)=0;d.highRate.longitudinalSpeed(:)=0;l.yawRate(:)=.2;
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
            l=struct('time',t,'lateralVelocity',nan(n,1));
            c=freshObserverTestConfig();c.initialState=[.2;0;0;-.1;0;0;.02];
            a=syntheticObserver.run(d,c,LateralInputs=l);
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
            c=freshObserverTestConfig();v=[8;.4];a=[.2;1.3];psi=.7;r=.14;
            R=[cos(psi),-sin(psi);sin(psi),cos(psi)];vg=R*v;ag=R*a;
            x=[0;vg(1);ag(1);0;vg(2);ag(2);psi;r];
            packet=localizationSupport.measurements(v(1),v(2),a(1),a(2),true,true,c);
            y=localizationSupport.outputs(x,c);
            testCase.verifyEqual(y,packet.value,AbsTol=1e-12);
        end
    end
end

function [d,l,c]=fixture()
    t=(0:.1:1).';zero=zeros(size(t));
    d=struct('highRate',struct('time',t,'longitudinalSpeed',8+zero, ...
        'longitudinalAcceleration',zero,'lateralAcceleration',zero,'yawRate',zero));
    l=struct('time',t,'lateralVelocity',zero,'yawRate',zero);
    c=freshObserverTestConfig();c.initialState=[0;8;0;0;0;0;.1];
end
