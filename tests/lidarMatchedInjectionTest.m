classdef lidarMatchedInjectionTest < matlab.unittest.TestCase
% lidarMatchedInjectionTest Geometry, fixed-metric energy and sampled steps.
    properties (TestParameter)
        angle={0,.2,1.3,2.8}
    end
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));setupVehicleLocalization;
        end
    end
    methods (Test)
        function idealCurbRetainsLongitudinalNullspace(testCase)
            f=fixture();m=curb([0;0;0]);
            [c,a]=computeLidarMatchedCorrection(m,[0;0;0],f.G,f.design,f.cfg);
            testCase.verifyEqual(a.filter.normalizedInformation,diag([0,3,8]),AbsTol=1e-12);
            testCase.verifyEqual(a.filter.rank,2);
            testCase.verifyEqual(c,zeros(7,1),AbsTol=0);
            testCase.verifyEqual(a.filter.S*[1;0;0],zeros(3,1),AbsTol=0);
        end
        function oneLeverArmRetainsLateralYawAmbiguity(testCase)
            f=fixture();m=buildLidarLineMeasurement([3,0;3,0],[3,0;3,0], ...
                [0,1;0,1],[0;0;0],ones(2,1));
            [~,a]=computeLidarMatchedCorrection(m,[0;0;0],f.G,f.design,f.cfg);
            testCase.verifyEqual(a.filter.rank,1);
            testCase.verifyEqual(a.filter.S*[0;-3;1],zeros(3,1),AbsTol=1e-12);
            testCase.verifyGreaterThan(abs(a.filter.S(2,3)),0);
        end
        function residualAndPoseChannelsAgreeForAffineTranslation(testCase)
            f=fixture();pose=[.7;.2;0];m=curb(pose);
            [a,info]=computeLidarMatchedCorrection(m,pose,f.G,f.design,f.cfg);
            p=struct('pose',zeros(3,1),'information',info.filter.normalizedInformation);
            b=computeLidarMatchedCorrection(p,pose,f.G,f.design,f.cfg);
            testCase.verifyEqual(a,b,AbsTol=1e-12);
        end
        function residualIsEvaluatedAtPrediction(testCase)
            f=fixture();pose=[0;.2;0];m=curb(pose);
            c=computeLidarMatchedCorrection(m,pose,f.G,f.design,f.cfg);
            testCase.verifyLessThan(c(4),0);
            testCase.verifyError(@()computeLidarMatchedCorrection(curb(zeros(3,1)),pose, ...
                f.G,f.design,f.cfg),'VehicleLocalization:StaleLidarLinearization');
        end
        function correlatedCopiesDoNotMultiplyInformation(testCase)
            f=fixture();single=struct('residual',.2,'jacobian',[0,1,0], ...
                'weights',1,'linearizationPose',zeros(3,1));
            duplicate=struct('residual',[.2;.2],'jacobian',[0,1,0;0,1,0], ...
                'weights',ones(2)/4,'linearizationPose',zeros(3,1));
            [a,aa]=computeLidarMatchedCorrection(single,zeros(3,1),f.G,f.design,f.cfg);
            [b,bb]=computeLidarMatchedCorrection(duplicate,zeros(3,1),f.G,f.design,f.cfg);
            testCase.verifyEqual(a,b,AbsTol=1e-12);
            testCase.verifyEqual(aa.filter.normalizedInformation,bb.filter.normalizedInformation,AbsTol=1e-12);
        end
        function changingDirectionsDissipateInANondiagonalMetric(testCase,angle)
            f=rotatedFixture(angle);e=-(f.design.T\f.state);
            [c,a]=computeLidarMatchedCorrection(f.measurement,f.G*f.state,f.G,f.design,f.cfg);
            rate=-2*e.'*f.design.P*(f.design.T\c);
            testCase.verifyEqual(rate,-2*e.'*a.dissipationMatrix*e,AbsTol=1e-10);
            testCase.verifyLessThanOrEqual(rate,1e-12);
        end
        function implicitStepCannotIncreaseAffineEnergy(testCase,angle)
            f=rotatedFixture(angle);e=-(f.design.T\f.state);
            [c,a]=computeLidarMatchedCorrection(f.measurement,f.G*f.state,f.G,f.design,f.cfg, ...
                StepSize=100,Discretization="implicit");
            after=e-f.design.T\c;
            testCase.verifyLessThanOrEqual(after.'*f.design.P*after,e.'*f.design.P*e+1e-10);
            testCase.verifyLessThanOrEqual(a.jumpMaximumEnergyRatio,1+1e-10);
            testCase.verifyEqual(a.appliedStep,100,AbsTol=0);
        end
        function explicitStepEnforcesStrictSpectralMargin(testCase,angle)
            f=rotatedFixture(angle);e=-(f.design.T\f.state);
            [c,a]=computeLidarMatchedCorrection(f.measurement,f.G*f.state,f.G,f.design,f.cfg, ...
                StepSize=100,Discretization="explicit");
            after=e-f.design.T\c;
            testCase.verifyEqual(a.explicitStepProduct,1.8,AbsTol=1e-12);
            testCase.verifyLessThanOrEqual(after.'*f.design.P*after,e.'*f.design.P*e+1e-10);
            testCase.verifyTrue(a.jumpNonexpansive);
        end
        function zeroInformationCannotCorrectInvisiblePose(testCase)
            f=fixture();m=struct('pose',[10;20;1],'information',zeros(3));
            [c,a]=computeLidarMatchedCorrection(m,zeros(3,1),f.G,f.design,f.cfg, ...
                StepSize=1,Discretization="implicit");
            testCase.verifyEqual(c,zeros(7,1),AbsTol=0);
            testCase.verifyEqual(a.filter.effectiveDimension,0,AbsTol=0);
            testCase.verifyEqual(a.filter.rank,0);
            testCase.verifyEqual(a.jumpMaximumEnergyRatio,1,AbsTol=1e-12);
        end
        function frameRejectionWithdrawsAllCorrection(testCase)
            f=fixture();m=struct('pose',[1;1;1],'information',1e8*eye(3),'frameReliability',0);
            c=computeLidarMatchedCorrection(m,zeros(3,1),f.G,f.design,f.cfg);
            testCase.verifyEqual(c,zeros(7,1),AbsTol=0);
        end
        function repeatedEigenspacesUseEqualReliability(testCase)
            f=fixture();a=filterLidarPoseInformation(10*eye(3),f.cfg,1,[0;.5;1]);
            testCase.verifyEqual(a.S,zeros(3),AbsTol=0);
            testCase.verifyEqual(a.reliability,zeros(3,1),AbsTol=0);
        end
        function directionalRejectionAndSaturationRemainBounded(testCase)
            f=fixture();a=filterLidarPoseInformation(diag([0,1,1e12]),f.cfg,.5,[1;0;1]);
            testCase.verifyEqual(a.strengths(1:2),zeros(2,1),AbsTol=0);
            testCase.verifyLessThanOrEqual(max(a.strengths),.5);
            testCase.verifyGreaterThan(a.strengths(3),.499);
        end
        function normalizationSeparatesMetresAndRadians(testCase)
            f=fixture();f.cfg.lidar.poseScales=[2;3;.1];
            a=filterLidarPoseInformation(diag([1,2,3]),f.cfg);
            testCase.verifyEqual(a.normalizedInformation,diag([4,18,.03]),AbsTol=1e-12);
            testCase.verifyEqual(a.trace,22.03,AbsTol=1e-12);
        end
        function indefiniteInformationAndReliabilityAreRejected(testCase)
            f=fixture();
            testCase.verifyError(@()filterLidarPoseInformation(diag([1,1,-.1]),f.cfg), ...
                'VehicleLocalization:InvalidLidarInformation');
            testCase.verifyError(@()filterLidarPoseInformation(eye(3),f.cfg,1.1), ...
                'VehicleLocalization:InvalidLidarReliability');
        end
        function angleInnovationUsesLocalChart(testCase)
            f=fixture();m=struct('pose',[0;0;-pi+.01],'information',diag([0,0,100]));
            c=computeLidarMatchedCorrection(m,[0;0;pi-.01],f.G,f.design,f.cfg);
            testCase.verifyGreaterThan(c(7),0);
            testCase.verifyLessThan(c(7),.1);
        end
        function currentDesignDoesNotInventBaselineYawCertificate(testCase)
            cfg=fullObserverConfig();d=designFullObserverGains(cfg);
            testCase.verifyEqual(d.lidarMatched.T,eye(7),AbsTol=0);
            testCase.verifyEqual(d.lidarMatched.P(1:6,1:6),d.translationP,AbsTol=0);
            testCase.verifyFalse(d.baselineFullStateCertified);
        end
        function arbitraryPsdWeightingCanDestabilizeAFixedGain(testCase)
            v=[1;-1]/sqrt(2);N=[1,4;0,1];Q=v*v.';
            fixed=-eye(2)-2*N;scalar=-eye(2)-N;
            unsafe=-eye(2)-2*N*Q;matched=-eye(2)-2*Q;
            testCase.verifyLessThan(max(real(eig(fixed))),0);
            testCase.verifyLessThan(max(real(eig(scalar))),0);
            testCase.verifyGreaterThan(max(real(eig(unsafe))),0);
            testCase.verifyLessThan(max(real(eig(matched))),0);
        end
        function pointToLineJacobianIncludesRotationAndTranslation(testCase)
            pose=[.2;-.1;.7];points=[2,1;-1,3];map=[3,-1;2,4];normals=[.6,.8;0,1];
            m=buildLidarLineMeasurement(points,map,normals,pose,ones(2,1));
            numeric=lineJacobian(points,map,normals,pose);
            testCase.verifyEqual(m.jacobian,numeric,AbsTol=1e-8);
        end
        function rapidlySwitchingRankAndDirectionsKeepJumpEnergy(testCase)
            changes=switchingEnergy();
            testCase.verifyLessThanOrEqual(max(changes),1e-10);
        end
        function indefiniteMetricCannotBeUsedAsACertificate(testCase)
            f=fixture();f.design.P(7,7)=-1;m=struct('pose',zeros(3,1),'information',eye(3));
            testCase.verifyError(@()computeLidarMatchedCorrection(m,zeros(3,1),f.G,f.design,f.cfg), ...
                'VehicleLocalization:InvalidLidarMetric');
        end
    end
end

function f=fixture()
    f.cfg=fullObserverConfig();d=designFullObserverGains(f.cfg);f.design=d.lidarMatched;
    f.G=zeros(3,7);f.G(1,1)=1;f.G(2,4)=1;f.G(3,7)=1;
end

function f=rotatedFixture(angle)
    f=fixture();B=eye(7);B(1,2)=.3;B(4,7)=.4;f.design.P=B.'*B;
    f.design.T=diag(2.^[1,2,3,1,2,3,1]);
    U=[cos(angle),-sin(angle),0;sin(angle),cos(angle),0;0,0,1];
    f.measurement=struct('pose',zeros(3,1),'information',U*diag([0,20,2])*U.');
    f.state=[.3;.2;-.1;-.4;.1;.2;.05];
end

function m=curb(pose)
    p=[-2,0;0,0;2,0];m=buildLidarLineMeasurement(p,p,repmat([0,1],3,1),pose,ones(3,1));
end

function J=lineJacobian(points,map,normals,pose)
    J=zeros(size(points,1),3);step=1e-6;
    for k=1:3
        direction=zeros(3,1);direction(k)=step;
        plus=buildLidarLineMeasurement(points,map,normals,pose+direction,ones(size(points,1),1));
        minus=buildLidarLineMeasurement(points,map,normals,pose-direction,ones(size(points,1),1));
        J(:,k)=(plus.residual-minus.residual)/(2*step);
    end
end

function changes=switchingEnergy()
    original=rng;cleanup=onCleanup(@()rng(original));rng(20261001,'twister');
    f=rotatedFixture(0);state=f.state;changes=zeros(250,1);
    for k=1:numel(changes)
        [U,~]=qr(randn(3));values=10.^(-4+8*rand(3,1));values(1:mod(k,4))=0;
        m=struct('pose',zeros(3,1),'information',U*diag(values)*U.', ...
            'frameReliability',double(mod(k,7)~=0),'directionReliability',rand(3,1));
        before=f.design.T\state;
        delta=computeLidarMatchedCorrection(m,f.G*state,f.G,f.design,f.cfg, ...
            StepSize=.01+rand,Discretization="implicit");
        state=state+delta;after=f.design.T\state;
        changes(k)=after.'*f.design.P*after-before.'*f.design.P*before;
    end
end
