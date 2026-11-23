classdef (SharedTestFixtures={freshObserverGainsFixture}) lidarMatchedInjectionTest < matlab.unittest.TestCase
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
            [c,a]=localizationSupport.computeLidarMatchedCorrection(m,[0;0;0],f.G,f.design,f.cfg);
            testCase.verifyEqual(a.filter.F,diag([0,3,8]),AbsTol=1e-12);
            testCase.verifyEqual(a.filter.rank,2);
            testCase.verifyEqual(c,zeros(7,1),AbsTol=0);
            testCase.verifyEqual(a.filter.S*[1;0;0],zeros(3,1),AbsTol=0);
        end
        function productionMetricGivesTheDirectPhysicalGain(testCase)
            cfg=freshObserverTestConfig();r=localizationSupport.runtime(cfg);
            G=eye(8);G=G([1,4,7],:);
            m=struct('gradient',[.2;-.1;.04],'information',diag([3,2,.8]), ...
                'linearizationPose',zeros(3,1));
            [rate,a]=localizationSupport.computeLidarMatchedCorrection(m,zeros(3,1),G,r.lidarMatched,cfg);
            C=(G*r.toPhysical)/r.theta;
            expected=-r.toPhysical*(r.P\(C.'*m.information*m.gradient));
            testCase.verifyEqual(rate,expected,AbsTol=1e-10);
            stale=r.lidarMatched;stale.unknownOption=1;
            testCase.verifyError(@()localizationSupport.computeLidarMatchedCorrection(m,zeros(3,1),G,stale,cfg), ...
                'VehicleLocalization:StaleInvariantDesign');
        end
        function oneLeverArmRetainsLateralYawAmbiguity(testCase)
            f=fixture();m=localizationSupport.buildLidarLineMeasurement([3,0;3,0],[3,0;3,0], ...
                [0,1;0,1],[0;0;0],ones(2,1));
            [~,a]=localizationSupport.computeLidarMatchedCorrection(m,[0;0;0],f.G,f.design,f.cfg);
            testCase.verifyEqual(a.filter.rank,1);
            testCase.verifyEqual(a.filter.S*[0;-3;1],zeros(3,1),AbsTol=1e-12);
            testCase.verifyGreaterThan(abs(a.filter.S(2,3)),0);
        end
        function directPoseIsRejected(testCase)
            f=fixture();packet=struct('pose',zeros(3,1),'information',eye(3));
            testCase.verifyError(@()localizationSupport.computeLidarMatchedCorrection(packet,[.7;.2;0], ...
                f.G,f.design,f.cfg),'VehicleLocalization:InvalidLidarMeasurement');
        end
        function residualIsEvaluatedAtPrediction(testCase)
            f=fixture();pose=[0;.2;0];m=curb(pose);
            c=localizationSupport.computeLidarMatchedCorrection(m,pose,f.G,f.design,f.cfg);
            testCase.verifyLessThan(c(4),0);
            testCase.verifyError(@()localizationSupport.computeLidarMatchedCorrection(curb(zeros(3,1)),pose, ...
                f.G,f.design,f.cfg),'VehicleLocalization:StaleLidarLinearization');
        end
        function correlatedCopiesDoNotMultiplyInformation(testCase)
            f=fixture();single=struct('residual',.2,'jacobian',[0,1,0], ...
                'weights',1,'linearizationPose',zeros(3,1));
            duplicate=struct('residual',[.2;.2],'jacobian',[0,1,0;0,1,0], ...
                'weights',ones(2)/4,'linearizationPose',zeros(3,1));
            [a,aa]=localizationSupport.computeLidarMatchedCorrection(single,zeros(3,1),f.G,f.design,f.cfg);
            [b,bb]=localizationSupport.computeLidarMatchedCorrection(duplicate,zeros(3,1),f.G,f.design,f.cfg);
            testCase.verifyEqual(a,b,AbsTol=1e-12);
            testCase.verifyEqual(aa.filter.F,bb.filter.F,AbsTol=1e-12);
        end
        function changingDirectionsDissipateInANondiagonalMetric(testCase,angle)
            f=rotatedFixture(angle);e=-(f.design.T\f.state);
            [c,a]=localizationSupport.computeLidarMatchedCorrection(f.measurement,f.G*f.state,f.G,f.design,f.cfg);
            rate=-2*e.'*f.design.P*(f.design.T\c);
            testCase.verifyEqual(rate,-2*e.'*a.dissipationMatrix*e,AbsTol=1e-10);
            testCase.verifyLessThanOrEqual(rate,1e-12);
        end
        function implicitStepCannotIncreaseAffineEnergy(testCase,angle)
            f=rotatedFixture(angle);e=-(f.design.T\f.state);
            [c,a]=localizationSupport.computeLidarMatchedCorrection(f.measurement,f.G*f.state,f.G,f.design,f.cfg, ...
                StepSize=100,Discretization="implicit");
            after=e-f.design.T\c;
            testCase.verifyLessThanOrEqual(after.'*f.design.P*after,e.'*f.design.P*e+1e-10);
            testCase.verifyLessThanOrEqual(a.jumpMaximumEnergyRatio,1+1e-10);
            testCase.verifyEqual(a.appliedStep,100,AbsTol=0);
        end
        function explicitStepEnforcesStrictSpectralMargin(testCase,angle)
            f=rotatedFixture(angle);e=-(f.design.T\f.state);
            [c,a]=localizationSupport.computeLidarMatchedCorrection(f.measurement,f.G*f.state,f.G,f.design,f.cfg, ...
                StepSize=100,Discretization="explicit");
            after=e-f.design.T\c;
            testCase.verifyEqual(a.explicitStepProduct,1.8,AbsTol=1e-12);
            testCase.verifyLessThanOrEqual(after.'*f.design.P*after,e.'*f.design.P*e+1e-10);
            testCase.verifyTrue(a.jumpNonexpansive);
        end
        function zeroInformationCannotCorrectInvisiblePose(testCase)
            f=fixture();m=struct('gradient',[10;20;1],'linearizationPose',zeros(3,1),'information',zeros(3));
            [c,a]=localizationSupport.computeLidarMatchedCorrection(m,zeros(3,1),f.G,f.design,f.cfg, ...
                StepSize=1,Discretization="implicit");
            testCase.verifyEqual(c,zeros(7,1),AbsTol=0);
            testCase.verifyEqual(a.filter.effectiveDimension,0,AbsTol=0);
            testCase.verifyEqual(a.filter.rank,0);
            testCase.verifyEqual(a.jumpMaximumEnergyRatio,1,AbsTol=1e-12);
        end
        function directGeometryScalesGradientAndDissipationDifferently(testCase)
            a=localizationSupport.filterLidarPoseInformation(diag([0,1,1e12]));
            testCase.verifyEqual(a.strengths(1:2),[0;1],AbsTol=0);
            testCase.verifyEqual(a.F,diag([0,1,1e12]));
            testCase.verifyEqual(a.strengths(3),1e24,RelTol=1e-14);
        end
        function scalingInformationScalesTheGainAndItsMatchedStrength(testCase)
            theta=.52;R=[cos(theta),0,-sin(theta);0,1,0;sin(theta),0,cos(theta)];
            I=R*diag([0,3,8])*R.';
            a=localizationSupport.filterLidarPoseInformation(I);
            b=localizationSupport.filterLidarPoseInformation(7*I);
            testCase.verifyEqual(a.F*I,a.S,AbsTol=2e-13);
            testCase.verifyEqual(b.F,7*a.F,AbsTol=3e-14);
            testCase.verifyEqual(b.S,49*a.S,AbsTol=2e-12);
        end
        function weakDirectionSurvivesAnisotropicGeometry(testCase)
            f=fixture();info=diag([0,1,1e12]);pose=zeros(3,1);
            m=struct('gradient',[3;2;0],'information',info,'linearizationPose',pose);
            [d,a]=localizationSupport.computeLidarMatchedCorrection(m,pose,f.G,f.design,f.cfg, ...
                StepSize=1,Discretization="implicit");
            testCase.verifyEqual(d(4),-8/5,AbsTol=1e-12);
            testCase.verifyEqual(d([1,7]),zeros(2,1),AbsTol=1e-12);
            testCase.verifyTrue(a.jumpNonexpansive);
        end
        function indefiniteAndOverflowingInformationAreRejected(testCase)
            testCase.verifyError(@()localizationSupport.filterLidarPoseInformation(diag([1,1,-.1])), ...
                'VehicleLocalization:InvalidLidarInformation');
            testCase.verifyError(@()localizationSupport.filterLidarPoseInformation(1e170*eye(3)), ...
                'VehicleLocalization:InvalidLidarInformation');
        end
        function currentDesignDoesNotInventBaselineYawCertificate(testCase)
            cfg=freshObserverTestConfig();d=localizationSupport.designFullObserverGains(cfg);
            testCase.verifySize(d.lidarMatched.T,[8,8]);
            testCase.verifyTrue(d.nominalFusionLmiVerified);
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
            m=localizationSupport.buildLidarLineMeasurement(points,map,normals,pose,ones(2,1));
            numeric=lineJacobian(points,map,normals,pose);
            testCase.verifyEqual(m.jacobian,numeric,AbsTol=1e-8);
        end
        function rapidlySwitchingRankAndDirectionsKeepJumpEnergy(testCase)
            changes=switchingEnergy();
            testCase.verifyLessThanOrEqual(max(changes),1e-10);
        end
        function indefiniteMetricCannotBeUsedAsACertificate(testCase)
            f=fixture();f.design.P(7,7)=-1;m=struct('gradient',zeros(3,1),'linearizationPose',zeros(3,1),'information',eye(3));
            testCase.verifyError(@()localizationSupport.computeLidarMatchedCorrection(m,zeros(3,1),f.G,f.design,f.cfg), ...
                'VehicleLocalization:InvalidLidarMetric');
        end
    end
end

function f=fixture()
    f.cfg=struct();
    f.design=struct('P',eye(7)/4,'T',eye(7),'theta',1);
    f.G=zeros(3,7);f.G(1,1)=1;f.G(2,4)=1;f.G(3,7)=1;
end

function f=rotatedFixture(angle)
    f=fixture();B=eye(7);B(1,2)=.3;B(4,7)=.4;f.design.P=B.'*B;
    f.design.T=diag(2.^[1,2,3,1,2,3,1]);
    U=[cos(angle),-sin(angle),0;sin(angle),cos(angle),0;0,0,1];
    f.state=[.3;.2;-.1;-.4;.1;.2;.05];
    information=U*diag([0,20,2])*U.';p=f.G*f.state;
    f.measurement=struct('gradient',information*p,'linearizationPose',p,'information',information);
end

function m=curb(pose)
    p=[-2,0;0,0;2,0];m=localizationSupport.buildLidarLineMeasurement(p,p,repmat([0,1],3,1),pose,ones(3,1));
end

function J=lineJacobian(points,map,normals,pose)
    J=zeros(size(points,1),3);step=1e-6;
    for k=1:3
        direction=zeros(3,1);direction(k)=step;
        plus=localizationSupport.buildLidarLineMeasurement(points,map,normals,pose+direction,ones(size(points,1),1));
        minus=localizationSupport.buildLidarLineMeasurement(points,map,normals,pose-direction,ones(size(points,1),1));
        J(:,k)=(plus.residual-minus.residual)/(2*step);
    end
end

function changes=switchingEnergy()
    original=rng;cleanup=onCleanup(@()rng(original));rng(20261001,'twister');
    f=rotatedFixture(0);state=f.state;changes=zeros(250,1);
    for k=1:numel(changes)
        [U,~]=qr(randn(3));values=10.^(-4+8*rand(3,1));values(1:mod(k,4))=0;
        p=f.G*state;information=U*diag(values)*U.';
        m=struct('gradient',information*p,'linearizationPose',p,'information',information);
        before=f.design.T\state;
        delta=localizationSupport.computeLidarMatchedCorrection(m,f.G*state,f.G,f.design,f.cfg, ...
            StepSize=.01+rand,Discretization="implicit");
        state=state+delta;after=f.design.T\state;
        changes(k)=after.'*f.design.P*after-before.'*f.design.P*before;
    end
end
