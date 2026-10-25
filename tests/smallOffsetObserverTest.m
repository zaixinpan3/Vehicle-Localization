classdef smallOffsetObserverTest < matlab.unittest.TestCase
% Fresh common design and sampled behavior for the temporary assumed geometry.
    properties (Access=private)
        Config
    end
    properties (TestParameter)
        speed={1,3,10,30}
        sourceMode={"gnss","lidar","alternating"}
    end
    methods (TestClassSetup)
        function solveCurrentDesign(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));
            setupVehicleLocalization;
            [~,testCase.Config]=synthesizeMncavInvariantDesign(mncavSmallOffsetObserverConfig());
        end
    end
    methods (Test)
        function assumedGeometryDoesNotReplaceCalibrationOrReduceNoise(testCase)
            original=mncavFullObserverConfig();c=mncavSmallOffsetObserverConfig();
            testCase.verifyTrue(c.gnss.outputPoint.assumedGeometry);
            testCase.verifyEqual(norm(c.gnss.outputPoint.bodyOffset),.01,AbsTol=1e-14);
            testCase.verifyGreaterThan(norm(original.gnss.outputPoint.bodyOffset),1);
            testCase.verifyEqual(c.gnss.outputPoint.bodyCovariance,original.gnss.outputPoint.bodyCovariance);
            testCase.verifyEqual(c.invariantObserver.noise,original.invariantObserver.noise);
            testCase.verifyEqual(c.invariantObserver.redundancy.degradedLidarMismatch, ...
                original.invariantObserver.redundancy.degradedLidarMismatch);
            testCase.verifyFalse(isfield(c,'invariantDesign'));
        end
        function commonCertificateCoversTheDeclaredMovingDomain(testCase)
            c=testCase.Config;v=c.invariantDesign.verification;
            testCase.verifyEqual([c.invariantObserver.redundancy.minimumGnssSpeed,c.iss.maximumSpeed],[1,30]);
            testCase.verifyTrue(v.certified);
            testCase.verifyTrue(v.feedbackAuthority.verified);
            testCase.verifyTrue(v.regionalIss.certified);
            testCase.verifyEqual(v.jointSensor.vertices,13440);
            testCase.verifyLessThan(max([v.jointSensor.worstLidarEigenvalue, ...
                v.jointSensor.worstGnssEigenvalue,v.jointSensor.worstMixedEigenvalue]),-.009);
        end
        function measuredOffsetCannotReuseThisTemporaryDesign(testCase)
            c=testCase.Config;original=mncavFullObserverConfig();
            c.gnss.outputPoint=original.gnss.outputPoint;
            testCase.verifyError(@()invariantObserverSupport.runtime(c),'VehicleLocalization:StaleInvariantDesign');
        end
        function enlargedDomainCannotReuseThisTemporaryDesign(testCase)
            c=testCase.Config;c.iss.maximumSpeed=31;
            testCase.verifyError(@()invariantObserverSupport.runtime(c),'VehicleLocalization:StaleInvariantDesign');
        end
        function movingPoseErrorDecreasesAtOneHundredMilliseconds(testCase,speed,sourceMode)
            [estimate,initialError,finalError]=runScenario(testCase.Config,speed,sourceMode);
            testCase.verifyTrue(all(isfinite(estimate.z),'all'));
            testCase.verifyLessThan(finalError,initialError);
            testCase.verifyEqual(estimate.diagnostics.stateResets,0);
            testCase.verifyFalse(estimate.diagnostics.allTheoremHypothesesVerified);
        end
    end
end

function [estimate,initialError,finalError]=runScenario(cfg,speed,mode)
% Exact synthetic sensor outputs at the assumed point; this is not a replay
% or a test of a real overlap map. Nonzero errors test sampled recovery,
% without claiming they satisfy the conservative continuous ISS sublevel.
    time=(0:.1:40).';n=numel(time);zero=zeros(n,1);
    truth=[speed*time,zero,zero];
    high=struct('time',time,'longitudinalSpeed',speed*ones(n,1), ...
        'longitudinalAcceleration',zero,'lateralAcceleration',zero,'yawRate',zero);
    lateral=struct('time',time,'lateralVelocity',zero,'sideSlipAngle',zero,'sideSlipAngleRate',zero);
    gnss=struct('time',time,'position',truth(:,1:2)+cfg.gnss.outputPoint.bodyOffset(:).', ...
        'information',repmat(1e6*eye(2),1,1,n),'valid',true(n,1),'delay',0);
    lidar=struct('time',time,'pose',truth,'information',repmat(1e6*eye(3),1,1,n), ...
        'valid',true(n,1),'delay',0);
    switch mode
        case "gnss",lidar.valid(:)=false;
        case "lidar",gnss.valid(:)=false;
        case "alternating"
            gnss.valid=mod((0:n-1).',2)==0;lidar.valid=~gnss.valid;
    end
    cfg.initialState=[.02;speed;0;-.015;0;0;.01;0];
    data=struct('highRate',high,'gnss',gnss,'lidar',lidar);
    estimate=runFullLocalizationObserver(data,struct(),cfg,LateralInputs=lateral);
    % Use the same physical pose norm for every speed and availability case.
    initialError=norm([.02,-.015,.01]);
    finalError=norm([estimate.position(end,:)-truth(end,1:2),estimate.heading(end)]);
    fprintf('SMALL OFFSET REPLAY speed=%g mode=%s dt=0.1 finalPosition=%.9g finalHeading=%.9g ratio=%.9g\n', ...
        speed,mode,norm(estimate.position(end,:)-truth(end,1:2)),abs(estimate.heading(end)),finalError/initialError);
end
