classdef motionAidedObserverTest < matlab.unittest.TestCase
% motionAidedObserverTest Accuracy, signal contracts and dissipation checks.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));setupVehicleLocalization;
        end
    end
    methods (Test)
        function constantVelocityIsPreserved(testCase)
            f=fixture(2);result=runMotionAidedVehicleObserver(f.data,f.lateral,f.cfg);
            testCase.verifyEqual(result.pose,f.truth,AbsTol=1e-10);
            testCase.verifyEqual(result.velocity,repmat([8,0],numel(result.time),1),AbsTol=1e-10);
        end
        function constantAccelerationIsPreserved(testCase)
            f=fixture(2);t=f.data.highRate.time;f.data.highRate.longitudinalSpeed=8+.5*t;
            f.data.highRate.longitudinalAcceleration(:)=.5;f.data.lidar.pose(:,1)=8*t+.25*t.^2;
            result=runMotionAidedVehicleObserver(f.data,f.lateral,f.cfg);
            % Linear source reconstruction has error <= max|p''|*dt^2/8.
            testCase.verifyEqual(result.position(:,1),8*t+.25*t.^2,AbsTol=.5*.01^2/8+1e-9);
            testCase.verifyEqual(result.acceleration(:,1),.5*ones(size(t)),AbsTol=1e-10);
        end
        function constantTurnIsPreserved(testCase)
            f=turnFixture;result=runMotionAidedVehicleObserver(f.data,f.lateral,f.cfg);
            % The circular position source has curvature bound v*r=1.6.
            testCase.verifyEqual(result.pose,f.truth,AbsTol=1.6*.01^2/8+1e-9);
            testCase.verifyLessThan(max(abs(result.acceleration),[],'all'),2);
        end
        function headingLiftCrossesPiContinuously(testCase)
            f=fixture(3);t=f.data.highRate.time;f.data.highRate.longitudinalSpeed(:)=0;
            f.data.highRate.yawRate(:)=.2;f.data.lidar.pose=[zeros(numel(t),2),3+.2*t];
            result=runMotionAidedVehicleObserver(f.data,f.lateral,f.cfg);
            testCase.verifyEqual(result.headingUnwrapped,3+.2*t,AbsTol=1e-10);
            testCase.verifyLessThan(result.heading(end),-2.5);
        end
        function lidarPositionNoiseCannotKickMotionStates(testCase)
            f=fixture(25);t=f.data.highRate.time;f.data.highRate.longitudinalSpeed(:)=0;
            f.data.lidar.pose(:,1)=.01*sin(2*pi*.3587165*t);
            result=runMotionAidedVehicleObserver(f.data,f.lateral,f.cfg);
            testCase.verifyEqual(result.velocity,zeros(numel(t),2),AbsTol=1e-12);
            testCase.verifyEqual(result.acceleration,zeros(numel(t),2),AbsTol=1e-12);
            testCase.verifyLessThan(max(abs(result.position(t>15,1))),.009);
        end
        function bodyMotionMeasurementsAffectPosition(testCase)
            f=fixture(2);f.data.highRate.longitudinalSpeed(:)=0;f.data.lidar.pose(:)=0;
            base=runMotionAidedVehicleObserver(f.data,f.lateral,f.cfg);
            f.data.highRate.longitudinalSpeed(:)=1;
            changed=runMotionAidedVehicleObserver(f.data,f.lateral,f.cfg);
            testCase.verifyGreaterThan(changed.position(end,1)-base.position(end,1),.2);
        end
        function offsetsDecayWithoutStateResets(testCase)
            f=fixture(8);f.cfg.initialState=[1;9;.2;-1;.3;-.2;0];
            result=runMotionAidedVehicleObserver(f.data,f.lateral,f.cfg);
            testCase.verifyLessThan(norm(result.pose(end,:)-f.truth(end,:)),1e-8);
            testCase.verifyEqual(result.diagnostics.stateResets,0);
        end
        function certificateCoversRotatingAnisotropicWeights(testCase)
            cfg=motionAidedObserverConfig;design=designMotionAidedObserverGains(cfg);
            margin=independentDissipationMinimum(cfg);
            testCase.verifyGreaterThanOrEqual(margin,design.translationDissipationMargin-1e-10);
        end
        function weakGainsFailTheCertificate(testCase)
            cfg=motionAidedObserverConfig;cfg.gains=[.01,.01,.01,1];
            testCase.verifyError(@() designMotionAidedObserverGains(cfg),'VehicleLocalization:InfeasibleMotionCertificate');
        end
        function nonzeroDelayIsRejected(testCase)
            f=fixture(1);f.data.lidar.delay=.1;
            testCase.verifyError(@() runMotionAidedVehicleObserver(f.data,f.lateral,f.cfg),'VehicleLocalization:ZeroDelayRequired');
        end
        function missingHeadingLiftIsRejected(testCase)
            f=fixture(1);f.data.lidar=rmfield(f.data.lidar,'headingConvention');
            testCase.verifyError(@() runMotionAidedVehicleObserver(f.data,f.lateral,f.cfg),'VehicleLocalization:InvalidContinuousSignal');
        end
        function misalignedLateralTimeIsRejected(testCase)
            f=fixture(1);f.lateral.time=f.lateral.time+.001;
            testCase.verifyError(@() runMotionAidedVehicleObserver(f.data,f.lateral,f.cfg),'VehicleLocalization:InvalidLateralInputs');
        end
        function degenerateInformationIsRejected(testCase)
            f=fixture(1);f.data.lidar.information(2,2,:)=0;
            testCase.verifyError(@() runMotionAidedVehicleObserver(f.data,f.lateral,f.cfg),'VehicleLocalization:InsufficientLidarInformation');
        end
        function outsideRateBoundIsReportedWithoutClamping(testCase)
            f=fixture(1);f.data.highRate.yawRate(:)=.6;
            result=runMotionAidedVehicleObserver(f.data,f.lateral,f.cfg);
            testCase.verifyFalse(result.diagnostics.rateEnvelopeSatisfied);
            testCase.verifyEqual(result.trackAngleRate,.6*ones(size(result.time)),AbsTol=1e-12);
            testCase.verifyFalse(result.diagnostics.unconditionalStabilityClaimed);
        end
    end
end

function f=fixture(duration)
    t=(0:.01:duration).';n=numel(t);zero=zeros(n,1);
    high=struct('time',t,'longitudinalSpeed',8+zero,'longitudinalAcceleration',zero, ...
        'lateralAcceleration',zero,'yawRate',zero);
    pose=[8*t,zero,zero];source=struct('time',t,'pose',pose,'information',repmat(1e6*eye(3),1,1,n), ...
        'delay',0,'representation',"piecewiseLinear",'headingConvention',"unwrapped");
    lateral=struct('time',t,'lateralVelocity',zero,'sideSlipAngleRate',zero);
    f=struct('data',struct('highRate',high,'lidar',source),'lateral',lateral,'truth',pose,'cfg',motionAidedObserverConfig);
end

function f=turnFixture
    f=fixture(3);t=f.data.highRate.time;r=.2;v=8;
    f.data.highRate.yawRate(:)=r;f.data.highRate.lateralAcceleration(:)=v*r;
    f.truth=[v/r*sin(r*t),v/r*(1-cos(r*t)),r*t];f.data.lidar.pose=f.truth;
end

function smallest=independentDissipationMinimum(cfg)
    g=cfg.gains;J=[0,-1;1,0];smallest=Inf;
    for q=linspace(-cfg.maximumTrackAngleRate,cfg.maximumTrackAngleRate,41)
        for angle=linspace(0,pi,11)
            R=[cos(angle),-sin(angle);sin(angle),cos(angle)];W=R*diag([cfg.lidar.minimumPoseWeight,1])*R.';
            A=[-g(1)*W,eye(2),zeros(2);zeros(2),-g(2)*eye(2),eye(2); ...
                zeros(2),q^2*eye(2),-g(3)*eye(2)+2*q*J];
            smallest=min(smallest,min(eig(-(A+A.'))));
        end
    end
end
