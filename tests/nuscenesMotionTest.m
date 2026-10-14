classdef nuscenesMotionTest < matlab.unittest.TestCase
% nuscenesMotionTest Sensor units, causality and reference exclusion contracts.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));setupVehicleLocalization;
        end
    end
    methods (Test)
        function rearRpmAndSteeringUnitsAreExplicit(testCase)
            f=fixture();[m,a]=prepareNuscenesMotion(f.imu,f.wheels,f.steering,f.cfg);
            testCase.verifyEqual(m.longitudinalSpeed,2*pi*.305*ones(4,1),AbsTol=1e-12);
            testCase.verifyEqual(m.steeringAngle,.1*ones(4,1),AbsTol=1e-12);
            testCase.verifyEqual(a.wheelSpeedsMps(:,1),4*pi*.305*ones(4,1),AbsTol=1e-12);
        end
        function gravityTiltAndArbitraryImuYawAreHandled(testCase)
            f=fixture();phi=.2;theta=-.1;yaw=1.7;
            Rx=[1,0,0;0,cos(phi),-sin(phi);0,sin(phi),cos(phi)];
            Ry=[cos(theta),0,sin(theta);0,1,0;-sin(theta),0,cos(theta)];
            q=[cos(yaw/2)*cos(theta/2)*cos(phi/2)+sin(yaw/2)*sin(theta/2)*sin(phi/2); ...
                cos(yaw/2)*cos(theta/2)*sin(phi/2)-sin(yaw/2)*sin(theta/2)*cos(phi/2); ...
                cos(yaw/2)*sin(theta/2)*cos(phi/2)+sin(yaw/2)*cos(theta/2)*sin(phi/2); ...
                sin(yaw/2)*cos(theta/2)*cos(phi/2)-cos(yaw/2)*sin(theta/2)*sin(phi/2)];
            force=(Ry*Rx)'*[0;0;9.81];f.imu=repmat(struct('utime',0,'q',q,'linear_accel',force,'rotation_rate',zeros(3,1)),4,1);
            [f.imu.utime]=deal(1e6,1.01e6,1.02e6,1.03e6);
            m=prepareNuscenesMotion(f.imu,f.wheels,f.steering,f.cfg);
            testCase.verifyEqual([m.longitudinalAcceleration,m.lateralAcceleration],zeros(4,2),AbsTol=1e-12);
        end
        function accelerationTransportsToRearAxle(testCase)
            f=fixture();[f.imu.rotation_rate]=deal([0;0;.2]);
            [m,a]=prepareNuscenesMotion(f.imu,f.wheels,f.steering,f.cfg);
            testCase.verifyEqual(a.rearAxleAcceleration(:,1),m.longitudinalAcceleration+f.cfg.vehicle.lr*.2^2,AbsTol=1e-12);
            testCase.verifyEqual(a.rearAxleAcceleration(:,2),m.lateralAcceleration,AbsTol=1e-12);
        end
        function navigationPayloadCannotAffectMotion(testCase)
            f=fixture();baseline=prepareNuscenesMotion(f.imu,f.wheels,f.steering,f.cfg);
            [f.imu.pos]=deal([1e8;NaN;Inf]);[f.imu.vel]=deal([-1e8;Inf;NaN]);
            [f.wheels.pose]=deal([1e9;1e9;1e9]);[f.steering.reference]=deal(NaN);
            changed=prepareNuscenesMotion(f.imu,f.wheels,f.steering,f.cfg);
            testCase.verifyEqual(changed,baseline);
        end
        function futureCanPacketsCannotChangeEarlierOutputs(testCase)
            f=fixture();baseline=prepareNuscenesMotion(f.imu,f.wheels,f.steering,f.cfg);
            f.wheels(4).RL_wheel_speed=1e6;f.steering(4).value=100;
            changed=prepareNuscenesMotion(f.imu,f.wheels,f.steering,f.cfg);
            testCase.verifyEqual(changed.longitudinalSpeed(1:3),baseline.longitudinalSpeed(1:3));
            testCase.verifyEqual(changed.steeringAngle(1:3),baseline.steeringAngle(1:3));
        end
        function staleCanIsRejected(testCase)
            f=fixture();f.wheels=f.wheels([1,4]);f.cfg.maximumCanAgeS=.005;
            testCase.verifyError(@()prepareNuscenesMotion(f.imu,f.wheels,f.steering,f.cfg), ...
                'VehicleLocalization:StaleNuscenesCan');
        end
        function zoeOverrideCannotReuseMncavVehicle(testCase)
            cfg=nuscenesTrialConfig(16);nominal=mncavVehicleConfig();
            testCase.verifyNotEqual(cfg.vehicle,nominal.vehicle);
            testCase.verifyEqual(cfg.lateral.outputPoint.forwardOffsetM,cfg.vehicle.lr);
            testCase.verifyFalse(isfield(cfg.global,'invariantDesign'));
        end
        function calibrationAndEvaluationScenesMustBeDisjoint(testCase)
            testCase.verifyError(@()runNuscenesOverlapTrial("missing","unused", ...
                Scenes="scene-0061",CalibrationScenes="scene-0061"), ...
                'VehicleLocalization:NuscenesCalibrationLeakage');
        end
    end
end

function f=fixture()
    imu=repmat(struct('utime',0,'q',[1;0;0;0],'linear_accel',[0;0;9.81], ...
        'rotation_rate',zeros(3,1)),4,1);[imu.utime]=deal(1e6,1.01e6,1.02e6,1.03e6);
    wheels=repmat(struct('utime',0,'FL_wheel_speed',120,'FR_wheel_speed',120, ...
        'RL_wheel_speed',60,'RR_wheel_speed',60),4,1);[wheels.utime]=deal(1e6,1.01e6,1.02e6,1.03e6);
    steering=repmat(struct('utime',0,'value',1.6),4,1);[steering.utime]=deal(1e6,1.01e6,1.02e6,1.03e6);
    f=struct('imu',imu,'wheels',wheels,'steering',steering,'cfg',nuscenesTrialConfig(16));
end
