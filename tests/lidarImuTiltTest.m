classdef lidarImuTiltTest < matlab.unittest.TestCase
% lidarImuTiltTest Physical tilt, causal availability and cache isolation.
    methods (TestClassSetup)
        function paths(~)
            addpath(fileparts(fileparts(mfilename('fullpath'))));
            setupVehicleLocalization();
        end
    end
    methods (Test)
        function stationaryGravityIsLeveled(testCase)
            cfg=lidarImuTiltConfig();roll=.12;pitch=-.08;
            up=[-sin(pitch);cos(pitch)*sin(roll);cos(pitch)*cos(roll)];
            sample=struct('time',0,'specificForce',cfg.gravityMps2*up,'angularVelocity',[.01;.02;-.01]);
            [R,~,d]=updateLidarImuTilt(sample,0,[],cfg);
            testCase.verifyEqual(R*up,[0;0;1],AbsTol=1e-12);
            testCase.verifyEqual([d.roll,d.pitch],[roll,pitch],AbsTol=1e-12);
            testCase.verifyEqual(R*R.',eye(3),AbsTol=1e-12);
        end
        function gyroPropagatesWhenGravityIsRejected(testCase)
            sample=struct('time',0,'specificForce',[0;0;9.80665],'angularVelocity',zeros(3,1));
            [~,state]=updateLidarImuTilt(sample,0,[]);
            sample.time=.05;sample.angularVelocity=[.2;0;0];sample.specificForce=[0;0;30];
            [~,~,d]=updateLidarImuTilt(sample,1,state);
            testCase.verifyEqual(d.roll,.01,AbsTol=1e-12);
            testCase.verifyFalse(d.gravityAccepted);
        end
        function steadyTurnAccelerationDoesNotBecomeRoll(testCase)
            sample=struct('time',0,'specificForce',[0;0;9.80665],'angularVelocity',zeros(3,1));
            [~,state]=updateLidarImuTilt(sample,0,[]);
            state.speed=10;state.initialized=true;
            sample.time=.05;sample.angularVelocity=[0;0;.2];sample.specificForce=[0;2;9.80665];
            [R,~,d]=updateLidarImuTilt(sample,10,state);
            testCase.verifyEqual(R,eye(3),AbsTol=1e-12);
            testCase.verifyTrue(d.gravityAccepted);
        end
        function futureImuAndWheelCannotChangePrefix(testCase)
            [imu,wheel]=lidarImuTiltTest.fixture();q=(0:.02:2).';
            a=estimateLidarImuTilt(imu,wheel,q);
            imu.specificForce(imu.arrivalTime>1,:)=30;
            imu.angularVelocity(imu.arrivalTime>1,:)=.5;
            wheel.speed(wheel.arrivalTime>1)=10;
            b=estimateLidarImuTilt(imu,wheel,q);
            testCase.verifyEqual(a.rotation(:,:,q<=1),b.rotation(:,:,q<=1));
        end
        function referenceFieldsCannotAffectTilt(testCase)
            [imu,wheel]=lidarImuTiltTest.fixture();q=(0:.02:2).';
            a=estimateLidarImuTilt(imu,wheel,q);
            imu.referencePose=1e6*ones(numel(imu.deviceTime),6);
            b=estimateLidarImuTilt(imu,wheel,q);
            testCase.verifyEqual(a,b);
            testCase.verifyFalse(b.referencePoseUsed);
        end
        function startupDoesNotUseFutureAlignment(testCase)
            [imu,wheel]=lidarImuTiltTest.fixture();wheel.arrivalTime=wheel.arrivalTime+.02;
            a=estimateLidarImuTilt(imu,wheel,[0;.02;.04]);
            testCase.verifyFalse(a.valid(1));
            testCase.verifyEqual(a.rotation(:,:,1),eye(3));
            testCase.verifyTrue(a.valid(2));
            testCase.verifyFalse(any(a.aligned));
        end
        function missingImuFailsWithoutReferenceFallback(testCase)
            [imu,wheel]=lidarImuTiltTest.fixture();
            testCase.verifyError(@()estimateLidarImuTilt(imu,wheel,[0;3]), ...
                'VehicleLocalization:TiltImuGap');
        end
        function legacyAndReferenceCachesAreRejected(testCase)
            testCase.verifyError(@()assertSensorOnlyLidarTilt(struct()), ...
                'VehicleLocalization:ReferenceTiltCache');
            s=struct('schemaVersion',1,'source',"reference",'referencePoseUsed',true, ...
                'causal',true,'configuration',lidarImuTiltConfig());
            testCase.verifyError(@()assertSensorOnlyLidarTilt(s), ...
                'VehicleLocalization:ReferenceTiltCache');
        end
        function installationRotationIsUnchanged(testCase)
            old=[.931395,.364011,0;-.364011,.931395,0;0,0,1]* ...
                [.925216,-.367871,.093195;.368647,.929524,.009468;-.090110,.025595,.995625];
            current=mississippiLidarMountConfig();
            testCase.verifyEqual(current.rotation,old);
        end
    end
    methods (Static)
        function [imu,wheel]=fixture()
            t=(0:.01:2).';n=numel(t);
            imu=struct('arrivalTime',t,'deviceTime',t,'specificForce',repmat([0,0,9.80665],n,1), ...
                'angularVelocity',zeros(n,3));
            wheel=struct('arrivalTime',t,'speed',zeros(n,1));
        end
    end
end
