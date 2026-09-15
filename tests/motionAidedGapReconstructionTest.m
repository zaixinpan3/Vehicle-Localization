classdef motionAidedGapReconstructionTest < matlab.unittest.TestCase
% Exercise trajectory geometry, endpoint preservation and offline constraints.
    methods(TestClassSetup)
        function addPaths(~)
            setupVehicleLocalization();
        end
    end
    methods(Test)
        function reconstructsAcceleratingTurn(testCase)
            t=(0:.005:3).';yaw=.2*t;vx=2+.6*t;vy=.1*ones(size(t));
            % Closed-form trajectory, independent of the trapezoidal rule.
            r=.2;a=2;b=.6;c=.1;
            x=a*sin(yaw)/r+b*(t.*sin(yaw)/r+(cos(yaw)-1)/r^2)+c*(cos(yaw)-1)/r;
            y=a*(1-cos(yaw))/r+b*(-t.*cos(yaw)/r+sin(yaw)/r^2)+c*sin(yaw)/r;
            truth=[x,y,yaw];
            [data,lateral]=fixture(t,vx,vy,.2*ones(size(t)),truth([1,end],:));
            [actual,meta]=reconstructMotionAidedLidarGaps(data,lateral,t([1,end]));
            testCase.verifyEqual(actual.lidar.pose,truth,AbsTol=1e-5);
            testCase.verifyGreaterThan(max(vecnorm(data.lidar.pose(:,1:2)-truth(:,1:2),2,2)),.5);
            testCase.verifyTrue(meta.futureEndpointUsed);
            testCase.verifyFalse(meta.referenceUsed);
        end
        function preservesNoisyEndpointsAndInformation(testCase)
            t=(0:.01:1).';endpoint=[10,20,0;14,21,.3];
            [data,lateral]=fixture(t,3+0*t,.1+0*t,.2+0*t,endpoint);
            actual=reconstructMotionAidedLidarGaps(data,lateral,t([1,end]));
            testCase.verifyEqual(actual.lidar.pose([1,end],:),endpoint);
            testCase.verifyEqual(actual.lidar.information,data.lidar.information);
        end
        function leavesFrequentMeasurementsAndTheirIntervalsUnchanged(testCase)
            t=(0:.01:1).';[data,lateral]=fixture(t,3+0*t,0*t,.2+0*t,[0,0,0;3,0,.2]);
            [actual,meta]=reconstructMotionAidedLidarGaps(data,lateral,t(1:10:end));
            testCase.verifyEqual(actual.lidar.pose,data.lidar.pose);
            testCase.verifyEqual(meta.correctedIntervals,0);
        end
        function preservesUnwrappedHeadingAcrossPi(testCase)
            t=(0:.01:1).';yaw=deg2rad(179+2*t);
            [data,lateral]=fixture(t,ones(size(t)),0*t,deg2rad(2)+0*t,[0,0,yaw(1);-1,0,yaw(end)]);
            actual=reconstructMotionAidedLidarGaps(data,lateral,t([1,end]));
            testCase.verifyEqual(actual.lidar.pose(:,3),yaw,AbsTol=1e-12);
        end
        function rejectsDelayAndIncompleteAnchors(testCase)
            t=(0:.01:1).';[data,lateral]=fixture(t,3+0*t,0*t,0*t,[0,0,0;3,0,0]);
            testCase.verifyError(@()reconstructMotionAidedLidarGaps(data,lateral,t([2,end])), ...
                'VehicleLocalization:InvalidMotionGapAnchors');
            data.lidar.delay=.1;
            testCase.verifyError(@()reconstructMotionAidedLidarGaps(data,lateral,t([1,end])), ...
                'VehicleLocalization:InvalidMotionGapInput');
        end
    end
end
function [data,lateral]=fixture(t,vx,vy,gyro,endpoints)
    data.highRate=struct('time',t,'longitudinalSpeed',vx,'yawRate',gyro);
    data.lidar=struct('time',t,'pose',interp1(t([1,end]),endpoints,t),'delay',0, ...
        'representation',"piecewiseLinear",'headingConvention',"unwrapped", ...
        'information',repmat(eye(3),1,1,numel(t)));
    lateral=struct('time',t,'lateralVelocity',vy);
end
