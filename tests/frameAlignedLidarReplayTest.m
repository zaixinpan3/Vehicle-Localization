classdef frameAlignedLidarReplayTest < matlab.unittest.TestCase
    % Frame timing, source identity and edge coverage of precomputed replay.
    methods (TestClassSetup)
        function addProjectPath(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function nativeFrameKnotsPreserveTurningPoseAndInformation(testCase)
            f=fixture();
            [data,~,meta]=reconstructFrameAlignedLidarSignals(f.high,f.lateral,f.frames, ...
                f.frames,f.pose,f.information,f.cfg);
            testCase.verifyEqual(data.highRate.time(meta.frameIndicesInIntegrationGrid),f.frames,AbsTol=1e-14);
            testCase.verifyEqual(data.lidar.pose(meta.frameIndicesInIntegrationGrid,:),f.pose,AbsTol=1e-13);
            testCase.verifyEqual(data.lidar.information(:,:,meta.frameIndicesInIntegrationGrid),f.information,AbsTol=1e-10);
        end
        function movingTruthHasNoFrameLag(testCase)
            f=fixture();f.pose=[8*f.frames,zeros(numel(f.frames),2)];
            f.cfg.observer.initialState=[0;8;0;0;0;0;0];
            [data,lateral]=reconstructFrameAlignedLidarSignals(f.high,f.lateral,f.frames, ...
                f.frames,f.pose,f.information,f.cfg);
            design=improvedObserverReferenceDesign(f.cfg);
            result=runImprovedVehicleObserver(data,struct(),design,f.cfg,LateralInputs=lateral);
            testCase.verifyEqual(result.position,[8*result.time,zeros(size(result.time))],AbsTol=1e-10);
            testCase.verifyLessThan(max(abs(result.innovations.pose),[],'all'),1e-10);
            testCase.verifyEqual(result.diagnostics.maximumHistoryNodes,0);
        end
        function rejectedFrameIsNotAnOriginalPoseKnot(testCase)
            f=fixture();f.pose=[0,0,0;999,999,0;1,2,.2];
            [data,~,meta]=reconstructFrameAlignedLidarSignals(f.high,f.lateral,f.frames, ...
                f.frames([1,3]),f.pose([1,3],:),f.information(:,:,[1,3]),f.cfg);
            testCase.verifyEqual(data.lidar.pose(meta.frameIndicesInIntegrationGrid(2),:), ...
                f.frames(2)/f.frames(3)*[1,2,.2],AbsTol=1e-12);
            testCase.verifyEqual(meta.acceptedPoseCount,2);
        end
        function finalFractionalFrameUsesDeclaredMotionHold(testCase)
            f=fixture();
            [data,lateral,meta]=reconstructFrameAlignedLidarSignals(f.high,f.lateral,f.frames, ...
                f.frames,f.pose,f.information,f.cfg);
            testCase.verifyEqual(data.highRate.time(end),.205,AbsTol=1e-14);
            testCase.verifyEqual(meta.motionEndHoldSeconds,.005,AbsTol=1e-14);
            testCase.verifyEqual(data.highRate.longitudinalSpeed(end),8,AbsTol=1e-14);
            testCase.verifyEqual(lateral.lateralVelocity(end),0,AbsTol=1e-14);
        end
        function rejectsUndeclaredLongMotionExtension(testCase)
            f=fixture();f.frames(end)=.3;
            testCase.verifyError(@() reconstructFrameAlignedLidarSignals(f.high,f.lateral,f.frames, ...
                f.frames,f.pose,f.information,f.cfg),'VehicleLocalization:MotionCoverage');
        end
        function rejectsNonzeroDelay(testCase)
            f=fixture();f.cfg.measurement.fixedLidarDelay=.15;
            testCase.verifyError(@() reconstructFrameAlignedLidarSignals(f.high,f.lateral,f.frames, ...
                f.frames,f.pose,f.information,f.cfg),'VehicleLocalization:ZeroDelayRequired');
        end
        function respectsExplicitMissingPoseGapLimit(testCase)
            f=fixture();
            testCase.verifyError(@() reconstructFrameAlignedLidarSignals(f.high,f.lateral,f.frames, ...
                f.frames,f.pose,f.information,f.cfg,MaximumOfflineGap=.1),'VehicleLocalization:ReconstructionGap');
        end
    end
end

function f=fixture()
    f.cfg=improvedObserverConfig('lidar');f.cfg.measurement.fixedLidarDelay=0;
    time=(0:.01:.2).';z=zeros(size(time));
    f.high=struct('time',time,'longitudinalSpeed',8+z,'longitudinalAcceleration',z, ...
        'lateralAcceleration',z,'yawRate',z,'steeringAngle',z);
    f.lateral=struct('time',time,'lateralVelocity',z,'sideSlipAngle',z,'sideSlipAngleRate',z);
    f.frames=[0;.073;.205];f.pose=[0,0,.1;.7,.03,.12;1.5,.12,.15];
    f.information=cat(3,1e6*eye(3),[2e6,1e4,2e4;1e4,1e6,3e4;2e4,3e4,3e6],4e6*eye(3));
end
