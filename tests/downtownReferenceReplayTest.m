classdef downtownReferenceReplayTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function paths(~),setupVehicleLocalization();end
    end
    methods (Test)
        function initializationReadsOnlyTheFirstSelectedReference(test)
            folder=string(tempname);mkdir(folder);cleanup=onCleanup(@()rmdir(folder,'s')); %#ok<NASGU>
            path=fullfile(folder,'mapping_poses.csv');
            f=fopen(path,'w');fprintf(f,'frame_index,lidar_stamp_sec,pose_x_m,pose_y_m,pose_z_m,pose_qw,pose_qx,pose_qy,pose_qz\n2,10,4,-3,0,1,0,0,0\nInvalid future pose must not be read\n');fclose(f);
            packet=readDowntownInitialPose(folder,2,10);
            test.verifyEqual(validateDowntownInitialPose(packet,2,10),[4,-3,0]);
            test.verifyEqual(packet.referencePoseCount,1);
            f=fopen(path,'w');fprintf(f,'frame_index,lidar_stamp_sec,pose_x_m,pose_y_m,pose_z_m,pose_qw,pose_qx,pose_qy,pose_qz\n2,10,4,-3,0,1,0,0,0\nEntirely changed future reference\n');fclose(f);
            test.verifyEqual(readDowntownInitialPose(folder,2,10),packet);
            test.verifyError(@()readDowntownInitialPose(folder,2,10.01),'VehicleLocalization:InitialReferenceClock');
            test.verifyError(@()validateDowntownInitialPose(packet,2,10.01),'VehicleLocalization:InitialPoseClock');
            delete(path);test.verifyEqual(validateDowntownInitialPose(packet,2,10),[4,-3,0]);
            test.verifyError(@()validateDowntownInitialPose(packet,4,10.2),'VehicleLocalization:InitialPoseClock');
            test.verifyError(@()readDowntownInitialPose(folder,2,10),'VehicleLocalization:InitialReferenceMissing');
        end
        function queryScheduleUsesSensorCoverageOnly(test)
            folder=string(tempname);mkdir(folder);cleanup=onCleanup(@()rmdir(folder,'s')); %#ok<NASGU>
            frames=table((1:6).',(10:.1:10.5).',ones(6,1),'VariableNames',{'frame_index','native_time_sec','available'});
            writetable(frames,fullfile(folder,'frames.csv'));
            inputs=struct('nativeOriginSeconds',10.15,'highRate',struct('time',[0;.5]));
            [ids,time]=downtownReplayFrameSchedule(folder,inputs);
            test.verifyEqual(ids,[4;6]);test.verifyEqual(time,[10.3;10.5],'AbsTol',1e-12);
        end
        function motionIgnoresCorruptedReferenceArtifacts(test)
            folder=string(tempname);mkdir(folder);mkdir(fullfile(folder,'sensors'));cleanup=onCleanup(@()rmdir(folder,'s')); %#ok<NASGU>
            time=(100:.02:102).';n=numel(time);p=mncavReplayConfig();wheel=wheelSpeedObserverConfig();
            imu=table(time,repmat(-p.input_correction.longitudinalAcceleration.offset,n,1), ...
                repmat(p.input_correction.lateralAcceleration.offset,n,1), ...
                repmat(-p.input_correction.yawRate.offset,n,1), ...
                'VariableNames',{'native_time_sec','acceleration_x_mps2','acceleration_y_mps2','angular_z_radps'});
            steering=table(time,repmat(p.steeringWheelOffsetRad,n,1), ...
                'VariableNames',{'native_time_sec','steering_wheel_angle_rad'});
            speed=repmat(10./wheel.effectiveRadius,n,1);
            wheels=array2table([time,speed],'VariableNames',{'native_time_sec','front_left','front_right','rear_left','rear_right'});
            writetable(imu,fullfile(folder,'sensors','imu.csv'));writetable(steering,fullfile(folder,'sensors','steering.csv'));writetable(wheels,fullfile(folder,'sensors','wheel_speed_report.csv'));
            design=designLateralObserverGains(lateralObserverConfig());
            f=fopen(fullfile(folder,'mapping_poses.csv'),'w');fprintf(f,'Invalid reference must never be read');fclose(f);
            a=prepareDowntownMotionInputs(folder,design);
            f=fopen(fullfile(folder,'mapping_poses.csv'),'w');fprintf(f,'Entirely different invalid reference');fclose(f);
            b=prepareDowntownMotionInputs(folder,design);
            test.verifyEqual(a.highRate,b.highRate);test.verifyEqual(a.lateral,b.lateral);
            test.verifyFalse(a.referenceUsed);test.verifyFalse(a.gnssUsed);
            test.verifyEqual(a.highRate.longitudinalSpeed,10*ones(size(a.highRate.time)),'AbsTol',1e-8);
        end
        function measuredDeskewUsesTranslationAndRejectsExtrapolation(test)
            frame=struct('x',single([3,3]),'y',single([0,0]),'z',single([0,0]), ...
                'timestamp',10,'pointTimeSeconds',[0,.1]);
            inputs=struct('highRate',struct('time',[0;.1;.2]),'nativeOriginSeconds',10, ...
                'motion',[0,0,0;.2,0,0;.4,0,0]);
            out=deskewDowntownMotionFrame(frame,inputs);
            test.verifyEqual(double(out.x),[3,3.2],'AbsTol',1e-6);
            frame.x=single([0,0]);out=deskewDowntownMotionFrame(frame,inputs);
            test.verifyTrue(all(isnan(out.x)));test.verifyTrue(all(isnan(out.y)));
            frame.pointTimeSeconds=[0,.3];
            test.verifyError(@()deskewDowntownMotionFrame(frame,inputs),'VehicleLocalization:QueryDeskewCoverage');
        end
        function mappingDeskewUsesFullQuaternionMotion(test)
            poses=table([1;2],[10;11],[0;0],[0;0],[0;0], ...
                [1;sqrt(.5)],[0;0],[0;0],[0;sqrt(.5)], ...
                'VariableNames',{'frame_index','lidar_stamp_sec','pose_x_m','pose_y_m','pose_z_m','pose_qw','pose_qx','pose_qy','pose_qz'});
            frame=struct('x',single(1),'y',single(0),'z',single(0), ...
                'timestamp',10,'pointTimeSeconds',.5);
            out=deskewReferenceMappingFrame(frame,poses);
            test.verifyEqual(double([out.x,out.y]),[sqrt(.5),sqrt(.5)],'AbsTol',1e-6);
            frame.x=single(0);out=deskewReferenceMappingFrame(frame,poses);
            test.verifyTrue(isnan(out.x) && isnan(out.y) && isnan(out.z));
        end
        function sparseAcquisitionsRetainOriginalOwnership(test)
            cfg=featureMapBuildConfig();cfg.logEnabled=false;cfg.batchFrameCount=3;cfg.batchFrameStride=2;
            cfg.temporalMap.defaultParams.emMaxIterations=5;
            xyz=[(-.4:.1:.4).',zeros(9,2)];ids=[1,3,5,7];
            data=struct('frameIndices',ids,'featureNames',"curb",'pointsByFeatureFrame',{repmat({xyz},1,4)});
            map=buildSlidingWindowMap(data,cfg);
            test.verifyEqual(map.frameIndices,ids);
            test.verifyEqual(sort([map.batchMaps.ingestedFrameIndices]),ids);
            data.frameIndices=[1,3,3,7];
            test.verifyError(@()buildSlidingWindowMap(data,cfg),'buildSlidingWindowMap:InvalidFrames');
        end
    end
end
