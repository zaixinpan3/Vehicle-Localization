classdef mncavVdbIsolationTest < matlab.unittest.TestCase
    methods(TestClassSetup)
        function paths(tc)
            root=fileparts(fileparts(mfilename('fullpath')));
            tc.applyFixture(matlab.unittest.fixtures.PathFixture(root));
            setupVehicleLocalization();addpath(fullfile(root,'scripts','carla'));
        end
    end
    methods(Test)
        function futureGnssAndReferenceFieldsCannotAffectInitialization(tc)
            [s,i]=fixture();first=initializeMncavVdbFromGnss(s,i,8);
            s.gnssX(s.time>8)=1e8;s.gnssY(s.time>8)=-1e9;
            s.trueHeading=nan(height(s),1);s.trueVelocity=1e12*ones(height(s),1);
            s.refX=-1e12*ones(height(s),1);s.refY=1e12*ones(height(s),1);
            second=initializeMncavVdbFromGnss(s,i,8);
            tc.verifyEqual(first,second);
            tc.verifyEqual(first.pose,[34 52 atan2(4,3)],AbsTol=1e-12);
        end
        function initializationRequiresObservableMeasuredMotion(tc)
            [s,i]=fixture();s.gnssX(:)=0;s.gnssY(:)=0;
            tc.verifyError(@()initializeMncavVdbFromGnss(s,i,8),'VehicleLocalization:InitializationUnavailable');
        end
        function invalidGnssPacketsDoNotInitializeHeading(tc)
            [s,i]=fixture();s.gnssValid(:)=0;
            tc.verifyError(@()initializeMncavVdbFromGnss(s,i,8),'VehicleLocalization:InitializationUnavailable');
        end
        function oracleWheelStateCalibrationIsRejected(tc)
            folder=tempname;mkdir(folder);cleanup=onCleanup(@()rmdir(folder,'s'));
            s=table([2;3],'VariableNames',{'time'});writetable(s,fullfile(folder,'sensors.csv'));
            f=fopen(fullfile(folder,'interface.json'),'w');fprintf(f,'%s',jsonencode(struct('effectiveRadiusM',[.35 .35 .36 .36])));fclose(f);
            tc.verifyError(@()prepareMncavVdbInputs(folder,fullfile(folder,'interface.json')), ...
                'VehicleLocalization:OracleWheelCalibration');
        end
        function pointPackNeverReadsPoseOrLabels(tc)
            folder=tempname;mkdir(folder);mkdir(fullfile(folder,'points'));cleanup=onCleanup(@()rmdir(folder,'s'));
            frames=table(1,8,2,'VariableNames',{'frame_index','lidar_stamp_sec','points'});writetable(frames,fullfile(folder,'frames.csv'));
            xyz=single([1 2 3;4 5 6]);f=fopen(fullfile(folder,'points','000000.bin'),'w','ieee-le');fwrite(f,xyz.','single');fclose(f);
            f=fopen(fullfile(folder,'poses.csv'),'w');fprintf(f,'UNREADABLE REFERENCE POSE');fclose(f);
            file=buildMncavVdbMeasurementClouds(folder);a=load(file);
            tc.verifyEqual(sort(fieldnames(a.pointClouds)),sort({'x';'y';'z';'timestamp'}));
            tc.verifyEqual([a.pointClouds.x,a.pointClouds.y,a.pointClouds.z],xyz);
        end
        function scoringChangesOnlyScoresWhenTruthChanges(tc)
            folder=tempname;mkdir(folder);cleanup=onCleanup(@()rmdir(folder,'s'));
            predictions=table((1:4).',(8:11).',zeros(4,1),zeros(4,1),zeros(4,1), ...
                'VariableNames',{'frame','time','x','y','yaw'});
            path=fullfile(folder,'fused_trajectory.csv');writetable(predictions,path);before=fileread(path);
            reference=table((8:11).',zeros(4,1),zeros(4,1),zeros(4,1),ones(4,1), ...
                zeros(4,1),zeros(4,1),zeros(4,1),'VariableNames', ...
                {'lidar_stamp_sec','pose_x_m','pose_y_m','pose_z_m','pose_qw','pose_qx','pose_qy','pose_qz'});
            ref=fullfile(folder,'reference.csv');writetable(reference,ref);
            a=scoreMncavVdbLocalization(folder,ref,fullfile(folder,'score1'));
            reference.pose_x_m(:)=10;writetable(reference,ref);
            b=scoreMncavVdbLocalization(folder,ref,fullfile(folder,'score2'));
            tc.verifyEqual(a.scores.allRmseM,0);tc.verifyEqual(b.scores.allRmseM,10);
            tc.verifyEqual(fileread(path),before);
            tc.verifyError(@()scoreMncavVdbLocalization(folder,ref,fullfile(folder,'.')), ...
                'VehicleLocalization:SeparateScoringRequired');
        end
    end
end
function [s,i]=fixture()
    t=(0:.1:12).';s=table(t,10+3*t,20+4*t,ones(size(t)), ...
        'VariableNames',{'time','gnssX','gnssY','gnssValid'});
    h=struct('time',t,'longitudinalSpeed',5*ones(size(t)),'longitudinalAcceleration',zeros(size(t)), ...
        'lateralAcceleration',zeros(size(t)),'yawRate',zeros(size(t)));
    i=struct('highRate',h,'lateral',struct('lateralVelocity',zeros(size(t))));
end
