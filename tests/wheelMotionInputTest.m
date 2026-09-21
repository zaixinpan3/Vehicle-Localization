classdef wheelMotionInputTest < matlab.unittest.TestCase
% wheelMotionInputTest Recorded input preparation has no alternate Vx source.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));setupVehicleLocalization;
        end
    end
    methods (Test)
        function fourWheelsWorkWithoutAnotherSpeedFile(testCase)
            f=fixture(testCase);[h,w,m]=prepare(f);
            testCase.verifyEqual(h.longitudinalSpeed,10*ones(size(h.time)),AbsTol=1e-12);
            testCase.verifyTrue(all(w.valid));
            testCase.verifyFalse(m.alternativeSpeedFallback);
            testCase.verifyFalse(isfile(fullfile(f.folder,'twist.csv')));
        end
        function UnusedLegacyFilesAndSpeedFieldsCannotAffectVx(testCase)
            f=fixture(testCase);a=prepare(f);
            fid=fopen(fullfile(f.folder,'twist.csv'),'w');fprintf(fid,'invalid obsolete speed data');fclose(fid);
            steering=readtable(fullfile(f.folder,'steering.csv'));steering.speed_mps(:)=9000;
            writetable(steering,fullfile(f.folder,'steering.csv'));b=prepare(f);
            testCase.verifyEqual(a.longitudinalSpeed,b.longitudinalSpeed,AbsTol=0);
        end
        function missingWheelFileNeverFallsBack(testCase)
            f=fixture(testCase);delete(fullfile(f.folder,'wheel_speed_report.csv'));
            writetable(table([0;2],[10;10],VariableNames={'stamp_sec','linear_x_mps'}),fullfile(f.folder,'twist.csv'));
            testCase.verifyError(@()prepare(f),'VehicleLocalization:MissingWheelSpeed');
        end
        function expiredWheelPacketsDoNotShortenTheExperiment(testCase)
            f=fixture(testCase);w=readtable(fullfile(f.folder,'wheel_speed_report.csv'));
            writetable(w(1:6,:),fullfile(f.folder,'wheel_speed_report.csv'));
            testCase.verifyError(@()prepare(f),'VehicleLocalization:WheelSpeedCoverage');
        end
        function missingWheelColumnIsRejected(testCase)
            f=fixture(testCase);w=readtable(fullfile(f.folder,'wheel_speed_report.csv'));w.rear_right=[];
            writetable(w,fullfile(f.folder,'wheel_speed_report.csv'));
            testCase.verifyError(@()prepare(f),'VehicleLocalization:InvalidWheelInput');
        end
        function wheelChangesReachTheMotionInput(testCase)
            f=fixture(testCase);w=readtable(fullfile(f.folder,'wheel_speed_report.csv'));w{:,2:5}=w{:,2:5}*.8;
            writetable(w,fullfile(f.folder,'wheel_speed_report.csv'));h=prepare(f);
            testCase.verifyEqual(h.longitudinalSpeed,8*ones(size(h.time)),AbsTol=1e-12);
        end
        function declaredStartupDoesNotInventValidWheels(testCase)
            f=fixture(testCase);w=readtable(fullfile(f.folder,'wheel_speed_report.csv'));w=w(3:end,:);
            writetable(w,fullfile(f.folder,'wheel_speed_report.csv'));
            [h,e,m]=prepareWheelMotionInputs(f.folder,f.parameters,f.clock,0,2,InitialSpeed=0);
            testCase.verifyFalse(any(e.valid(h.time<.04)));
            testCase.verifyEqual(m.initialPredictionOnlySamples,4);
        end
        function prolongedStartupCannotUseAnInitialStateForever(testCase)
            f=fixture(testCase);w=readtable(fullfile(f.folder,'wheel_speed_report.csv'));w=w(21:end,:);
            writetable(w,fullfile(f.folder,'wheel_speed_report.csv'));
            testCase.verifyError(@()prepareWheelMotionInputs(f.folder,f.parameters,f.clock,0,2,InitialSpeed=0), ...
                'VehicleLocalization:WheelSpeedCoverage');
        end
    end
end

function f=fixture(testCase)
    folder=string(tempname);mkdir(folder);testCase.addTeardown(@()rmdir(folder,'s'));
    t=(0:.01:2).';wt=(0:.02:2).';c=wheelSpeedObserverConfig();
    wh=array2table([wt,repmat(10./c.effectiveRadius,numel(wt),1)], ...
        VariableNames={'stamp_sec','front_left','front_right','rear_left','rear_right'});
    imu=table(t,zeros(size(t)),zeros(size(t)),zeros(size(t)), ...
        VariableNames={'stamp_sec','acceleration_x_mps2','acceleration_y_mps2','angular_z_radps'});
    steering=table(t,zeros(size(t)),10*ones(size(t)), ...
        VariableNames={'stamp_sec','steering_wheel_angle_rad','speed_mps'});
    writetable(wh,fullfile(folder,'wheel_speed_report.csv'));writetable(imu,fullfile(folder,'imu.csv'));
    writetable(steering,fullfile(folder,'steering.csv'));correction=struct('sign',1,'offset',0);
    parameters=struct('steeringRatio',16.2,'steeringWheelOffsetRad',0,'input_correction', ...
        struct('longitudinalAcceleration',correction,'lateralAcceleration',correction,'yawRate',correction));
    f=struct('folder',folder,'parameters',parameters,'clock',struct('schemaVersion',1,'method',"robust_affine_receiver_clock", ...
        'sourceOriginSeconds',0,'scale',1,'offsetSeconds',0,'sourceSpanSeconds',2, ...
        'maximumExtrapolationSeconds',.05,'modelId',"synthetic_identity"));
end

function [h,w,m]=prepare(f)
    [h,w,m]=prepareWheelMotionInputs(f.folder,f.parameters,f.clock,0,2);
end
