classdef receiverClockTest < matlab.unittest.TestCase
% receiverClockTest Verify the shared Python artifact and fail-closed timing.
    methods (TestClassSetup)
        function paths(~)
            setupVehicleLocalization();
        end
    end
    methods (Test)
        function pythonAndMatlabAgreeAtLargeEpoch(testCase)
            folder=fixtureFolder();clock=loadReceiverClock(fullfile(folder,'native.csv'));
            query=jsondecode(fileread(fullfile(folder,'queries.json')));
            actual=receiverClockTime(clock,query.stamps);
            testCase.verifyEqual(actual,query.receiverSeconds,AbsTol=1e-12);
        end
        function outsideSegmentIsRejected(testCase)
            clock=loadReceiverClock(fullfile(fixtureFolder(),'native.csv'));
            testCase.verifyError(@()receiverClockTime(clock,clock.sourceOriginSeconds-1), ...
                'VehicleLocalization:ClockCoverage');
        end
        function nonfiniteStampIsRejected(testCase)
            clock=loadReceiverClock(fullfile(fixtureFolder(),'native.csv'));
            testCase.verifyError(@()receiverClockTime(clock,NaN),'VehicleLocalization:ClockCoverage');
        end
        function invalidScaleIsRejected(testCase)
            clock=loadReceiverClock(fullfile(fixtureFolder(),'native.csv'));clock.scale=-1;
            testCase.verifyError(@()receiverClockTime(clock,clock.sourceOriginSeconds), ...
                'VehicleLocalization:InvalidReceiverClock');
        end
        function changedSourceCannotReuseModel(testCase)
            folder=string(tempname);mkdir(folder);testCase.addTeardown(@()rmdir(folder,'s'));
            copyfile(fullfile(fixtureFolder(),'native.clock.json'),folder);
            fid=fopen(fullfile(folder,'native.csv'),'w');fprintf(fid,'changed source');fclose(fid);
            testCase.verifyError(@()loadReceiverClock(fullfile(folder,'native.csv')), ...
                'VehicleLocalization:StaleReceiverClock');
        end
        function modifiedCoefficientsCannotKeepTheOldIdentity(testCase)
            folder=string(tempname);mkdir(folder);testCase.addTeardown(@()rmdir(folder,'s'));
            copyfile(fullfile(fixtureFolder(),'native.csv'),folder);
            model=loadReceiverClock(fullfile(fixtureFolder(),'native.csv'));model.offsetSeconds=1;
            fid=fopen(fullfile(folder,'native.clock.json'),'w');fprintf(fid,'%s',jsonencode(model));fclose(fid);
            testCase.verifyError(@()loadReceiverClock(fullfile(folder,'native.csv')), ...
                'VehicleLocalization:InvalidReceiverClock');
        end
        function missingModelHasNoHeaderInterpolationFallback(testCase)
            testCase.verifyError(@()loadReceiverClock(string(tempname)+".csv"), ...
                'VehicleLocalization:MissingReceiverClock');
        end
    end
end

function folder=fixtureFolder()
    folder=fullfile(fileparts(mfilename('fullpath')),'fixtures','receiver_clock');
end
