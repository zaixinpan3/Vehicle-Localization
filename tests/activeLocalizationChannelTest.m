classdef activeLocalizationChannelTest < matlab.unittest.TestCase
% activeLocalizationChannelTest Registration cannot select a current replay.
    methods (TestClassSetup)
        function paths(test)
            root=fileparts(fileparts(mfilename('fullpath')));
            test.applyFixture(matlab.unittest.fixtures.PathFixture(root));
            test.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts','carla')));
            setupVehicleLocalization();
        end
    end
    methods (Test)
        function defaultUsesOverlap(test)
            test.verifyEqual(activeLocalizationChannel(),"overlapGradient");
        end
        function registrationIsArchived(test)
            test.verifyError(@()activeLocalizationChannel("registration"),'VehicleLocalization:ArchivedRegistration');
        end
        function mississippiRejectsRegistrationBeforeOpeningInputs(test)
            test.verifyError(@()runMncavFullObserverExperiment(LidarChannel="registration"), ...
                'VehicleLocalization:ArchivedRegistration');
        end
        function downtownRejectsRegistrationBeforeOpeningInputs(test)
            test.verifyError(@()replayDowntownGnssFreeLocalization("missing","missing","missing",struct(), ...
                LidarChannel="registration"),'VehicleLocalization:ArchivedRegistration');
        end
        function carlaRejectsRegistrationBeforeOpeningInputs(test)
            test.verifyError(@()runMncavVdbLocalization("missing","missing","missing","missing", ...
                LidarChannel="registration"),'VehicleLocalization:ArchivedRegistration');
        end
        function retiredMatcherCannotReachTheObserver(test)
            data=struct('lidarMatcher',@(~,~,~)struct());
            test.verifyError(@()runFullLocalizationObserver(data,struct()), ...
                'VehicleLocalization:ArchivedRegistration');
        end
        function synchronizationPreservesCoveredOverlapSources(test)
            high=struct('time',[0;.1;.2],'longitudinalSpeed',[2;2;2]);
            lateral=struct('time',high.time,'lateralVelocity',[0;0;0],'sideSlipAngle',[0;0;0],'sideSlipAngleRate',[0;0;0]);
            overlap=struct('time',[-.1;0;.1;.2;.3],'sources',{{1;2;3;4;5}},'valid',true(5,1));
            [aligned,lat]=fullObserverSupport.synchronizeLocalizationInputs( ...
                struct('highRate',high,'lidarOverlap',overlap),lateral);
            test.verifyFalse(isfield(aligned,'lidar'));
            test.verifyEqual(aligned.lidarOverlap.sources,{2;3;4});
            test.verifyEqual(aligned.lidarOverlap.time,high.time,'AbsTol',1e-12);
            test.verifyEqual(lat.time,high.time,'AbsTol',1e-12);
        end
    end
end
