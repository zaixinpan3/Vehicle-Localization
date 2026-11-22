classdef localizationLidarInputTest < matlab.unittest.TestCase
% localizationLidarInputTest Raw feature clouds on the frame clock supply the LiDAR channel.
    methods (TestClassSetup)
        function paths(test)
            root=fileparts(fileparts(mfilename('fullpath')));
            test.applyFixture(matlab.unittest.fixtures.PathFixture(root));
            test.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts','carla')));
            setupVehicleLocalization();
        end
    end
    methods (Test)
        function productionRejectsExternalPoseAndResidualProviders(test)
            payloads={struct('pose',[1 2 .1]), struct('evaluateResidual',@(~,~)struct())};
            for k=1:numel(payloads)
                data=struct('lidar',payloads{k});
                test.verifyError(@()runFullLocalizationObserver(data),'VehicleLocalization:ExternalLidarPose');
                test.verifyError(@()localizationSupport.synchronizeLocalizationInputs(data,struct()), ...
                    'VehicleLocalization:ExternalLidarPose');
                data=struct('lidarOverlap',payloads{k});
                test.verifyError(@()runFullLocalizationObserver(data),'VehicleLocalization:ExternalLidarPose');
            end
        end
        function synchronizationPreservesCoveredOverlapSources(test)
            high=struct('time',[0;.1;.2],'longitudinalSpeed',[2;2;2]);
            lateral=struct('time',high.time,'lateralVelocity',[0;0;0],'yawRate',[0;0;0]);
            overlap=struct('time',[-.1;0;.1;.2;.3],'sources',{{1;2;3;4;5}},'valid',true(5,1));
            [aligned,lat]=localizationSupport.synchronizeLocalizationInputs( ...
                struct('highRate',high,'lidarOverlap',overlap),lateral);
            test.verifyFalse(isfield(aligned,'lidar'));
            test.verifyEqual(aligned.lidarOverlap.sources,{2;3;4});
            test.verifyEqual(aligned.lidarOverlap.time,high.time,'AbsTol',1e-12);
            test.verifyEqual(lat.time,high.time,'AbsTol',1e-12);
        end
    end
end
