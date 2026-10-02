classdef mncavVdbRouteProfileTest < matlab.unittest.TestCase
    methods(TestClassSetup)
        function paths(tc)
            root=fileparts(fileparts(mfilename('fullpath')));
            tc.applyFixture(matlab.unittest.fixtures.PathFixture(root));
            setupVehicleLocalization();
            tc.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts','carla')));
        end
    end
    methods(Test)
        function laneChangeWithoutHeadingCurvatureSlowsVehicle(tc)
            xy=[(0:2:40).',zeros(21,1)];xy(11:end,2)=3.5;
            route=table(xy(:,1),xy(:,2),zeros(21,1), ...
                'VariableNames',{'x_carla','y_carla','curvature_per_m'});
            cfg=mncavVdbConfig();[path,report]=mncavVdbRouteProfile(route,cfg);
            tc.verifyGreaterThan(report.geometricCurvature(10),0);
            tc.verifyLessThan(path(10,4),cfg.simulation.maximumSpeedMps);
            tc.verifyLessThanOrEqual(path(10,4)^2*report.geometricCurvature(10), ...
                cfg.simulation.lateralAccelerationMps2+1e-12);
            tc.verifyLessThanOrEqual(path(1:end-1,4).^2-path(2:end,4).^2, ...
                2*cfg.simulation.maximumBrakeMps2*diff(path(:,3))+1e-12);
        end
        function circleHasKnownCurvature(tc)
            radius=12;angle=(0:.02:2).';
            route=table(radius*cos(angle),radius*sin(angle),zeros(size(angle)), ...
                'VariableNames',{'x_carla','y_carla','curvature_per_m'});
            cfg=mncavVdbConfig();[path,report]=mncavVdbRouteProfile(route,cfg);
            tc.verifyEqual(report.geometricCurvature,ones(size(angle))/radius,AbsTol=1e-11);
            tc.verifyEqual(path(1,4),sqrt(cfg.simulation.lateralAccelerationMps2*radius),AbsTol=1e-10);
            tc.verifyEqual(path(end,4),0);
        end
        function straightRoadKeepsCruiseAndRemovesDuplicates(tc)
            x=[0;0;(2:2:100).'];
            route=table(x,zeros(size(x)),zeros(size(x)), ...
                'VariableNames',{'x_carla','y_carla','curvature_per_m'});
            cfg=mncavVdbConfig();[path,report]=mncavVdbRouteProfile(route,cfg);
            tc.verifyEqual(size(path,1),51);
            tc.verifyEqual(path(1,4),cfg.simulation.maximumSpeedMps);
            tc.verifyEqual(report.maximumGeometricCurvature,0);
            tc.verifyEqual(report.keptSourceRows,[1;(3:52).']);
            tc.verifyEqual(path(end,4),0);
        end
        function suppliedCurvatureStillConstrainsSpeed(tc)
            x=(0:2:100).';
            route=table(x,zeros(size(x)),ones(size(x))*.2, ...
                'VariableNames',{'x_carla','y_carla','curvature_per_m'});
            cfg=mncavVdbConfig();[path,report]=mncavVdbRouteProfile(route,cfg);
            tc.verifyEqual(report.usedCurvature,route.curvature_per_m);
            tc.verifyEqual(path(1,4),sqrt(cfg.simulation.lateralAccelerationMps2/.2),AbsTol=1e-12);
        end
        function immediateReversalIsRejected(tc)
            route=table([0;1;0],zeros(3,1),zeros(3,1), ...
                'VariableNames',{'x_carla','y_carla','curvature_per_m'});
            tc.verifyError(@()mncavVdbRouteProfile(route),'VehicleLocalization:InvalidVdbRoute');
        end
    end
end
