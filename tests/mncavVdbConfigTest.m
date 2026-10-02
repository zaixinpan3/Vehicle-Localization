classdef mncavVdbConfigTest < matlab.unittest.TestCase
    methods(TestClassSetup)
        function paths(tc)
            root=fileparts(fileparts(mfilename('fullpath')));
            tc.applyFixture(matlab.unittest.fixtures.PathFixture(root));
            setupVehicleLocalization();
            addpath(fullfile(root,'scripts','carla'));
        end
    end
    methods(Test)
        function virtualScanPreservesBodyCoordinates(tc)
            source=struct('rotation',[0 -1 0;1 0 0;0 0 1],'translation',[2 .1 1.4],'identifier',"physical");
            target=struct('rotation',eye(3),'translation',[1.4 0 2],'identifier',"virtual");
            xyz=[1 2 3;-4 5 -2;.2 .3 .4];frame=struct('x',xyz(:,1),'y',xyz(:,2),'z',xyz(:,3));
            transformed=reframeLidarForCalibration(frame,source,target);
            actual=[transformed.x transformed.y transformed.z]*target.rotation.'+target.translation;
            tc.verifyEqual(actual,xyz*source.rotation.'+source.translation,AbsTol=1e-14);
        end
        function preservesCanonicalVehicleAndSensorPriors(tc)
            c=mncavVdbConfig();v=mncavVehicleConfig();s=mncavSensorConfig();
            tc.verifyEqual(c.vehicle,v.vehicle);tc.verifyEqual(c.stock,v.stock);
            tc.verifyEqual(c.steeringRatio,v.steeringRatio);
            tc.verifyEqual(c.sensors.gyroNoiseStdRadps,s.lateralSimulation.yawRateNoiseStdRadps);
            tc.verifyEqual(c.sensors.imuPeriodSeconds,1/s.observerImu.nativeRateHz);
        end
        function physicalAssumptionsAreConsistent(tc)
            c=mncavVdbConfig();t=sscanf(c.stock.standardTireSize,'%f/%fR%f');
            tc.verifyEqual(c.wheel.unloadedRadiusM,t(1)/1000*t(2)/100+t(3)*.0254/2,AbsTol=1e-14);
            tc.verifyGreaterThan(c.vehicle.mass-sum(c.wheel.unsprungMassKg),0);
            tc.verifyGreaterThan(c.suspension.springRateNpm,zeros(1,4));
            tc.verifyGreaterThan(c.suspension.damperRateNspm,zeros(1,4));
            loads=c.vehicle.mass*9.81*[c.vehicle.lr,c.vehicle.lr,c.vehicle.lf,c.vehicle.lf]/(2*(c.vehicle.lf+c.vehicle.lr));
            tc.verifyEqual(sum(loads),c.vehicle.mass*9.81,AbsTol=1e-8);
            tc.verifyGreaterThan(c.body.rollInertiaKgM2+c.body.pitchInertiaKgM2,c.vehicle.yawInertia);
        end
    end
end
