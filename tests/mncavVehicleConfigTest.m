classdef mncavVehicleConfigTest < matlab.unittest.TestCase
    % Source geometry and explicit profile selection must stay consistent.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function stockGeometryPreservesAxleLoadBalance(testCase)
            p=mncavVehicleConfig();v=p.vehicle;
            testCase.verifyEqual(v.mass,2273,AbsTol=1e-12);
            testCase.verifyEqual(v.lf+v.lr,p.stock.wheelbaseM,AbsTol=1e-12);
            testCase.verifyEqual(v.lr/(v.lf+v.lr),p.stock.frontStaticLoadFraction,AbsTol=1e-12);
            testCase.verifyEqual(p.steeringRatio,16.2,AbsTol=1e-12);
            testCase.verifyFalse(p.loadedVehicleParametersIdentified);
        end
        function mncavProfileBuildsTheIntendedBicycleModel(testCase)
            p=mncavVehicleConfig();cfg=lateralObserverConfig("mncav");
            model=lateralBicycleModel(cfg.vehicle);
            testCase.verifyEqual(model.vehicle,p.vehicle);
            testCase.verifyEqual(model.B(1),p.vehicle.frontCorneringStiffness/p.vehicle.mass,AbsTol=1e-12);
            testCase.verifyEqual(model.B(2),p.vehicle.lf*p.vehicle.frontCorneringStiffness/p.vehicle.yawInertia,AbsTol=1e-12);
        end
        function archivedReferenceRemainsExplicitlySeparate(testCase)
            old=lateralObserverConfig("reference");current=lateralObserverConfig("mncav");
            testCase.verifyEqual(old.vehicle.mass,1575,AbsTol=1e-12);
            testCase.verifyNotEqual(current.vehicle,old.vehicle);
            testCase.verifyEqual(current.scheduling,old.scheduling);
        end
    end
end
