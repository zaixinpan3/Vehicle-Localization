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
        function mncavProfileLoadsTheCalibratedOutputPoint(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            stored=jsondecode(fileread(fullfile(root,'config','mncavMotionOutputPoint.json')));
            cfg=lateralObserverConfig("mncav");
            testCase.verifyEqual(cfg.outputPoint.forwardOffsetM,stored.forwardOffsetM);
            testCase.verifyGreaterThan(cfg.outputPoint.forwardOffsetM,1);
            testCase.verifyLessThan(cfg.outputPoint.forwardOffsetM,4);
            testCase.verifyFalse(stored.evaluationDriveUsed);
            testCase.verifyFalse(stored.runtimeReferenceUsed);
        end
        function removedReferenceProfileIsRejected(testCase)
            testCase.verifyError(@() lateralObserverConfig("reference"), ...
                'VehicleLocalization:RemovedVehicleProfile');
            testCase.verifyEqual(lateralObserverConfig().vehicle,lateralObserverConfig("mncav").vehicle);
        end
    end
end
