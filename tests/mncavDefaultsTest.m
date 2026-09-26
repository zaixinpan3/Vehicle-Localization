classdef mncavDefaultsTest < matlab.unittest.TestCase
% mncavDefaultsTest Default runs use the adopted model, not archived snapshots.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function defaultProfileIsCurrentMncav(testCase)
            canonical=mncavVehicleConfig();cfg=lateralObserverConfig();
            testCase.verifyEqual(cfg.vehicle,canonical.vehicle);
            testCase.verifyEqual(cfg.outputPoint,lateralObserverConfig("mncav").outputPoint);
        end
        function wheelGeometryUsesTheSameVehicle(testCase)
            p=mncavVehicleConfig();wheel=wheelSpeedObserverConfig();
            testCase.verifyEqual(wheel.wheelbase,p.vehicle.lf+p.vehicle.lr);
            testCase.verifyEqual(wheel.rearCgDistance,p.vehicle.lr);
            testCase.verifyEqual(wheel.frontTrack,p.stock.frontTrackM);
            testCase.verifyEqual(wheel.rearTrack,p.stock.rearTrackM);
        end
        function replayDefaultsContainCanonicalSteering(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            interface=jsondecode(fileread(fullfile(root,'config','mncavReplayInterface.json')));
            parameters=mncavReplayConfig();canonical=mncavVehicleConfig();
            testCase.verifyEqual(parameters.vehicle,canonical.vehicle);
            testCase.verifyEqual(parameters.steeringRatio,canonical.steeringRatio);
            testCase.verifyEqual(parameters.steeringWheelOffsetRad,interface.steeringWheelOffsetRad);
            testCase.verifyTrue(isfield(parameters,'input_correction'));
        end
        function staleExportCannotOverrideVehicleOrSteering(testCase)
            folder=testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            path=fullfile(folder.Folder,'old.json');old=mncavReplayConfig();
            old.vehicle.mass=999;old.steeringRatio=99;old.steeringWheelOffsetRad=99;
            old.input_correction.yawRate.offset=.123;
            fid=fopen(path,'w');fprintf(fid,'%s',jsonencode(old));fclose(fid);
            actual=mncavReplayConfig(path);current=mncavReplayConfig();
            testCase.verifyEqual(actual.vehicle,current.vehicle);
            testCase.verifyEqual(actual.steeringRatio,current.steeringRatio);
            testCase.verifyEqual(actual.steeringWheelOffsetRad,current.steeringWheelOffsetRad);
            testCase.verifyEqual(actual.input_correction.yawRate.offset,.123);
        end
        function staleDesignIsRejectedBeforeUsingMeasurements(testCase)
            changed=lateralObserverConfig();changed.vehicle.mass=1.1*changed.vehicle.mass;
            design=struct('model',lateralBicycleModel(changed.vehicle));
            testCase.verifyError(@() runLateralVelocityObserver(struct(),design), ...
                'VehicleLocalization:VehicleParameterMismatch');
            testCase.verifyError(@() simulateLateralObserverScenario(design), ...
                'VehicleLocalization:VehicleParameterMismatch');
        end
        function staleSteeringCacheIsRejected(testCase)
            cached=mncavReplayConfig();cached.steeringWheelOffsetRad=0;
            testCase.verifyError(@() assertMncavReplayCurrent(cached), ...
                'VehicleLocalization:StaleReplayConfiguration');
            testCase.verifyWarningFree(@() assertMncavReplayCurrent(mncavReplayConfig()));
        end
        function syntheticTruthUsesTheConfiguredOutputPoint(testCase)
            cfg=lateralObserverConfig();cfg.simulation.tFinal=2;
            design=designLateralObserverGains(cfg);
            cfg.outputPoint.forwardOffsetM=2;
            result=simulateLateralObserverScenario(design,cfg);
            expected=result.truth.state(:,1)-2*result.truth.state(:,2);
            testCase.verifyEqual(result.truth.lateralVelocity,expected,AbsTol=1e-12);
            testCase.verifyEqual(result.truth.lateralVelocityRate, ...
                result.truth.observerPointLateralVelocityRate-2*result.truth.yawRateRate,AbsTol=1e-12);
        end
        function explicitSensitivityConfigurationRemainsSupported(testCase)
            current=lateralObserverConfig();changed=current;
            changed.vehicle.mass=1.1*current.vehicle.mass;
            testCase.verifyWarningFree(@() assertLateralVehicleMatches(struct('model',lateralBicycleModel(changed.vehicle)),changed));
            testCase.verifyWarningFree(@() assertLateralVehicleMatches(struct('model',lateralBicycleModel(current.vehicle)),current));
        end
    end
end
