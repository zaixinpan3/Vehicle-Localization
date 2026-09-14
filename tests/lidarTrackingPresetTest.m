classdef lidarTrackingPresetTest < matlab.unittest.TestCase
    % Verify gain-only selection and certificate integrity for delayed LiDAR.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function presetKeepsTheSameDelayAndOperatingBounds(testCase)
            reference=improvedObserverConfig("lidar");cfg=improvedObserverConfig("lidar","tracking");
            design=improvedObserverReferenceDesign(cfg);original=improvedObserverReferenceDesign(reference);
            check=verifyImprovedObserverDesign(design,cfg);
            testCase.verifyEqual(cfg.operating,reference.operating);
            testCase.verifyEqual(cfg.measurement,reference.measurement);
            testCase.verifyEqual(cfg.lidar,reference.lidar);
            testCase.verifyEqual(design.N,original.N,AbsTol=1e-15);
            testCase.verifyEqual(design.K([3,6],[1,2]),1.5*original.K([3,6],[1,2]),AbsTol=1e-15);
            testCase.verifyGreaterThan(check.uniformMargin,.14);
            testCase.verifyTrue(check.certified);
        end
        function gnssCannotSilentlySelectLidarProfile(testCase)
            testCase.verifyError(@() improvedObserverConfig("gnss","tracking"), ...
                'VehicleLocalization:UnsupportedObserverProfile');
        end
        function changedDelayMustStillPassVerification(testCase)
            cfg=improvedObserverConfig("lidar","tracking");design=improvedObserverReferenceDesign(cfg);
            cfg.measurement.fixedLidarDelay=1;
            check=verifyImprovedObserverDesign(design,cfg);
            testCase.verifyFalse(check.certified);
        end
        function gainChangesCannotReuseASuccessFlag(testCase)
            cfg=improvedObserverConfig("lidar","tracking");design=improvedObserverReferenceDesign(cfg);
            design.K=10*design.K;
            check=verifyImprovedObserverDesign(design,cfg);
            testCase.verifyFalse(check.certified);
        end
    end
end
