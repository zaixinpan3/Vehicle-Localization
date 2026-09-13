classdef observerLowPeakingTest < matlab.unittest.TestCase
    % Tests the optional constant GNSS preset and its startup behavior.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function presetVerifiesAtOriginalOperatingBounds(testCase)
            cfg=improvedObserverConfig("gnss","lowPeaking");
            reference=improvedObserverConfig("gnss");
            design=improvedObserverReferenceDesign(cfg);
            verification=verifyImprovedObserverDesign(design,cfg);
            data=buildImprovedObserverCertificateData(cfg);
            testCase.verifyEqual(cfg.operating,reference.operating);
            testCase.verifyTrue(verification.certified);
            testCase.verifyGreaterThan(verification.uniformMargin,.14);
            testCase.verifyEqual(data.T(1:3,1:3)*design.K(1:3,1),[48;512;1536],AbsTol=1e-12);
        end
        function lidarDoesNotSilentlyUseGnssPreset(testCase)
            testCase.verifyError(@() improvedObserverConfig("lidar","lowPeaking"), ...
                'VehicleLocalization:UnsupportedObserverProfile');
        end
        function reducesStartupPeakWithoutLosingSettledAccuracy(testCase)
            reference=trial("reference");
            tuned=trial("lowPeaking");
            testCase.verifyLessThan(tuned.metrics.maximumAccelerationError, ...
                .3*reference.metrics.maximumAccelerationError);
            testCase.verifyLessThan(tuned.metrics.maximumHeadingError, ...
                .5*reference.metrics.maximumHeadingError);
            testCase.verifyLessThan(tuned.metrics.positionRmse,.15);
            testCase.verifyLessThan(tuned.metrics.velocityRmse,.15);
            testCase.verifyLessThan(tuned.metrics.accelerationRmse,.15);
            testCase.verifyLessThan(tuned.metrics.headingRmse,deg2rad(1));
        end
    end
end

function result=trial(profile)
    cfg=improvedObserverConfig("gnss",profile);
    cfg.simulation.finalTime=12;
    cfg.simulation.courseRate=.04;
    cfg.simulation.initialError=[1;.3;.1;-1;-.2;-.1;deg2rad(10)];
    cfg.simulation.positionNoiseAmplitude=.05;
    result=simulateImprovedObserverScenario(improvedObserverReferenceDesign(cfg),struct(),cfg);
end
