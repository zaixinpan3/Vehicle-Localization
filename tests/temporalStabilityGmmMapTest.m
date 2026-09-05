classdef temporalStabilityGmmMapTest < matlab.unittest.TestCase
% temporalStabilityGmmMapTest: Unit tests of the semantic temporal-stability
% Gaussian support map on a compact three-frame synthetic fixture.

    methods (TestClassSetup)
        function addProjectPaths(testCase)
        % addProjectPaths: Put the vehicleLocalization modules on the path.
        %
        % Input:
        %   testCase: matlab.unittest.TestCase instance
        %
        % Output:
        %   none
            projectFolder = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(projectFolder, "config")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(projectFolder, "mapping")));
        end
    end

    methods (Test)
        function standardGmmTrainingContractIsExplicit(testCase)
        % standardGmmTrainingContractIsExplicit: Temporal reliability is used
        % only for resampling and post-hoc support; the EM stage itself is a
        % timestamp-free standard Gaussian mixture fit.
            [cfg, points, labels, timestamps] = testCase.syntheticStableFixture();
            cfg.defaultParams.storeEmAssignments = true;
            gmmMap = buildTemporalStabilityGmmMap(points, labels, timestamps, cfg);
            layer = gmmMap.layers(1);
            component = layer.components(1);

            testCase.verifyEqual(layer.gmmTrainingModel, "standardGaussianMixtureEmOnTemporalReliabilityResample");
            testCase.verifyFalse(layer.gmmTrainingUsesTimestamps);
            testCase.verifyFalse(layer.gmmTrainingUsesTemporalWeights);
            testCase.verifyFalse(layer.gmmTrainingUsesStudentTWeights);
            testCase.verifyFalse(layer.gmmTrainingUsesUniformBackground);
            testCase.verifyEqual(layer.emRefinementModel, layer.gmmTrainingModel);
            testCase.verifyEqual(layer.emResponsibilityRowSumSemantics, "ordinaryGmmForegroundOnly");
            testCase.verifyEqual(layer.emResponsibilityRowSums, ones(numel(layer.resampledSourceIndices), 1), AbsTol=1.0e-10);
            testCase.verifyTrue(layer.posthocTemporalSupportEstimated);
            testCase.verifyEqual(layer.integratedSupportAssignmentModel, "ordinaryGmmResponsibilitiesTimesPrecomputedTemporalReliability");
            testCase.verifyEqual(component.posthocSupportAssignmentModel, layer.integratedSupportAssignmentModel);
            testCase.verifyGreaterThan(component.posthocSupportEvidenceMass, 0);
            testCase.verifyEqual(component.mixtureWeight, layer.componentMixtureWeights(1), AbsTol=1.0e-12);
        end

        function reliabilitySamplingDiagnosticsAreStored(testCase)
        % reliabilitySamplingDiagnosticsAreStored: Point reliability scores,
        % sampling probabilities, Bernoulli counts, and sampled source indices
        % are retained on the semantic layer.
            [cfg, points, labels, timestamps] = testCase.syntheticStableFixture();
            gmmMap = buildTemporalStabilityGmmMap(points, labels, timestamps, cfg);
            layer = gmmMap.layers(1);

            testCase.verifySize(layer.temporalReliabilityScores, [size(points, 1), 1]);
            testCase.verifySize(layer.samplingProbabilities, [size(points, 1), 1]);
            testCase.verifySize(layer.resampleCounts, [size(points, 1), 1]);
            testCase.verifyEqual(sum(layer.samplingProbabilities), 1, AbsTol=1.0e-12);
            testCase.verifyGreaterThanOrEqual(layer.temporalReliabilityScores, zeros(size(points, 1), 1));
            testCase.verifyLessThanOrEqual(layer.temporalReliabilityScores, ones(size(points, 1), 1));
            testCase.verifyEqual(numel(layer.resampledSourceIndices), sum(layer.resampleCounts));
            testCase.verifyEqual(layer.resampleModel, "independentBernoulliTemporalReliabilitySampling");
            testCase.verifyEqual(layer.temporalReliabilityModel, "leaveOneBinOutKernelSupportTimesPatchGeometry");
        end

        function queryUsesPosthocSupportAmplitude(testCase)
        % queryUsesPosthocSupportAmplitude: A query near repeated cross-frame
        % support scores above a far-away query.
            [cfg, points, labels, timestamps] = testCase.syntheticStableFixture();
            gmmMap = buildTemporalStabilityGmmMap(points, labels, timestamps, cfg);
            scores = queryTemporalStabilityGmmMap(gmmMap, [1.5 0; 1.5 3], "stable");

            testCase.verifyGreaterThan(scores(1), scores(2));
            testCase.verifyGreaterThanOrEqual(scores, zeros(2, 1));
            testCase.verifyLessThanOrEqual(scores, ones(2, 1));
        end

        function removedStudentTAndBackgroundConfigFieldsAreRejected(testCase)
        % removedStudentTAndBackgroundConfigFieldsAreRejected: Legacy Student-t
        % and uniform-background fields fail fast instead of silently changing
        % the standard EM training path.
            [cfg, points, labels, timestamps] = testCase.syntheticStableFixture();
            cfg.defaultParams.emStudentTDegreesOfFreedom = 4.0;

            testCase.verifyError(@() buildTemporalStabilityGmmMap(points, labels, timestamps, cfg), ...
                "buildTemporalStabilityGmmMap:RemovedModeField");
        end

        function sharedLoggingKeepsStageSwitchesDistinct(testCase)
            cfg = struct('stepLogsEnabled', true, 'logEnabled', false);
            testCase.verifyTrue(mappingSupport.isLogEnabled(cfg));
            testCase.verifyFalse(mappingSupport.isLogEnabled(cfg, "logEnabled"));
            testCase.verifyFalse(mappingSupport.isLogEnabled(struct()));
            testCase.verifyFalse(mappingSupport.isLogEnabled(struct('stepLogsEnabled', [true false])));
            testCase.verifyEqual(string(mappingSupport.formatFrameIndexSet(260:289)), "260:289");
            testCase.verifyEqual(mappingSupport.formatFrameIndexSet([260 300 326]), "[260 300 326]");
        end

        function sharedSummaryPreservesFiniteAndEmptyResults(testCase)
            [minimum, medianValue, maximum] = mappingSupport.finiteSummary([NaN 4 1 Inf 2]);
            testCase.verifyEqual([minimum medianValue maximum], [1 2 4], AbsTol=1.0e-12);
            [minimum, medianValue, maximum] = mappingSupport.finiteSummary([NaN Inf]);
            testCase.verifyTrue(all(isnan([minimum medianValue maximum])));
        end

        function sharedCovarianceChecksPreserveErrorIdentifiers(testCase)
            testCase.verifyError(@() mappingSupport.validateCovarianceMatrix(nan(2), "fixture"), ...
                "buildTemporalStabilityGmmMap:InvalidCovarianceMatrix");
            testCase.verifyError(@() mappingSupport.validateCovarianceMatrix([1 1; 0 1], "fixture"), ...
                "buildTemporalStabilityGmmMap:NonSymmetricCovariance");
            testCase.verifyError(@() mappingSupport.validateCovarianceMatrix(diag([1 -1]), "fixture"), ...
                "buildTemporalStabilityGmmMap:NonPositiveDefiniteCovariance");
            testCase.verifyError(@() mappingSupport.validateCovarianceMatrix(nan(2), "fixture", ...
                "queryTemporalStabilityGmmMap"), "queryTemporalStabilityGmmMap:InvalidCovarianceMatrix");
            testCase.verifyError(@() mappingSupport.validateCovarianceMatrix([1 1; 0 1], "fixture", ...
                "queryTemporalStabilityGmmMap"), "queryTemporalStabilityGmmMap:NonSymmetricCovariance");
            testCase.verifyError(@() mappingSupport.validateCovarianceMatrix(diag([1 -1]), "fixture", ...
                "queryTemporalStabilityGmmMap"), "queryTemporalStabilityGmmMap:NonPositiveDefiniteCovariance");
        end

        function sharedUnitChecksPreserveToleranceAndErrorIdentifiers(testCase)
            testCase.verifyEqual(mappingSupport.clipUnit([-1.0e-13 0.5 1+1.0e-13]), ...
                [0 0.5 1], AbsTol=1.0e-15);
            testCase.verifyEqual(mappingSupport.clipUnit([-1.0e-13 0.5 1+1.0e-13], ...
                "queryTemporalStabilityGmmMap"), [0 0.5 1], AbsTol=1.0e-15);
            testCase.verifyError(@() mappingSupport.clipUnit(NaN), ...
                "buildTemporalStabilityGmmMap:InvalidUnitValue");
            testCase.verifyError(@() mappingSupport.clipUnit(1+1.0e-6), ...
                "buildTemporalStabilityGmmMap:UnitValueOutOfRange");
            testCase.verifyError(@() mappingSupport.clipUnit(NaN, "queryTemporalStabilityGmmMap"), ...
                "queryTemporalStabilityGmmMap:InvalidUnitValue");
            testCase.verifyError(@() mappingSupport.clipUnit(-1.0e-6, "queryTemporalStabilityGmmMap"), ...
                "queryTemporalStabilityGmmMap:UnitValueOutOfRange");
        end
    end

    methods (Static, Access = private)
        function [cfg, points, labels, timestamps] = syntheticStableFixture()
        % syntheticStableFixture: Three frames of the same short BEV line with
        % small lateral jitter, enough for repeated cross-frame support.
            cfg = temporalStabilityMapConfig();
            cfg.classes = "stable";
            cfg.classParams = cfg.classParams([]);
            cfg.defaultParams.k = 4;
            cfg.defaultParams.radius = 1.5;
            cfg.defaultParams.maxDiameter = 10.0;
            cfg.defaultParams.lengthParallel = 4.0;
            cfg.defaultParams.lengthPerp = 0.5;
            cfg.defaultParams.timestampMaxBins = 3;
            cfg.defaultParams.minTimestampBins = 2;
            cfg.defaultParams.minComponentPoints = 2;
            cfg.defaultParams.minComponentSupport = 0.0;
            cfg.defaultParams.minNormalStability = 0.0;
            cfg.defaultParams.emMaxIterations = 5;
            cfg.defaultParams.emTolerance = 1.0e-4;
            cfg.defaultParams.emMinFrameWeight = 1.0e-9;
            cfg.defaultParams.temporalReliabilityKernelBandwidth = 1.5;
            cfg.defaultParams.temporalReliabilitySamplingFloor = 0.05;
            cfg.defaultParams.temporalResampleRandomSeed = 1;
            cfg.defaultParams.queryCandidateComponentCount = inf;
            cfg.defaultParams.queryCandidateRadius = inf;

            x = repmat((0:0.75:3).', 3, 1);
            y = [zeros(5, 1); 0.05 .* ones(5, 1); -0.04 .* ones(5, 1)];
            points = [x, y];
            labels = repmat("stable", size(points, 1), 1);
            timestamps = repelem((1:3).', 5);
        end
    end
end
