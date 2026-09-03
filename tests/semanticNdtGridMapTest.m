classdef semanticNdtGridMapTest < matlab.unittest.TestCase
% semanticNdtGridMapTest: Unit test of the semantic NDT grid map built from
% coarse 3D semantic voxel tags.

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
        function coarseTagsProjectToSemanticNdtCells(testCase)
        % coarseTagsProjectToSemanticNdtCells: Coarse voxel tags are recovered
        % at the original points, projected into fixed 2D cells per class, and
        % converted into regularized NDT Gaussian cells.
            [frame, semanticGrid, cfg] = semanticNdtGridMapTest.syntheticFixture();
            ndtMap = buildSemanticNdtGridMap(frame, semanticGrid, cfg);

            roadLayer = ndtMap.layers(1);
            poleLayer = ndtMap.layers(2);

            testCase.verifyEqual(ndtMap.components.numComponents, 2);
            testCase.verifyEqual(roadLayer.semanticName, "roadBoundary");
            testCase.verifyEqual(numel(roadLayer.cellLinIdx), 1);
            testCase.verifyEqual(roadLayer.count(1), uint16(3));
            testCase.verifyEqual(roadLayer.mean(1, :), [0.3, 0.3], AbsTol=1.0e-12);
            testCase.verifyEqual(poleLayer.semanticName, "poleCandidate");
            testCase.verifyEqual(numel(poleLayer.cellLinIdx), 1);
            testCase.verifyEqual(poleLayer.count(1), uint16(3));
            testCase.verifyEqual(poleLayer.mean(1, :), [1.3, 1.3], AbsTol=1.0e-12);
            testCase.verifyGreaterThan(det(roadLayer.covariance(:, :, 1)), 0);
            testCase.verifyGreaterThan(det(poleLayer.covariance(:, :, 1)), 0);
        end
    end

    methods (Static, Access = private)
        function [frame, semanticGrid, cfg] = syntheticFixture()
        % syntheticFixture: Two semantic classes occupying two distinct 2D cells.
            frame = struct();
            frame.x = [0.2; 0.3; 0.4; 1.2; 1.3; 1.4];
            frame.y = [0.2; 0.3; 0.4; 1.2; 1.3; 1.4];
            semanticNames = ["empty", "unknown", "roadBoundary", "poleCandidate"];
            primaryTagVolume = zeros(2, 2, 2, "uint8");
            roadVoxelIdx = [sub2ind([2, 2, 2], 1, 1, 1); sub2ind([2, 2, 2], 1, 1, 2); sub2ind([2, 2, 2], 1, 1, 1)];
            poleVoxelIdx = [sub2ind([2, 2, 2], 2, 2, 1); sub2ind([2, 2, 2], 2, 2, 2); sub2ind([2, 2, 2], 2, 2, 1)];
            primaryTagVolume(roadVoxelIdx) = uint8(3);
            primaryTagVolume(poleVoxelIdx) = uint8(4);

            semanticGrid = struct();
            semanticGrid.semanticNames = semanticNames;
            semanticGrid.primaryTagVolume = primaryTagVolume;
            semanticGrid.recovery = struct();
            semanticGrid.recovery.originalPointIdx = (1:6).';
            semanticGrid.recovery.pointVoxelLinIdx = [roadVoxelIdx; poleVoxelIdx];

            cfg = semanticNdtGridMapConfig();
            cfg.xMin = 0;
            cfg.xMax = 2;
            cfg.yMin = 0;
            cfg.yMax = 2;
            cfg.resolution = 1;
            cfg.semanticNames = ["roadBoundary", "poleCandidate"];
            cfg.minPointsPerGaussian = 3;
            cfg.minCovarianceEigenvalue = 1.0e-4;
            cfg.maxCovarianceEigenvalue = 10;
            cfg.regularizationVariance = 1.0e-6;
        end
    end
end
