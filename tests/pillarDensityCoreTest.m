classdef pillarDensityCoreTest < matlab.unittest.TestCase
% pillarDensityCoreTest: Metric neighbourhood and selective evaluation contracts.
    methods (TestClassSetup)
        function paths(~)
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function isolationIncludesTheOtherHalfOfABoundaryShaft(testCase)
            geometry=struct('origin',[0 0],'cellSize',[0.6 0.6],'mapSize',[1 3]);
            points=[0.59 0.3 0;0.59 0.3 2;0.61 0.3 0;0.61 0.3 2; ...
                1.0 0.3 1;1.3 0.3 1];
            [fraction,height,isolation,peak,count]=computePillarDensityCore( ...
                points,[1;1;2;2;2;3],geometry,0.1,0.15,0.6,[true;false;false]);
            testCase.verifyEqual(fraction,[1;0;0],'AbsTol',1e-12);
            testCase.verifyEqual(height,[2;0;0],'AbsTol',1e-12);
            testCase.verifyEqual(isolation,[4/5;0;0],'AbsTol',1e-12);
            testCase.verifyEqual(count,[4;0;0],'AbsTol',1e-12);
            testCase.verifyEqual(peak(1,:),[0.59 0.3],'AbsTol',1e-12);
            testCase.verifyTrue(all(isnan(peak(2:3,:)),'all'));
        end
        function clippedColumnRasterUsesMetricDistance(testCase)
            geometry=struct('origin',[0 0],'cellSize',[0.6 0.6],'mapSize',[3 1]);
            points=[0.3 0.59 0;0.3 0.59 2;0.3 0.61 0;0.3 0.61 2; ...
                0.3 1.0 1;0.3 1.3 1];
            [~,~,isolation]=computePillarDensityCore( ...
                points,[1;1;2;2;2;3],geometry,0.1,0.15,0.6,[true;false;false]);
            testCase.verifyEqual(isolation,[4/5;0;0],'AbsTol',1e-12);
        end
        function evaluationMaskDoesNotRemoveNeighbourEvidence(testCase)
            geometry=struct('origin',[0 0],'cellSize',[0.6 0.6],'mapSize',[1 2]);
            points=[0.59 0.3 0;0.59 0.3 2;0.9 0.3 0;0.9 0.3 2];
            [~,~,allIsolation]=computePillarDensityCore(points,[1;1;2;2],geometry,0.1,0.15,0.6);
            [~,~,selectedIsolation]=computePillarDensityCore( ...
                points,[1;1;2;2],geometry,0.1,0.15,0.6,[true;false]);
            testCase.verifyEqual(selectedIsolation,[0.5;0],'AbsTol',1e-12);
            testCase.verifyEqual(selectedIsolation(1),allIsolation(1),'AbsTol',1e-12);
        end
        function emptyAndUnevaluatedInputsKeepOutputShapes(testCase)
            geometry=struct('origin',[0 0],'cellSize',[0.6 0.6],'mapSize',[1 1]);
            [f,h,i,p]=computePillarDensityCore(zeros(0,3),zeros(0,1),geometry,0.1,0.15);
            testCase.verifySize(f,[0 1]);
            testCase.verifySize(h,[0 1]);
            testCase.verifySize(i,[0 1]);
            testCase.verifySize(p,[0 2]);
            [f,h,i,p]=computePillarDensityCore([0.3 0.3 0],1,geometry,0.1,0.15,0.6,false);
            testCase.verifyEqual([f h i],[0 0 0],'AbsTol',1e-12);
            testCase.verifyTrue(all(isnan(p)));
        end
        function invalidEvaluationLengthIsRejected(testCase)
            geometry=struct('origin',[0 0],'cellSize',[0.6 0.6],'mapSize',[1 1]);
            testCase.verifyError(@() computePillarDensityCore( ...
                [0.3 0.3 0],1,geometry,0.1,0.15,0.6,[true;false]), ...
                'perception:InvalidDensityCore');
        end
    end
end
