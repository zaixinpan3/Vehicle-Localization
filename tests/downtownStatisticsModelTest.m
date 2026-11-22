classdef downtownStatisticsModelTest < matlab.unittest.TestCase
% downtownStatisticsModelTest: Downtown pillar candidate rules and channel scope.
    methods (TestClassSetup)
        function paths(~)
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function usesBalancedRulesInBothProducts(testCase)
            for mode=["coarseProbabilityCloud","offline"]
                cfg=perceptionConfig("Downtown",mode);
                c=cfg.downtownCandidates;
                testCase.verifyEqual(c.precisionFilter.rules,jsondecode(fileread( ...
                    fullfile(fileparts(which('perceptionConfig')),'downtownBalancedRules.json'))));
            end
        end
        function pooledPoleGateRejectsSpreadAndIgnoresCoordinateTranslation(testCase)
            stats=struct('pillarIndices',int32([1;5]),'count',[30;30], ...
                'meanXYZ',[10 5 2;11 5 2], ...
                'covarianceXYZ',[.001 0 .001 0 0 1;.2 0 .001 0 0 .1]);
            maps=struct('statistics',stats,'mapSize',[1 5]);
            cfg=struct('radiusCells',1,'minimumVerticalVarianceFraction',.9, ...
                'maximumRadialVariance',.1);
            original=filterDowntownPoleDistribution(maps,cfg);
            maps.statistics.meanXYZ=maps.statistics.meanXYZ+[200 -300 50];
            translated=filterDowntownPoleDistribution(maps,cfg);
            testCase.verifyEqual(original,[true false false false false]);
            testCase.verifyEqual(translated,original);
        end
        function downtownCoarseSharesTheFineCandidateLattice(testCase)
            coarse=perceptionConfig('Downtown');
            fine=perceptionConfig('Downtown','offline');
            testCase.verifyEqual(coarse.executionMode,"coarseProbabilityCloud");
            testCase.verifyEqual(coarse.voxel.voxelSize,[.3 .3]);
            testCase.verifyEqual(coarse.voxel.gridDims,[334 334]);
            testCase.verifyEqual(coarse.voxel.voxelSize,fine.voxel.voxelSize);
            testCase.verifyEqual(pillarGridExtent(coarse.voxel),pillarGridExtent(fine.voxel));
            testCase.verifyEqual(coarse.downtownCandidates,fine.downtownCandidates);
            testCase.verifyEqual(perceptionConfig('Mississippi').voxel.voxelSize,[.6 .6]);
            testCase.verifyEqual(perceptionConfig('Carla').voxel.voxelSize,[.6 .6]);
        end
        function rejectsHistoricalClassifierLattice(testCase)
            testCase.verifyError(@()downtownCandidateConfig(.6),'perception:DowntownRuleLattice');
        end
        function otherDatasetsRetainTheirExistingConfiguration(testCase)
            for name=["Mississippi","Carla"]
                cfg=perceptionConfig(name);
                testCase.verifyFalse(isfield(cfg,'downtownCandidates'));
                testCase.verifyFalse(isfield(cfg,'downtownStructure'));
                testCase.verifyFalse(isfield(cfg,'downtownCurb'));
            end
        end
        function singleRowRasterRetainsOneDescriptorPerPillar(testCase)
            points=[.1 .1 0;.1 .1 2;1.3 .1 0;1.3 .1 3];
            pillars=struct('points',points,'pointPillarLinIdx',int32([1;1;3;3]), ...
                'pointAttributes',struct(),'pillarGeometry',struct('origin',[0 0], ...
                'cellSize',[.6 .6],'mapSize',[1 3]));
            cfg=perceptionConfig('Downtown');cloud=coarseSemanticProbabilityCloudConfig;
            branch=analyzeDowntownPillarStatistics(pillars,cfg.offGroundFeatures,cloud);
            [x,names,ids]=measureDowntownPillarStatistics(struct(),branch,'pole');
            testCase.verifyEqual(ids,[1;3]);
            testCase.verifySize(x,[2 numel(names)]);
            testCase.verifyTrue(all(isfinite(x),'all'));
        end
        function nestedDiagnosticsMatchFinalCandidateMasks(testCase)
            points=[10 5 0;10.03 5.04 1.4;10.05 5.01 2.5; ...
                10.7 5.01 .8;10.72 5.03 2.1;10.74 5.05 2.6];
            frame=struct('x',points(:,1),'y',points(:,2),'z',points(:,3), ...
                'intensity',[100;100;100;1700;1800;1900]);
            names=["pole","facade","trafficSign"];
            for mode=["coarseProbabilityCloud","offline"]
                cfg=perceptionConfig('Downtown',mode);
                pillars=pillarizePointCloud(frame,cfg.voxel);
                cloud=coarseSemanticProbabilityCloudConfig(cfg.voxel);cloud.semanticNames=names;
                branch=analyzeDowntownPillarStatistics(pillars,cfg.offGroundFeatures,cloud);
                % Both products must refresh aliases from the final candidate decisions.
                branch.facade.mask(:)=true;branch.facade.pillarLinIdx=1;
                branch.candidates.candidateMask(:)=true;
                branch.columnMaps.trafficSignCellMask(:)=true;
                [~,branch]=classifyDowntownPillarStatistics(struct(),branch,names,cfg.downtownCandidates);
                testCase.verifyEqual(branch.facade.mask,branch.facadeCellMask);
                testCase.verifyEqual(branch.facade.pillarLinIdx,find(branch.facadeCellMask));
                testCase.verifyEqual(branch.candidates.candidateMask,branch.poleCellMask);
                testCase.verifyEqual(branch.columnMaps.trafficSignCellMask,branch.trafficSignCellMask);
            end
        end
    end
end
