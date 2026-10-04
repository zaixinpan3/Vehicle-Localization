classdef downtownStatisticsModelTest < matlab.unittest.TestCase
% downtownStatisticsModelTest: Portable statistical model and scope contracts.
    methods (TestClassSetup)
        function paths(~)
            setupVehicleLocalization;
        end
    end
    methods (Test)
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
            testCase.verifyFalse(coarse.downtownCandidates.useModels);
            testCase.verifyEqual(coarse.downtownCandidates,fine.downtownCandidates);
            testCase.verifyEqual(perceptionConfig('Mississippi').voxel.voxelSize,[.6 .6]);
            testCase.verifyEqual(perceptionConfig('Carla').voxel.voxelSize,[.6 .6]);
        end
        function trainedModelsUseOnlyDistributionFeatures(testCase)
            cfg=downtownCandidateConfig(.6);
            testCase.verifyTrue(cfg.useModels);
            testCase.verifyFalse(downtownCandidateConfig(.3).useModels);
            for name=["curb","facade","pole","trafficSign"]
                model=jsondecode(fileread(fullfile(cfg.modelDirectory,cfg.modelFiles.(name))));
                features=string(model.featureNames);
                testCase.verifyEqual(numel(unique(features)),numel(features));
                testCase.verifyFalse(any(ismember(lower(features),["frame","pointindex","pillarindex","dataset","x","y"])));
                testCase.verifyTrue(all(isfinite(model.threshold)));
                testCase.verifyGreaterThan(model.decisionThreshold,0);
                testCase.verifyLessThan(model.decisionThreshold,1);
                testCase.verifyGreaterThanOrEqual(model.value,zeros(size(model.value)));
                testCase.verifyLessThanOrEqual(model.value,ones(size(model.value)));
                testCase.verifyEqual(numel(model.left),numel(model.value));
                testCase.verifyEqual(numel(model.right),numel(model.value));
                score=scoreSemanticPillarModel(zeros(2,numel(features)),model,model.decisionThreshold);
                testCase.verifyTrue(all(isfinite(score)));
                testCase.verifyEqual(score(1),score(2));
            end
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
        function nestedDiagnosticsMatchFinalStatisticalAndEnvelopeMasks(testCase)
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
                % Deliberately stale aliases must be refreshed regardless of
                % whether a model or the offline envelope selects candidates.
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
