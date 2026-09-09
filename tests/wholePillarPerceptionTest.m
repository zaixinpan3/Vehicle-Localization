classdef wholePillarPerceptionTest < matlab.unittest.TestCase
% wholePillarPerceptionTest: XYZ sufficient statistics and execution boundary.
    methods (TestClassSetup)
        function paths(~)
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function wholePillarMomentsRetainAllXYZCorrelations(testCase)
            cfg=frameVoxelizationConfig(); cfg.roiLimits=[0 2 0 2];
            cfg.exclusionHalfSize=0;
            p=[.01 .02 -4;.05 .08 1;.08 .14 8];
            grid=pillarizePointCloud(p,cfg);
            expected=cov(p,1);
            testCase.verifyEqual(grid.statistics.count,3);
            testCase.verifyEqual(grid.statistics.meanXYZ,mean(p,1),'AbsTol',1e-12);
            testCase.verifyEqual(grid.statistics.covarianceXYZ,expected([1 2 5 3 6 9]),'AbsTol',1e-12);
            testCase.verifyEqual(grid.statistics.minimumXYZ,min(p,[],1),'AbsTol',1e-12);
            testCase.verifyEqual(grid.statistics.maximumXYZ,max(p,[],1),'AbsTol',1e-12);
            testCase.verifyFalse(any(contains(string(fieldnames(grid)),["Voxel","voxelStatistics"])));
            testCase.verifySize(grid.pointPillarSub,[3 2]);
        end
        function verticalBinSettingsCannotAffectPillars(testCase)
            cfg=frameVoxelizationConfig(); cfg.roiLimits=[0 2 0 2]; cfg.exclusionHalfSize=0;
            points=[.01 .02 -100;.05 .08 .3;.08 .14 100];
            first=pillarizePointCloud(points,cfg);
            cfg.voxelSize(3)=NaN; cfg.buildPointLookup=true;
            second=pillarizePointCloud(points,cfg);
            testCase.verifyEqual(second,first);
        end
        function emptyInputRetainsValidStatistics(testCase)
            grid=pillarizePointCloud(zeros(0,3),frameVoxelizationConfig());
            testCase.verifySize(grid.statistics.meanXYZ,[0 3]);
            testCase.verifySize(grid.statistics.covarianceXYZ,[0 6]);
            testCase.verifyEmpty(grid.statistics.count);
        end
        function radiometryDoesNotRemoveMembersFromGeometry(testCase)
            points=[.01 .02 0;.02 .03 2;.03 .04 4];
            stats=aggregatePillarStatistics(points,ones(3,1),struct('intensity',[1;NaN;1700]));
            testCase.verifyEqual(stats.count,3);
            testCase.verifyEqual(stats.intensity.count,2);
            testCase.verifyEqual(stats.intensity.maximum,1700);
            testCase.verifyEqual(stats.meanXYZ,mean(points,1),'AbsTol',1e-12);
        end
        function splitBoundaryPoleUsesWholeNeighborPillars(testCase)
            cfg=frameVoxelizationConfig(); cfg.roiLimits=[0 3 0 3]; cfg.exclusionHalfSize=0;
            z=repelem(linspace(-1,3,12).',2);
            x=repmat([1.19;1.21],12,1); y=ones(size(x))*1.05;
            grid=pillarizePointCloud([x y z],cfg);
            cloudCfg=coarseSemanticProbabilityCloudConfig(); cloudCfg.semanticNames="pole";
            result=analyzeStructuralPillars(grid,structuralPillarConfig(),cloudCfg);
            testCase.verifyEqual(nnz(result.poleCellMask),2);
            testCase.verifyEqual(sum(result.columnMaps.statistics.count),24);
            testCase.verifyFalse(isfield(result.columnMaps,'voxelStatistics'));
        end
        function horizontalDistributionIsNotAPole(testCase)
            cfg=frameVoxelizationConfig(); cfg.roiLimits=[0 3 0 3]; cfg.exclusionHalfSize=0;
            x=linspace(1.21,1.49,20).'; y=ones(size(x))*1.35;
            grid=pillarizePointCloud([x y zeros(size(x))],cfg);
            cloudCfg=coarseSemanticProbabilityCloudConfig(); cloudCfg.semanticNames="pole";
            result=analyzeStructuralPillars(grid,structuralPillarConfig(),cloudCfg);
            testCase.verifyFalse(any(result.poleCellMask,'all'));
        end
        function reportedMissedPoleEntersCoarseAndOptionalFineRecovery(testCase)
            file=fullfile(fileparts(fileparts(mfilename('fullpath'))),'data','raw','MissisipiPointClouds.mat');
            testCase.assumeTrue(isfile(file));
            frame=loadPointCloudFrame(file,260);
            cfg=perceptionConfig(); cfg.executionMode="offline";
            cfg.fine.poleRecoveryEnabled=true;
            result=perceiveFrame(frame,cfg);
            grid=pillarizePointCloud(frame,cfg.voxel);
            row=find(grid.pointIndices==63010);
            pole=result.candidates.pillarIndices{result.candidates.semanticNames=="pole"};
            testCase.verifyTrue(ismember(grid.pointPillarLinIdx(row),pole));
            testCase.verifyTrue(result.featureMasks.pole(63010));
        end
        function coarseRuntimeHasNoSubpillarOrFineCalls(testCase)
            cfg=perceptionConfig("Downtown"); cfg.featureNames=["pole","facade","trafficSign"];
            frame=struct('x',[10;10.1;10.15],'y',[2;2.1;2.05],'z',[-1;1;3]);
            profile clear; profile on;
            result=perceiveFrame(frame,cfg);
            profile off; audit=profile('info');
            names=string({audit.FunctionTable.FunctionName});
            forbidden=["voxelizePointCloud","analyzeFineStructuralCandidates","detectPoleCandidates", ...
                "refinePerceptionCandidates","buildFineColumnFeatureMaps","refineFacadeWithFineGrid"];
            testCase.verifyFalse(any(contains(names,forbidden)));
            testCase.verifyFalse(isfield(result,'refinement'));
        end
    end
end
