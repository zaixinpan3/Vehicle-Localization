classdef wholePillarPerceptionTest < matlab.unittest.TestCase
% wholePillarPerceptionTest: XYZ sufficient statistics and execution boundary.
    methods (TestClassSetup)
        function paths(~)
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function wholePillarMomentsRetainAllXYZCorrelations(testCase)
            cfg=pillarGridConfig(); cfg.exclusionHalfSize=0;
            p=[-.19 -.18 -4;-.15 -.12 1;-.12 -.06 8];
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
        function offlineLatticePreservesTunedCellBoundaries(testCase)
            cfg=pillarGridConfig("offline");
            testCase.verifyFalse(isfield(cfg,'roiLimits'));
            testCase.verifyEqual(cfg.gridDims,[334 334]);
            testCase.verifyEqual(pillarGridExtent(cfg),[-50 50.2 -50 50.2],'AbsTol',1e-12);
            p=[-50 -50 0;50.19 -49.99 1;50.21 0 2;0 -50.01 3;12 5 4];
            grid=pillarizePointCloud(p,cfg);
            testCase.verifyEqual(grid.gridConfig.dims,[334 334]);
            testCase.verifyEqual(grid.gridConfig.minCorner,[-50 -50],'AbsTol',1e-12);
            testCase.verifyEqual(grid.gridConfig.maxCorner,[50.2 50.2],'AbsTol',1e-12);
            testCase.verifyEqual(grid.pillarGeometry.mapSize,[334 334]);
            testCase.verifyEqual(grid.pointIndices,int32([1;2;5]));
            testCase.verifyEqual(grid.pointPillarSub,int32([1 1;334 1;207 184]));
            testCase.verifyEqual(grid.numFilteredPoints,3);
        end
        function coarseLatticeNestsInsideTheOfflineLattice(testCase)
            cfg=pillarGridConfig();
            testCase.verifyEqual(cfg.executionMode,"coarseProbabilityCloud");
            testCase.verifyEqual(cfg.voxelSize,[0.6 0.6]);
            testCase.verifyEqual(cfg.gridDims,[100 100]);
            testCase.verifyEqual(pillarGridExtent(cfg),[-29.9 30.1 -29.9 30.1],'AbsTol',1e-12);
            % Every coarse boundary -29.9 + 0.6 k is an offline boundary -50 + 0.3 j.
            offline=pillarGridExtent(pillarGridConfig("offline"));
            k=0:100; j=(-29.9+0.6*k-offline(1))/0.3;
            testCase.verifyEqual(j,round(j),'AbsTol',1e-9);
            p=[-29.9 -29.9 0;30.09 -29.89 1;30.11 0 2;0 -29.91 3;12 5 4];
            grid=pillarizePointCloud(p,cfg);
            testCase.verifyEqual(grid.gridConfig.dims,[100 100]);
            testCase.verifyEqual(grid.pointIndices,int32([1;2;5]));
            testCase.verifyEqual(grid.pointPillarSub,int32([1 1;100 1;70 59]));
            testCase.verifyError(@() pillarGridConfig("legacyFull"),'perception:InvalidExecutionMode');
        end
        function obsoleteRoiLimitsAreRejected(testCase)
            cfg=pillarGridConfig(); cfg.roiLimits=[0 1 0 1];
            testCase.verifyError(@() pillarizePointCloud([0.5 0.5 0],cfg),'perception:ObsoleteRoiLimits');
            testCase.verifyError(@() pillarGridExtent(cfg),'perception:ObsoleteRoiLimits');
        end
        function rejectsVerticalPillarSpacing(testCase)
            cfg=pillarGridConfig(); cfg.voxelSize(3)=0.5;
            testCase.verifyError(@() pillarizePointCloud([10 2 3],cfg), ...
                'perception:InvalidPillarSpacing');
        end
        function emptyInputRetainsValidStatistics(testCase)
            grid=pillarizePointCloud(zeros(0,3),pillarGridConfig());
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
            cfg=pillarGridConfig(); cfg.exclusionHalfSize=0;
            z=repelem(linspace(-1,3,12).',2);
            x=repmat([1.29;1.31],12,1); y=ones(size(x))*1.05;
            grid=pillarizePointCloud([x y z],cfg);
            cloudCfg=coarseSemanticProbabilityCloudConfig(); cloudCfg.semanticNames="pole";
            result=analyzeStructuralPillars(grid,structuralPillarConfig(),cloudCfg);
            testCase.verifyEqual(nnz(result.poleCellMask),2);
            testCase.verifyEqual(sum(result.columnMaps.statistics.count),24);
            testCase.verifyFalse(isfield(result.columnMaps,'voxelStatistics'));
        end
        function densityCoreLocatesTheShaftInsideAWidePillar(testCase)
            % 30 shaft returns on a 0.12 m circle plus 12 returns spread over
            % the 0.6 m pillar: the peak holds the shaft, not the clutter.
            angle=linspace(0,2*pi,31).'; angle(end)=[];
            shaft=[1.6+0.06*cos(angle),1.6+0.06*sin(angle),linspace(-1,3,30).'];
            clutter=[1.3+0.6*rand(12,1),1.3+0.6*rand(12,1),0.5+0.2*randn(12,1)];
            geometry=struct('origin',[1.3 1.3],'cellSize',[0.6 0.6],'mapSize',[1 1]);
            [fraction,height]=computePillarDensityCore([shaft;clutter],ones(42,1),geometry,0.10,0.15);
            testCase.verifyGreaterThanOrEqual(fraction,30/42);
            testCase.verifyEqual(height,4,'AbsTol',0.5);
            [fraction,height]=computePillarDensityCore(clutter,ones(12,1),geometry,0.10,0.15);
            testCase.verifyLessThan(fraction,0.6);
            testCase.verifyLessThan(height,1.5);
            testCase.verifyError(@() computePillarDensityCore(shaft,ones(30,1),geometry,0,0.15), ...
                'perception:InvalidDensityCore');
        end
        function shaftSharingItsPillarWithClutterStaysAPole(testCase)
            rng(7);
            cfg=pillarGridConfig(); cfg.exclusionHalfSize=0;
            angle=linspace(0,2*pi,41).'; angle(end)=[];
            shaft=[1.6+0.05*cos(angle),1.6+0.05*sin(angle),linspace(-1,3,40).'];
            clutter=[1.3+0.6*rand(16,1),1.3+0.6*rand(16,1),0.3+0.3*randn(16,1)];
            grid=pillarizePointCloud([shaft;clutter],cfg);
            cloudCfg=coarseSemanticProbabilityCloudConfig(); cloudCfg.semanticNames="pole";
            structural=structuralPillarConfig(cfg.voxelSize(1));
            result=analyzeStructuralPillars(grid,structural,cloudCfg);
            testCase.verifyEqual(nnz(result.poleCellMask),1);
            % The total scatter alone would have rejected the shared pillar.
            radial=sqrt(sum(var([shaft;clutter],1)));
            testCase.verifyGreaterThan(radial,0.18);
            % Without the shaft the same clutter is not a pole.
            clutterOnly=pillarizePointCloud([clutter;clutter+[0.01 0.01 2]],cfg);
            result=analyzeStructuralPillars(clutterOnly,structural,cloudCfg);
            testCase.verifyFalse(any(result.poleCellMask,'all'));
        end
        function hedgeIsolationDoesNotVetoAnExistingShaft(testCase)
            % The shaft's own pillar is clean, but a hedge fills the next
            % pillar within 0.6 m of the peak: the legacy global isolation is low,
            % but an independently supported shaft must remain detectable.
            angle=linspace(0,2*pi,41).'; angle(end)=[];
            shaft=[1.6+0.05*cos(angle),1.6+0.05*sin(angle),linspace(-1,3,40).'];
            % A dense 1.3 m hedge 0.35 m from the shaft, too low to be a pole.
            [hx,hz]=meshgrid(1.95:0.05:2.15,-1:0.05:0.3);
            hedge=[hx(:),1.6+0.02*randn(numel(hx),1),hz(:)];
            geometry=struct('origin',[1.3 1.3],'cellSize',[0.6 0.6],'mapSize',[2 2]);
            ids=@(p) sub2ind([2 2],floor((p(:,2)-1.3)/0.6)+1,floor((p(:,1)-1.3)/0.6)+1);
            [fraction,~,isolation]=computePillarDensityCore(shaft,ids(shaft),geometry,0.10,0.15,0.60);
            testCase.verifyEqual(fraction,1);
            testCase.verifyEqual(isolation,1);
            both=[shaft;hedge];
            [~,~,isolation]=computePillarDensityCore(both,ids(both),geometry,0.10,0.15,0.60);
            shaftRow=unique(ids(both))==ids(shaft(1,:));
            testCase.verifyLessThan(isolation(shaftRow),0.30);
            testCase.verifyError(@() computePillarDensityCore(shaft,ids(shaft),geometry,0.10,0.15,0.10), ...
                'perception:InvalidDensityCore');
            cfg=pillarGridConfig(); cfg.exclusionHalfSize=0;
            cloudCfg=coarseSemanticProbabilityCloudConfig(); cloudCfg.semanticNames="pole";
            structural=structuralPillarConfig(cfg.voxelSize(1));
            legacy=analyzeStructuralPillars(pillarizePointCloud(both,cfg),structural,cloudCfg);
            testCase.verifyFalse(any(legacy.poleCellMask,'all'));
            structural.pole.detector="subset"; structural.pole.probabilityEvidence="subset";
            result=analyzeStructuralPillars(pillarizePointCloud(shaft,cfg),structural,cloudCfg);
            testCase.verifyEqual(nnz(result.poleCellMask),1);
            result=analyzeStructuralPillars(pillarizePointCloud(both,cfg),structural,cloudCfg);
            testCase.verifyTrue(any(result.poleCellMask,'all'));
        end
        function horizontalDistributionIsNotAPole(testCase)
            cfg=pillarGridConfig(); cfg.exclusionHalfSize=0;
            x=linspace(1.21,1.49,20).'; y=ones(size(x))*1.35;
            grid=pillarizePointCloud([x y zeros(size(x))],cfg);
            cloudCfg=coarseSemanticProbabilityCloudConfig(); cloudCfg.semanticNames="pole";
            result=analyzeStructuralPillars(grid,structuralPillarConfig(),cloudCfg);
            testCase.verifyFalse(any(result.poleCellMask,'all'));
        end
        function reportedMissedPoleEntersDefaultFineDetection(testCase)
            file=fullfile(fileparts(fileparts(mfilename('fullpath'))),'data','raw','MissisipiPointClouds.mat');
            testCase.assumeTrue(isfile(file));
            frame=loadPointCloudFrame(file,260);
            cfg=perceptionConfig("Mississippi","offline");
            result=perceiveFrame(frame,cfg);
            grid=pillarizePointCloud(frame,cfg.voxel);
            row=find(grid.pointIndices==63010);
            pole=result.candidates.pillarIndices{result.candidates.semanticNames=="pole"};
            testCase.verifyTrue(ismember(grid.pointPillarLinIdx(row),pole));
            testCase.verifyTrue(result.featureMasks.pole(63010));
            reference=loadPerceptionMaskReference("Missisipi",260);
            added=find(result.featureMasks.pole & ~reference.featureMasks.pole);
            xy=double([frame.x(added),frame.y(added)]);
            center=double([frame.x(63010),frame.y(63010)]);
            testCase.verifyNumElements(added,17);
            testCase.verifyLessThan(max(vecnorm(xy-center,2,2)),0.35);
            testCase.verifyTrue(all(result.featureMasks.pole(reference.featureMasks.pole)));
        end
        function coarseRuntimeHasNoSubpillarOrFineCalls(testCase)
            cfg=perceptionConfig("Downtown"); cfg.featureNames=["pole","facade","trafficSign"];
            frame=struct('x',[10;10.1;10.15],'y',[2;2.1;2.05],'z',[-1;1;3]);
            profile clear; profile on;
            result=perceiveFrame(frame,cfg);
            profile off; audit=profile('info');
            names=string({audit.FunctionTable.FunctionName});
            forbidden=["voxelizePillars","analyzeFineStructuralCandidates","detectPoleCandidates", ...
                "refinePerceptionCandidates","buildFineColumnFeatureMaps","refineFacadeWithFineGrid"];
            testCase.verifyFalse(any(contains(names,forbidden)));
            testCase.verifyFalse(isfield(result,'refinement'));
        end
    end
end
