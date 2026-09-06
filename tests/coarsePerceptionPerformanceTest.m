classdef coarsePerceptionPerformanceTest < matlab.unittest.TestCase
% coarsePerceptionPerformanceTest: Numerical and membership contracts of CPU acceleration.
    properties (TestParameter)
        shapeCase = {[1 1 1],[1 17 2],[19 1 3],[31 27 2],[11 13 5]}
        spacing = {[0.3 0.3],[0.2 0.4]}
        frameIndex = {75,260,775,1100,1125}
    end
    methods (TestClassSetup)
        function paths(~)
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function nativeMomentsRetainSmallScatterAtLargeCoordinates(testCase)
            testCase.assumeTrue(perceptionNativeAvailable);
            points=[700000 4300000 100;700000.002 4300000.004 100.006; ...
                700003 4300004 102;700003.02 4300004.04 102.06];
            cells=[1;1;3;3];
            reference=aggregatePlanarCellMoments(points,cells,4,eye(3),[0 0 0],false);
            actual=aggregatePlanarCellMoments(points,cells,4,eye(3),[0 0 0],true);
            testCase.verifyEqual(actual,reference,'AbsTol',1e-15);
            testCase.verifyGreaterThan(actual.heightCovariance(1,3),0);
        end
        function nativeMomentsPreserveEmptyAndSingletonCells(testCase)
            testCase.assumeTrue(perceptionNativeAvailable);
            empty=aggregatePlanarCellMoments(zeros(0,3),zeros(0,1),0,eye(3),[0 0 0],true);
            one=aggregatePlanarCellMoments([1 2 3],2,3,eye(3),[0 0 0],true);
            testCase.verifyEmpty(empty.count);
            testCase.verifySize(empty.heightCovariance,[0 3]);
            testCase.verifyEqual(one.count,[0;1;0]);
            testCase.verifyEqual(one.heightCovariance,zeros(3),'AbsTol',0);
            testCase.verifyEqual(one.meanZ,[0;3;0],'AbsTol',0);
        end
        function nativeNeighborDifferenceRespectsMissingCells(testCase)
            testCase.assumeTrue(perceptionNativeAvailable);
            values=[NaN 1 3;0 Inf 2]; valid=true(2,3);
            actual=perceptionKernelsMex('neighborDifference',values,valid);
            testCase.verifyEqual(actual,[0 2 2;1 0 1],'AbsTol',0);
        end
        function nativeDirectionalSupportMatchesConvolution(testCase,shapeCase)
            testCase.assumeTrue(perceptionNativeAvailable);
            stream=RandStream('mt19937ar','Seed',1729);
            seed=rand(stream,shapeCase(1),shapeCase(2))>0.45;
            active=rand(stream,size(seed))>0.2;
            parameters=[shapeCase(3) 2 7 0.4 0.9];
            expected=directionalConvolutionReference(seed,active,parameters);
            actual=perceptionKernelsMex('directionalSupport',seed,active,parameters);
            testCase.verifyEqual(actual,expected,'AbsTol',1e-14);
        end
        function nativePillarShapeMatchesMatlabAtRasterEdges(testCase,shapeCase)
            testCase.assumeTrue(perceptionNativeAvailable);
            stream=RandStream('mt19937ar','Seed',1729);
            run=single(randi(stream,12,shapeCase(1),shapeCase(2))-1);
            run(1)=NaN; run(end)=Inf;
            occupied=rand(stream,size(run))>0.2;
            cfg=offGroundFeatureConfig(); cfg.fineShapeScoreNeighborhoodRadiusCells=shapeCase(3);
            cfg.useNativeKernels=false;
            [point,line,orientation]=buildFineColumnShapeScores(run,occupied,cfg,0.2,0.4);
            cfg.useNativeKernels=true;
            [nativePoint,nativeLine,nativeOrientation]=buildFineColumnShapeScores(run,occupied,cfg,0.2,0.4);
            testCase.verifyEqual(nativePoint,point);
            testCase.verifyEqual(nativeLine,line,'AbsTol',single(2e-7));
            testCase.verifyEqual(isnan(nativeOrientation),isnan(orientation));
            finite=isfinite(orientation);
            testCase.verifyEqual(nativeOrientation(finite),orientation(finite),'AbsTol',single(2e-5));
        end
        function nativeGroundPropagationPreservesLabels(testCase,spacing)
            testCase.assumeTrue(perceptionNativeAvailable);
            [x,y]=meshgrid(-12:0.13:12,-5:0.17:5);
            z=-1.44+0.01*x+0.03*sin(y);
            z(x>4 & y>1)=z(x>4 & y>1)+0.45;
            frame=[x(:),y(:),z(:)];
            voxelCfg=frameVoxelizationConfig(); voxelCfg.statisticsMode="sparse";
            grid=voxelizePointCloud(frame,voxelCfg);
            cfg=groundSegmentationConfig(); cfg.slopeGridXYCellSize=spacing;
            cfg.useNativeKernels=false; reference=segmentGround(grid,cfg);
            cfg.useNativeKernels=true; actual=segmentGround(grid,cfg);
            testCase.verifyEqual(actual,reference);
        end
        function roadFloodStopsAtHeightDiscontinuityAndMissingCells(testCase)
            testCase.assumeTrue(perceptionNativeAvailable);
            actual=perceptionKernelsMex('growRoad',true(1,6),1,[0 0.1 0.2 0.5 NaN 0.2],[0 0.11 Inf]);
            testCase.verifyEqual(actual,logical([1 1 1 0 0 0]));
            empty=perceptionKernelsMex('growRoad',false(0),zeros(0,1),zeros(0),[0 1 1]);
            testCase.verifySize(empty,[0 0]);
        end
        function malformedNativeIndicesFailWithoutMutatingInput(testCase)
            testCase.assumeTrue(perceptionNativeAvailable);
            height=zeros(2);
            testCase.verifyError(@() invalidRoadSeed(height),'perception:native:InvalidInput');
            testCase.verifyEqual(height,zeros(2));
        end
        function propagationRetainsTheClosestSupportTolerance(testCase)
            testCase.assumeTrue(perceptionNativeAvailable);
            heights=nan(9,1); heights(1)=0.5; heights(2)=0;
            state=zeros(9,1,'uint8'); state([1 2])=1;
            [result,labels]=perceptionKernelsMex('propagateGround',1,5,0.7,true,0, ...
                heights,state,[3 3],[0.3 0.3],[0.03 0 0.2 0.08 0.25]);
            testCase.verifyEqual(labels(5),uint8(2));
            testCase.verifyEqual(result(5),0.25);
            testCase.verifyTrue(isnan(heights(5)));
        end
        function recordedNativeAndMatlabGroundAndFeaturesAgree(testCase,frameIndex)
            testCase.assumeTrue(perceptionNativeAvailable);
            root=fileparts(fileparts(mfilename('fullpath')));
            file=fullfile(root,'data','raw','MissisipiPointClouds.mat');
            testCase.assumeTrue(isfile(file));
            frame=loadPointCloudFrame(file,frameIndex);
            cfg=perceptionConfig(); cfg.executionMode="offline"; cfg.executionBackend="matlab";
            reference=perceiveFrame(frame,cfg);
            cfg.executionBackend="native"; native=perceiveFrame(frame,cfg);
            testCase.verifyEqual(native.featureMasks,reference.featureMasks);
            testCase.verifyEqual(native.candidates,reference.candidates);
            testCase.verifyEqual(native.probabilityCloud.components.semanticProbability, ...
                reference.probabilityCloud.components.semanticProbability,'AbsTol',1e-10);
        end
        function omittedInverseLookupPreservesForwardMembership(testCase)
            cfg=frameVoxelizationConfig(); cfg.statisticsMode="sparse";
            points=[10 2 -1;10.1 2.1 4;10.1 2.1 -0.8;11 3 0;NaN 0 0];
            full=voxelizePointCloud(points,cfg);
            cfg.buildPointLookup=false; lean=voxelizePointCloud(points,cfg);
            testCase.verifyEqual(lean.points,full.points);
            testCase.verifyEqual(lean.pointIndices,full.pointIndices);
            testCase.verifyEqual(lean.pointVoxelSub,full.pointVoxelSub);
            testCase.verifyEqual(lean.pointVoxelLinIdx,full.pointVoxelLinIdx);
            testCase.verifyEmpty(lean.voxelPointLocalIdx);
            testCase.verifyFalse(lean.hasPointLookup);
            testCase.verifyTrue(isnan(lean.numOccupiedVoxels));
        end
        function recordedCompactRasterPreservesPublicPillarIdsAndFinePoints(testCase,frameIndex)
            root=fileparts(fileparts(mfilename('fullpath')));
            file=fullfile(root,'data','raw','MissisipiPointClouds.mat');
            testCase.assumeTrue(isfile(file));
            frame=loadPointCloudFrame(file,frameIndex);
            cfg=perceptionConfig(); cfg.executionMode="offline"; cfg.executionBackend="matlab";
            cfg.compactGroundRaster=false; full=perceiveFrame(frame,cfg);
            cfg.compactGroundRaster=true; compact=perceiveFrame(frame,cfg);
            testCase.verifyEqual(compact.candidates,full.candidates);
            testCase.verifyEqual(compact.featureMasks,full.featureMasks);
            testCase.verifyEqual(compact.probabilityCloud.components.mean,full.probabilityCloud.components.mean,'AbsTol',1e-12);
            testCase.verifyEqual(compact.probabilityCloud.components.covariance,full.probabilityCloud.components.covariance,'AbsTol',1e-12);
        end
        function backendSelectionIsExplicit(testCase)
            testCase.verifyFalse(perceptionNativeAvailable("matlab"));
            testCase.verifyError(@() perceptionNativeAvailable("unsupported"),'perception:InvalidBackend');
        end
        function singleCurbAnchorPromotesBothNeighborPeaks(testCase)
            [energy,cfg,view,seeds]=singleAnchorCurbScenario();
            result=refineCurbCellsByRoadAdjacency(energy,true(7),cfg,view,seeds);
            expected=false(7); expected(1:3,4)=true;
            testCase.verifyEqual(result.extractedMask,expected);
        end
    end
end

function invalidRoadSeed(height)
    unused=perceptionKernelsMex('growRoad',true(2),5,height,[0 1 1]); %#ok<NASGU>
end

function [energy,cfg,view,seeds] = singleAnchorCurbScenario()
    groundCfg=groundFeatureConfig(); cfg=groundCfg.curb;
    names=fieldnames(cfg);
    for k=1:numel(names)
        if endsWith(names{k},'Enabled'), cfg.(names{k})=false; end
    end
    cfg.roadAdjacencyFilterEnabled=true; cfg.sameColumnFeaturePeakSelectionEnabled=true;
    cfg.roadAdjacencyComponentMinAdjacentCells=1; cfg.roadAdjacencyComponentMinAdjacentFraction=0;
    cfg.sameColumnFeaturePeakSuppressionMinHeightGainMeters=1;
    energy=struct('extractedMask',false(7),'total',zeros(7),'totalBase',zeros(7), ...
        'heightStepMeters',zeros(7),'roughnessMeters',zeros(7),'linearity',zeros(7), ...
        'linearityComponentCenterEvidence',zeros(7));
    energy.extractedMask(2,4)=true; energy.total(2,4)=1;
    energy.totalBase(1:3,4)=0.5; energy.heightStepMeters(1:3,4)=[0.14;0.10;0.14];
    energy.roughnessMeters(1:3,4)=0.015; energy.linearity(1:3,4)=0.5;
    [x,y]=meshgrid(-3:3,-3:3);
    view=struct('xCenters',-3:3,'yCenters',(-3:3).','xMap',x,'yMap',y,'cellSize',[1 1]);
    seeds=false(7); seeds(4,4)=true;
end

function result=directionalConvolutionReference(seed,active,p)
% Independent dense convolution oracle for the sparse native traversal.
    result=zeros(size(seed)); directions=[0 1;1 0;1 1;1 -1];
    radius=p(1); width=2*radius+3; center=radius+2;
    for direction=directions.'
        kernels=zeros(width,width,3);
        normal=[-direction(2);direction(1)];
        for side=-1:1
            offsets=direction*(-radius:radius)+normal*side;
            ids=sub2ind([width width],center-offsets(1,:),center-offsets(2,:));
            kernel=zeros(width); kernel(ids)=1;
            kernels(:,:,side+2)=kernel;
        end
        counts=zeros([size(seed),3]); valid=true(size(seed));
        for k=1:3
            counts(:,:,k)=conv2(double(seed),kernels(:,:,k),'same');
            valid=valid & conv2(ones(size(seed)),kernels(:,:,k),'same')==2*radius+1;
        end
        line=counts(:,:,2); sides=min(counts(:,:,1),counts(:,:,3));
        thin=line./max(line+sides,eps);
        a=min(1,max(0,(line-p(2))/(p(3)-p(2))));
        b=min(1,max(0,(thin-p(4))/(p(5)-p(4))));
        score=(3*a.^2-2*a.^3).*(3*b.^2-2*b.^3);
        score(~valid | ~active)=0; result=max(result,score);
    end
end
