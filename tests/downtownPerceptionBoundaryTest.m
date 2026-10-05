classdef downtownPerceptionBoundaryTest < matlab.unittest.TestCase
% downtownPerceptionBoundaryTest: Enforce statistical coarse/fine separation.
    properties
        Root
    end
    methods (TestClassSetup)
        function setupPaths(testCase)
            testCase.Root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(testCase.Root));
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function syntheticCoarseUsesOnlyPillarStatistics(testCase)
            cfg=perceptionConfig('Downtown');
            auditCoarseExecution(testCase,syntheticFrame(),cfg);
        end

        function recordedCoarseNeverInvokesFineValidators(testCase)
            file=fullfile(testCase.Root,'data','raw','downTownPointClouds.mat');
            testCase.assumeTrue(isfile(file));
            frame=loadPointCloudFrame(file,25);
            auditCoarseExecution(testCase,frame,perceptionConfig('Downtown'));
        end

        function fineDecisionsAndRejectedPointsRemainInsideCandidates(testCase)
            file=fullfile(testCase.Root,'data','raw','downTownPointClouds.mat');
            testCase.assumeTrue(isfile(file));
            rejected=0;
            for frameIndex=[75,200]
                frame=loadPointCloudFrame(file,frameIndex);
                cfg=perceptionConfig('Downtown','offline');
                result=perceiveFrame(frame,cfg);
                grid=pillarizePointCloud(frame,cfg.voxel);
                testCase.verifyEqual(result.fineCandidates.semanticNames,result.candidates.semanticNames);
                for channel=1:numel(result.candidates.semanticNames)
                    name=result.candidates.semanticNames(channel);
                    candidates=result.candidates.pillarIndices{channel};
                    members=double(grid.pointIndices(ismember(grid.pointPillarLinIdx,candidates)));
                    if name=="curb"
                        members=members(result.featureMasks.groundPoint(members));
                    else
                        members=members(~result.featureMasks.groundPoint(members));
                    end
                    audit=result.refinement.(name);
                    testCase.verifyEqual(sort(double(audit.evaluatedPointIndices)),sort(members));
                    testCase.verifyEqual(result.fineCandidates.pillarIndices{channel},candidates, ...
                        sprintf('%s frame %d changed the coarse candidate set.',name,frameIndex));
                    testCase.verifyTrue(all(ismember(find(result.featureMasks.(name)),members)), ...
                        sprintf('%s frame %d labeled a point outside its candidate pillars.',name,frameIndex));
                    testCase.verifyTrue(all(ismember(double(audit.evaluatedPointIndices),members)), ...
                        sprintf('%s frame %d evaluated a semantic point outside its candidate pillars.',name,frameIndex));
                    testCase.verifyEqual(numel(unique(audit.evaluatedPointIndices)),numel(audit.evaluatedPointIndices));
                    testCase.verifyEqual(audit.numEvaluated,numel(audit.evaluatedPointIndices));
                    testCase.verifyEqual(audit.numAccepted,nnz(audit.accepted));
                    testCase.verifyEqual(sort(double(audit.evaluatedPointIndices(audit.accepted))), ...
                        find(result.featureMasks.(name)));
                    rejected=rejected+nnz(~audit.accepted);
                end
            end
            testCase.verifyGreaterThan(rejected,0,'The boundary check must include rejected fine points.');
        end

        function fineCannotRestoreExplicitlyRemovedCoarseCandidates(testCase)
            file=fullfile(testCase.Root,'data','raw','downTownPointClouds.mat');
            testCase.assumeTrue(isfile(file));
            frame=loadPointCloudFrame(file,75);
            cfg=perceptionConfig('Downtown','offline');
            cfg.coarseProbabilityCloud.storeDiagnostics=true;
            result=perceiveFrame(frame,cfg);
            context=struct('voxelGrid',pillarizePointCloud(frame,cfg.voxel), ...
                'groundContext',result.diagnostics.groundPointContext, ...
                'offGroundVoxelGrid',result.diagnostics.offGroundPointContext, ...
                'ground',result.diagnostics.ground,'offGround',result.diagnostics.offGround);
            candidates=result.candidates;
            for k=1:numel(candidates.pillarIndices)
                candidates.pillarIndices{k}=zeros(0,1,'int32');
            end
            % The supplied context intentionally retains the old positive
            % branch masks. Only the explicit candidate object is authoritative.
            fine=refinePerceptionCandidates(frame,candidates,context,cfg);
            for k=1:numel(candidates.semanticNames)
                name=candidates.semanticNames(k);
                testCase.verifyFalse(any(fine.featureMasks.(name)));
                testCase.verifyEmpty(fine.refinement.(name).evaluatedPointIndices);
                testCase.verifyEmpty(fine.candidates.pillarIndices{k});
            end
        end

        function descriptorsAndWholeMomentsArePermutationInvariant(testCase)
            frame=syntheticFrame();gridCfg=pillarGridConfig();gridCfg.exclusionHalfSize=0;
            cfg=perceptionConfig('Downtown');cfg.offGroundFeatures.useNativeKernels=false;
            cloudCfg=coarseSemanticProbabilityCloudConfig(gridCfg);
            cloudCfg.semanticNames=["pole","facade","trafficSign"];
            grid=pillarizePointCloud(frame,gridCfg);
            first=analyzeDowntownPillarStatistics(grid,cfg.offGroundFeatures,cloudCfg);
            order=numel(frame.x):-1:1;permuted=frame;
            for field=fieldnames(frame).'
                values=frame.(field{1});permuted.(field{1})=values(order);
            end
            shuffled=pillarizePointCloud(permuted,gridCfg);
            second=analyzeDowntownPillarStatistics(shuffled,cfg.offGroundFeatures,cloudCfg);
            a=first.columnMaps.statistics;b=second.columnMaps.statistics;
            testCase.verifyEqual(a.pillarIndices,b.pillarIndices);
            testCase.verifyEqual(a.count,b.count);
            testCase.verifyEqual(a.meanXYZ,b.meanXYZ,'AbsTol',1e-11);
            testCase.verifyEqual(a.covarianceXYZ,b.covarianceXYZ,'AbsTol',1e-11);
            testCase.verifyEqual(a.minimumXYZ,b.minimumXYZ);
            testCase.verifyEqual(a.maximumXYZ,b.maximumXYZ);
            testCase.verifyEqual(a.intensity.maximum,b.intensity.maximum);
            testCase.verifyEqual(sum(first.columnMaps.moments.count),numel(grid.pointIndices));
            testCase.verifyEqual(first.columnMaps.moments.mean(double(a.pillarIndices),:), ...
                a.meanXYZ(:,1:2),'AbsTol',1e-11);
            testCase.verifyEqual(first.columnMaps.moments.covariance(double(a.pillarIndices),:), ...
                a.covarianceXYZ(:,1:3),'AbsTol',1e-11);
            for semantic=cloudCfg.semanticNames
                [x,names,ids]=measureDowntownPillarStatistics(struct(),first,semantic);
                [y,otherNames,otherIds]=measureDowntownPillarStatistics(struct(),second,semantic);
                testCase.verifyEqual(names,otherNames);testCase.verifyEqual(ids,otherIds);
                testCase.verifyEqual(ids,double(a.pillarIndices));
                testCase.verifyEqual(x,y,'AbsTol',1e-7);
                % A proposal mask cannot hide rows from model inference.
                second.(semantic+'CellMask')(:)=false;
                [allRows,allNames,allIds]=measureDowntownPillarStatistics(struct(),second,semantic);
                testCase.verifyEqual(allNames,names);testCase.verifyEqual(allIds,ids);
                testCase.verifyEqual(allRows,y);
            end
        end
    end
end

function auditCoarseExecution(testCase,frame,cfg)
    cleanup=onCleanup(@() profile('off'));
    profile clear;profile on;
    result=perceiveFrame(frame,cfg);
    profile off;record=profile('info');
    names=string({record.FunctionTable.FunctionName});
    forbidden=["detectFacadeSurfaces","completeFacadeSurfaces","refinePerceptionCandidates", ...
        "detectDowntownStructures","detectDowntownCurbs","validateDowntown", ...
        "recoverDowntownDensePoles","filterDowntownCurb","extendDowntownCurbEndpoints", ...
        "findPillarShaftModes","assignPillarShaftSupport","findPillarPoleSubsets", ...
        "validatePillarPoleSupport","classifyPillarPoleSupport","measurePillarPoleSupport", ...
        "computePillarDensityCore","computePillarVerticalShape","continuousPillarHeightSupport", ...
        "measureSemanticPointDistributions","measureFacadeGroupEvidence", ...
        "voxelizePillars","analyzeFineStructuralCandidates","pcfitplane", ...
        "evaluateSemanticPillarCandidates"];
    for name=forbidden
        testCase.verifyFalse(any(contains(names,name)),sprintf('Coarse execution called %s.',name));
    end
    testCase.verifyTrue(any(contains(names,'analyzeDowntownPillarStatistics')));
    testCase.verifyFalse(isfield(result,'refinement'));
    testCase.verifyFalse(isfield(result,'featureMasks'));
end

function frame=syntheticFrame()
    [gx,gy]=meshgrid(5:.5:12,-4:.5:4);
    ground=[gx(:),gy(:),-2+0.002*gx(:)];
    z=(-1.8:.08:4).';angle=(1:numel(z)).';
    pole=[8+0.04*cos(angle),1+0.04*sin(angle),z];
    [wx,wz]=meshgrid(6:.15:14,-1.8:.25:6);
    wall=[wx(:),ones(numel(wx),1)*8,wz(:)];
    sign=[10+(0:5).'*0.03,2+zeros(6,1),1+(0:5).'*0.02];
    xyz=[ground;pole;wall;sign];
    intensity=[ones(size(xyz,1)-size(sign,1),1)*100;ones(size(sign,1),1)*2200];
    frame=struct('x',xyz(:,1),'y',xyz(:,2),'z',xyz(:,3),'intensity',intensity);
end
