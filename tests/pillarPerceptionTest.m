classdef pillarPerceptionTest < matlab.unittest.TestCase
% pillarPerceptionTest: Execution boundary, sparse storage and refinement contract.
    methods (TestClassSetup)
        function paths(~)
            run(fullfile(fileparts(fileparts(mfilename('fullpath'))),'setupVehicleLocalization.m'));
        end
    end
    methods (Test)
        function sparsePillarsNeverAllocateHeightVolume(testCase)
            cfg=frameVoxelizationConfig();
            cfg.roiLimits=[-30 30 -30 30 -1000 1000];
            cfg.maxRange=Inf;
            points=[10 2 0;10.01 2.01 50;11 3 5];
            pillars=pillarizePointCloud(points,cfg);
            testCase.verifyFalse(isfield(pillars,'pointVoxelSub'));
            testCase.verifySize(pillars.statistics.covarianceXYZ,[2 6]);
            testCase.verifyEqual(pillars.pointPillarLinIdx(1),pillars.pointPillarLinIdx(2));
            testCase.verifyEqual(pillars.statistics.count,[2;1]);
        end
        function momentAggregationUsesActualReturnLocations(testCase)
            moments=aggregatePlanarCellMoments([0 0;0.2 0.4;2 3],[1;1;2],3);
            testCase.verifyEqual(moments.mean(1,:),[0.1 0.2],'AbsTol',1e-12);
            testCase.verifyEqual(moments.covariance(1,:),[0.01 0.02 0.04],'AbsTol',1e-12);
            testCase.verifyEqual(moments.count,[2;1;0]);
        end
        function tiltProjectionMatchesFullOfflineTransform(testCase)
            angle=0.12; yaw=-0.4;
            tilt=[cos(angle) 0 sin(angle);0 1 0;-sin(angle) 0 cos(angle)];
            rotation=[cos(yaw) -sin(yaw) 0;sin(yaw) cos(yaw) 0;0 0 1];
            points=[10 2 -1;10.2 2.1 3;10.1 2.2 5];
            moments=aggregatePlanarCellMoments(points,ones(3,1),1,tilt);
            actualMean=moments.mean*rotation(1:2,1:2).';
            expected=points*(rotation*tilt).';
            testCase.verifyEqual(actualMean,mean(expected(:,1:2),1),'AbsTol',1e-12);
            covariance=[moments.covariance(1) moments.covariance(2);moments.covariance(2) moments.covariance(3)];
            testCase.verifyEqual(rotation(1:2,1:2)*covariance*rotation(1:2,1:2).', ...
                cov(expected(:,1:2),1),'AbsTol',1e-12);
        end
        function allRejectedCandidatesStayEmpty(testCase)
            frame=struct('x',[10;10.1;12],'y',[2;2.1;3],'z',[-1;-1;-1]);
            gc=struct('groundOriginalPointIdx',[1;2;3],'groundReflectivity',[4;5;100]);
            grid=struct('pointPillarLinIdx',[1;1;2],'pointIndices',[1;2;3]);
            context=struct('voxelGrid',grid,'ground',struct(),'groundContext',gc);
            candidates=struct('semanticNames',"roadMarking",'pillarIndices',{{1}},'groundReflectivityThreshold',10);
            fine=refinePerceptionCandidates(frame,candidates,context,perceptionConfig());
            testCase.verifyFalse(any(fine.featureMasks.roadMarking));
            testCase.verifyEqual(fine.refinement.roadMarking.evaluatedPointIndices,[1;2]);
            testCase.verifyEqual(fine.refinement.roadMarking.accepted,false(2,1));
        end
        function onlineAndOfflineShareCandidates(testCase)
            dataRoot=string(getenv('VEHICLE_LOCALIZATION_DATA_ROOT'));
            testCase.assumeTrue(isfile(fullfile(dataRoot,'raw','MissisipiPointClouds.mat')));
            frame=loadPointCloudFrame(fullfile(dataRoot,'raw','MissisipiPointClouds.mat'),260);
            cfg=perceptionConfig();
            online=perceiveFrame(frame,cfg);
            testCase.verifyEqual(online.executionMode,"coarseProbabilityCloud");
            testCase.verifyFalse(isfield(online,'featureMasks'));
            testCase.verifyFalse(isfield(online,'voxelGrid'));
            testCase.verifyEqual(online.candidates.productType,"sparseSemanticPillarCandidates");
            cfg.executionMode="offline";
            offline=perceiveFrame(frame,cfg);
            testCase.verifyEqual(offline.candidates,online.candidates);
            testCase.verifyEqual(offline.probabilityCloud.components,online.probabilityCloud.components);
            for name=["curb","roadMarking","pole"]
                audit=offline.refinement.(name);
                testCase.verifyEqual(audit.evaluatedPointIndices,audit.candidatePointIndices);
                testCase.verifyEqual(find(offline.featureMasks.(name)), ...
                    sort(audit.candidatePointIndices(audit.accepted)));
                testCase.verifyGreaterThan(audit.numEvaluated,0);
            end
        end
        function recordedPointFidelityPreservesBaseline(testCase)
            dataRoot=string(getenv('VEHICLE_LOCALIZATION_DATA_ROOT'));
            matPath=fullfile(dataRoot,'raw','MissisipiPointClouds.mat');
            testCase.assumeTrue(isfile(matPath));
            legacyCfg=perceptionConfig(); legacyCfg.executionMode="legacyFull";
            cfg=perceptionConfig(); cfg.executionMode="offline";
            counts=zeros(1,3);
            for frameIndex=[260 300 326 370 450 550 700 850 1000 1150 150 600 900 1100 200 500 800 1050]
                frame=loadPointCloudFrame(matPath,frameIndex);
                reference=perceiveFrame(frame,legacyCfg);
                actual=perceiveFrame(frame,cfg);
                for name=["curb" "roadMarking"]
                    testCase.verifyEqual(actual.featureMasks.(name),reference.featureMasks.(name), ...
                        sprintf('frame %d %s',frameIndex,name));
                end
                p=actual.featureMasks.pole; r=reference.featureMasks.pole;
                tp=nnz(p&r); fp=nnz(p&~r); fn=nnz(~p&r);
                counts=counts+[tp fp fn];
                if nnz(p|r)>0
                    testCase.verifyGreaterThanOrEqual(2*tp/(2*tp+fp+fn),0.80,sprintf('frame %d pole',frameIndex));
                end
            end
            testCase.verifyGreaterThanOrEqual(counts(1)/sum(counts([1 2])),0.95);
            testCase.verifyGreaterThanOrEqual(counts(1)/sum(counts([1 3])),0.95);
        end
    end
end
