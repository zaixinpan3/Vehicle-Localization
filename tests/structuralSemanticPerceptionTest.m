classdef structuralSemanticPerceptionTest < matlab.unittest.TestCase
% structuralSemanticPerceptionTest: Facade/sign coarse, fine, map and pose contracts.
    properties (TestParameter)
        dataset = struct('suburban',"Missisipi",'urban',"downTown")
        frameIndex = {100,200,300,400,500}
    end
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'tests')));
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function profilesKeepAllChannelsWithDatasetFacadeSwitch(testCase)
            suburban=perceptionConfig(); urban=perceptionConfig("Downtown");
            testCase.verifyFalse(any(suburban.featureNames=="facade"));
            testCase.verifyTrue(any(urban.featureNames=="facade"));
            testCase.verifyEqual(urban.featureNames, ...
                ["curb","roadMarking","pole","facade","trafficSign"]);
            mapCfg=featureMapBuildConfig();
            testCase.verifyTrue(any(mapCfg.featureNames=="trafficSign"));
        end
        function signGaussianRetainsNonreflectivePillarHeight(testCase)
            [frame,cfg,expected]=signScene();
            coarse=perceiveFrame(frame,cfg);
            cfg.executionMode="offline"; fine=perceiveFrame(frame,cfg);
            c=coarse.probabilityCloud.components;
            testCase.verifyFalse(isfield(coarse,'refinement'));
            testCase.verifyEqual(fine.candidates,coarse.candidates);
            testCase.verifyEqual(c.semanticName,"trafficSign");
            testCase.verifyEqual(double(c.count),size(expected,1));
            testCase.verifyEqual(c.meanXYZ,mean(expected,1),'AbsTol',1e-10);
            testCase.verifyGreaterThan(c.covarianceXYZ(3,3),1);
            testCase.verifyEqual(nnz(fine.featureMasks.trafficSign),1);
            testCase.verifyGreaterThan(fine.refinement.trafficSign.numEvaluated,1);
        end
        function fineSignCandidatesHonorConfiguredIntensity(testCase)
            [frame,cfg,~]=signScene();
            frame.intensity(end)=1300;
            cfg.offGroundFeatures.trafficSignIntensityThreshold=1200;
            cfg.executionMode="offline";
            result=perceiveFrame(frame,cfg);
            testCase.verifyEqual(nnz(result.featureMasks.trafficSign),1);
            testCase.verifyTrue(result.featureMasks.trafficSign(end));
        end
        function signValidationNeverVisitsOutsideCandidates(testCase)
            frame=struct('x',[10;10.1;12],'y',[2;2.1;3],'z',[1;2;3], ...
                'intensity',[1601;100;9000]);
            grid=struct('pointPillarLinIdx',[1;1;2],'pointIndices',[1;2;3]);
            gc=struct('groundOriginalPointIdx',zeros(0,1));
            context=struct('voxelGrid',grid,'ground',struct(),'groundContext',gc);
            candidates=struct('semanticNames',"trafficSign",'pillarIndices',{{1}});
            fine=refinePerceptionCandidates(frame,candidates,context,perceptionConfig());
            testCase.verifyEqual(fine.featureMasks.trafficSign,[true;false;false]);
            testCase.verifyEqual(fine.refinement.trafficSign.evaluatedPointIndices,[1;2]);
        end
        function facadeDistanceTestRejectsPointsSharingAcceptedPillars(testCase)
            [points,offGround,n]=facadeScene();
            accepted=validateFacadeCandidatePoints(points,offGround,finePerceptionConfig());
            testCase.verifyTrue(all(accepted(1:n)));
            testCase.verifyFalse(any(accepted(n+1:end)));
        end
        function failedFacadeValidationStaysEmpty(testCase)
            [points,offGround,~]=facadeScene();
            points(:,3)=0.1*points(:,3);
            accepted=validateFacadeCandidatePoints(points,offGround,finePerceptionConfig());
            testCase.verifyFalse(any(accepted));
        end
        function recordedSignsMatchReferenceAndAllFinePointsHaveCandidates(testCase,dataset,frameIndex)
            file=fullfile(fileparts(fileparts(mfilename('fullpath'))),'data','raw',dataset+'PointClouds.mat');
            testCase.assumeTrue(isfile(file));
            frame=loadPointCloudFrame(file,frameIndex);
            cfg=perceptionConfig(dataset); cfg.executionMode="offline";
            fine=perceiveFrame(frame,cfg);
            reference=loadPerceptionMaskReference(dataset,frameIndex);
            audit=fine.refinement.trafficSign;
            testCase.verifyEqual(fine.featureMasks.trafficSign,reference.featureMasks.trafficSign);
            testCase.verifyEqual(audit.evaluatedPointIndices,audit.candidatePointIndices);
            testCase.verifyEqual(find(fine.featureMasks.trafficSign),sort(audit.candidatePointIndices(audit.accepted)));
            testCase.verifyTrue(all(ismember(find(reference.featureMasks.facade),facadeAudit(fine).candidatePointIndices)));
            testCase.verifyEqual(find(fine.featureMasks.facade), ...
                sort(facadeAudit(fine).candidatePointIndices(facadeAudit(fine).accepted)));
        end
        function disabledChannelsStayEmptyInBothProducts(testCase)
            [frame,cfg,~]=signScene(); cfg.featureNames=["curb","pole"];
            cfg.executionMode="offline";
            p=perceiveFrame(frame,cfg);
            testCase.verifyFalse(any(p.featureMasks.trafficSign|p.featureMasks.facade));
            testCase.verifyFalse(any(ismember(p.probabilityCloud.components.semanticName,["facade","trafficSign"])));
            testCase.verifyFalse(isfield(p.refinement,'trafficSign'));
        end
        function nativeAndMatlabDowntownOutputsAgree(testCase)
            testCase.assumeTrue(perceptionNativeAvailable);
            file=fullfile(fileparts(fileparts(mfilename('fullpath'))),'data','raw','downTownPointClouds.mat');
            testCase.assumeTrue(isfile(file));
            frame=loadPointCloudFrame(file,200);
            cfg=perceptionConfig("Downtown"); cfg.executionMode="offline"; cfg.executionBackend="matlab";
            reference=perceiveFrame(frame,cfg); cfg.executionBackend="native";
            actual=perceiveFrame(frame,cfg);
            testCase.verifyEqual(actual.candidates,reference.candidates);
            testCase.verifyEqual(actual.featureMasks,reference.featureMasks);
            testCase.verifyEqual(actual.probabilityCloud.components.meanXYZ,reference.probabilityCloud.components.meanXYZ,'AbsTol',1e-12);
        end
        function facadeDoesNotInventAlongWallObservability(testCase)
            cloud=geometricRegistrationTest.parallelRoad();
            cloud.components.semanticName(:)="facade";
            result=registerSemanticProbabilityCloud(cloud,cloud,[0.8 0.3 0.02],distributionRegistrationConfig());
            testCase.verifyEqual(result.observableRank,2);
            testCase.verifyFalse(result.accepted);
            testCase.verifyEqual(result.poseXYTheta,[0.8 0 0],'AbsTol',1e-4);
        end
        function signsConstrainThreePlanarStates(testCase)
            moving=distributionRegistrationTest.exampleCloud();
            moving.components.semanticName(:)="trafficSign";
            expected=[1.2 -0.7 0.16];
            fixed=distributionRegistrationTest.transform(moving,expected);
            result=registerSemanticProbabilityCloud(fixed,moving,[1 -0.6 0.12],distributionRegistrationConfig());
            testCase.verifyTrue(result.accepted,result.reason);
            testCase.verifyEqual(result.poseXYTheta,expected,'AbsTol',1e-4);
            testCase.verifySize(result.poseXYTheta,[1 3]);
        end
        function mappingEntryUsesTheRequestedFeatureNames(testCase)
            file=fullfile(fileparts(fileparts(mfilename('fullpath'))),'data','raw','downTownPointClouds.mat');
            testCase.assumeTrue(isfile(file));
            folder=testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture());
            cfg=writeMappingFixture(folder.Folder,file);
            [map,features]=buildFeatureMap(folder.Folder,cfg);
            cloud=temporalMapToProbabilityCloud(map);
            testCase.verifyTrue(all(ismember(["facade","trafficSign"],features.featureNames)));
            testCase.verifyTrue(all(ismember(["facade","trafficSign"],unique(cloud.components.semanticName))));
            testCase.verifyGreaterThan(features.counts(:,features.featureNames=="facade"),0);
            testCase.verifyGreaterThan(features.counts(:,features.featureNames=="trafficSign"),0);
        end
        function repeatedObservationMapExportsBothStructuralClassesAndHeight(testCase)
            [points,labels,frames,cfg]=structuralMapScene();
            map=buildTemporalStabilityGmmMap(points,labels,frames,cfg);
            cloud=temporalMapToProbabilityCloud(map);
            testCase.verifyEqual(sort(unique(cloud.components.semanticName)),["facade";"trafficSign"]);
            testCase.verifyTrue(all(cloud.components.heightAvailable));
            testCase.verifySize(cloud.components.meanXYZ,[cloud.components.numComponents 3]);
            testCase.verifyTrue(all(isfinite(queryTemporalStabilityGmmMap(map,[1 0],"facade"))));
        end
    end
end

function [frame,cfg,expected]=signScene()
    [x,y]=meshgrid(-15:0.3:15,-10:0.3:10);
    ground=[x(:),y(:),-1.44*ones(numel(x),1)];
    expected=[10.06 5.06 0.5;10.08 5.08 1.5;10.09 5.09 3.5;10.1 5.1 4.5];
    xyz=[ground;expected]; intensity=zeros(size(xyz,1),1); intensity(end)=1700;
    frame=struct('x',xyz(:,1),'y',xyz(:,2),'z',xyz(:,3),'intensity',intensity);
    cfg=perceptionConfig(); cfg.featureNames="trafficSign";
    cfg.frameCalibration.rotation=eye(3); cfg.frameCalibration.translation=[0 0 0];
end

function [points,offGround,n]=facadeScene()
    [x,z]=meshgrid(0.1:0.1:4,0.1:0.1:3);
    plane=[x(:),5.01*ones(numel(x),1),z(:)]; n=size(plane,1);
    points=[plane;plane(1:20,:)+[0 0.26 0]];
    maps=struct('origin',[0 0],'dx',0.3,'dy',0.3,'mapSize',[20 15]);
    facade=struct('lineMap',ones(maps.mapSize,'uint16'),'detectedLines',[0 5 4 5]);
    offGround=struct('columnMaps',maps,'facade',facade);
end

function [points,labels,frames,cfg]=structuralMapScene()
    x=(0:0.1:3).'; wall=[x,zeros(size(x)),2+0.2*x];
    sign=[1 2 3;1.1 2 3.2;1.2 2 3.4;1.3 2 3.6];
    observation=[wall;sign]; names=[repmat("facade",size(wall,1),1);repmat("trafficSign",size(sign,1),1)];
    points=repmat(observation,3,1); labels=repmat(names,3,1);
    frames=repelem((1:3).',size(observation,1));
    cfg=temporalStabilityMapConfig(); cfg.classes=["facade";"trafficSign"];
end

function cfg=writeMappingFixture(folder,file)
% Repeated copies and identity poses test plumbing, not real temporal stability.
    frame=loadPointCloudFrame(file,200);
    pointClouds=repmat(frame,1,3);
    save(fullfile(folder,'pointClouds.mat'),'pointClouds','-v7.3');
    poses=table((1:3).',zeros(3,1),zeros(3,1),zeros(3,1),90*ones(3,1),zeros(3,1),zeros(3,1), ...
        'VariableNames',{'frame_index','odom_x_m','odom_y_m','odom_z_m','azimuth_deg','roll_deg','pitch_deg'});
    writetable(poses,fullfile(folder,'poses.csv'));
    cfg=featureMapBuildConfig(); cfg.pointCloudMatPath="pointClouds.mat";
    cfg.poseMatchCsvPath="poses.csv"; cfg.mapOutputPath="";
    cfg.frameIndices=1:3; cfg.batchFrameCount=3; cfg.batchFrameStride=3;
    cfg.logEnabled=false;
    perception=perceptionConfig("Downtown"); cfg.featureNames=perception.featureNames;
end

function audit=facadeAudit(perception)
    audit=struct('candidatePointIndices',zeros(0,1),'accepted',false(0,1));
    if isfield(perception.refinement,'facade'), audit=perception.refinement.facade; end
end
