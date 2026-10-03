classdef facadeSurfaceTest < matlab.unittest.TestCase
% facadeSurfaceTest: Connected wall validation and semantic isolation.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function recoversConnectedWallWithoutAddingDetachedOrOffsetObjects(testCase)
            [xyz,columns,seeds,eligible,lines,group,cfg]=wallScene();
            ids=completeFacadeSurfaces(xyz,columns,seeds,eligible,lines,cfg);
            testCase.verifyTrue(all(ids(group<=2 & eligible)>0));
            testCase.verifyFalse(any(ids(group>=3)>0));
            testCase.verifyFalse(any(ids(~eligible)>0));
        end
        function rejectsUnverifiedOriginalSeeds(testCase)
            [xyz,columns,seeds,eligible,lines,group,cfg]=wallScene();
            seeds(group>=3)=1;
            ids=completeFacadeSurfaces(xyz,columns,seeds,eligible,lines,cfg);
            testCase.verifyTrue(all(ids(group<=2 & eligible)>0));
            testCase.verifyFalse(any(ids(group>=3)>0));
        end
        function noAnchorsCannotBootstrapAPlane(testCase)
            [xyz,columns,seeds,eligible,lines,~,cfg]=wallScene();
            ids=completeFacadeSurfaces(xyz,columns,zeros(size(seeds),'uint16'),eligible,lines,cfg);
            testCase.verifyFalse(any(ids));
        end
        function everyOutputPointHasABoundedPlaneAndCandidate(testCase)
            [xyz,columns,seeds,eligible,lines,~,cfg]=wallScene();
            [ids,detail]=completeFacadeSurfaces(xyz,columns,seeds,eligible,lines,cfg);
            selected=ids>0; modelIds=detail.pointModelIds(selected);
            models=detail.allModels(modelIds,:);
            residual=abs(sum(xyz(selected,:).*models(:,1:3),2)+models(:,4));
            testCase.verifyGreaterThan(modelIds,zeros(size(modelIds),'uint32'));
            testCase.verifyLessThanOrEqual(residual,detail.allModelMaxDistances(modelIds)+1e-12);
            testCase.verifyTrue(all(detail.candidateMask(selected)));
        end
        function deterministicFitsPreserveCallerRandomState(testCase)
            [xyz,columns,seeds,eligible,lines,~,cfg]=wallScene();
            state=rng;
            a=completeFacadeSurfaces(xyz,columns,seeds,eligible,lines,cfg);
            b=completeFacadeSurfaces(xyz,columns,seeds,eligible,lines,cfg);
            testCase.verifyEqual(rng,state);
            testCase.verifyEqual(a,b);
        end
        function emptyAndRadiometricOnlyInputsHaveNoFacade(testCase)
            cfg=pillarGridConfig();cfg.exclusionHalfSize=0;
            empty=pillarizePointCloud(zeros(0,3),cfg);
            a=detectFacadeSurfaces(empty,0,facadeSurfaceConfig(.6));
            frame=struct('x',[10;10.1],'y',[5;5.1],'z',[1;4],'intensity',[2000;2000]);
            b=detectFacadeSurfaces(pillarizePointCloud(frame,cfg),1,facadeSurfaceConfig(.6));
            testCase.verifyFalse(any(a.mask,'all'));
            testCase.verifyEmpty(a.pointLineIds);
            testCase.verifyFalse(any(b.mask,'all'));
        end
        function profileLeavesOtherDatasetsOnTheirExistingAlgorithms(testCase)
            urban=perceptionConfig('Downtown');suburban=perceptionConfig('Mississippi');carla=perceptionConfig('Carla');
            testCase.verifyTrue(isfield(urban,'facadeSurface'));
            testCase.verifyFalse(any(urban.semanticPrecision.classes=="facade"));
            testCase.verifyFalse(isfield(suburban,'facadeSurface'));
            testCase.verifyFalse(isfield(carla,'facadeSurface'));
        end
        function onlineMomentsRetainEveryMemberOfAcceptedPillars(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            file=fullfile(root,'data/raw/downTownPointClouds.mat');
            testCase.assumeTrue(isfile(file));
            frame=loadPointCloudFrame(file,200);cfg=perceptionConfig('Downtown');
            cfg.featureNames="facade";cfg.coarseProbabilityCloud.storeDiagnostics=true;
            result=perceiveFrame(frame,cfg);mask=result.diagnostics.offGround.facadeCellMask;
            pillars=result.diagnostics.offGroundPointContext;
            count=nnz(mask(pillars.pointPillarLinIdx));
            testCase.verifyEqual(result.probabilityCloud.sourceSummary.selectedHitCount,count);
            testCase.verifyFalse(isfield(result.diagnostics.offGround.facade,'pointLineIds'));
            testCase.verifyFalse(isfield(result,'featureMasks'));
            testCase.verifyEqual(result.candidates.geometry.cellSize,[.6 .6],'AbsTol',1e-12);
            testCase.verifyEqual(result.probabilityCloud.classificationStage,"pillarOnlyCoarseValidation");
        end
        function mappingEntryHonorsTheFacadeProfile(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            file=fullfile(root,'data/raw/downTownPointClouds.mat');testCase.assumeTrue(isfile(file));
            frame=loadPointCloudFrame(file,200);
            folder=testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture());
            cfg=mappingScene(folder.Folder,frame);
            [map,features]=buildFeatureMap(folder.Folder,cfg);
            perceptionCfg=perceptionConfig('Downtown','offline');perceptionCfg.featureNames="facade";
            expected=perceiveFrame(frame,perceptionCfg);
            testCase.verifyEqual(features.counts,repmat(nnz(expected.featureMasks.facade),3,1));
            testCase.verifyEqual(map.perceptionProfile,"Downtown");
        end
    end
end

function [xyz,columns,seeds,eligible,lines,group,cfg]=wallScene()
% A tall anchored wall, its sparse extension, and two unrelated objects.
    [x,z]=ndgrid(0:.2:2.4,0:.2:4);anchor=[x(:),10*ones(numel(x),1),z(:)];
    [x,z]=ndgrid(2.6:.2:8,3:.2:4);wall=[x(:),10*ones(numel(x),1),z(:)];
    [x,z]=ndgrid(11:.2:14,0:.2:1);short=[x(:),10*ones(numel(x),1),z(:)];
    offset=wall+[0 .25 0];xyz=[anchor;wall;short;offset];
    group=repelem((1:4).',[size(anchor,1),size(wall,1),size(short,1),size(offset,1)]);
    seeds=uint16(group==1);eligible=true(size(group));eligible(find(group==2,1))=false;
    [~,~,columns]=unique(floor(xyz(:,1:2)/.3),'rows');lines=[0 10 14 10];
    cfg=facadeSurfaceConfig(.3).completion;
end

function cfg=mappingScene(folder,frame)
    pointClouds=repmat(frame,1,3);save(fullfile(folder,'points.mat'),'pointClouds','-v7.3');
    poses=table((1:3).',zeros(3,1),zeros(3,1),zeros(3,1),90*ones(3,1),zeros(3,1),zeros(3,1), ...
        'VariableNames',{'frame_index','odom_x_m','odom_y_m','odom_z_m','azimuth_deg','roll_deg','pitch_deg'});
    writetable(poses,fullfile(folder,'poses.csv'));
    cfg=featureMapBuildConfig();cfg.pointCloudMatPath="points.mat";cfg.poseMatchCsvPath="poses.csv";
    cfg.mapOutputPath="";cfg.frameIndices=1:3;cfg.batchFrameCount=3;cfg.batchFrameStride=3;
    cfg.logEnabled=false;cfg.featureNames="facade";cfg.perceptionProfile="Downtown";
    cfg.frameCalibration=lidarFrameCalibrationConfig('Downtown');
end
