classdef curbRoadSupportTest < matlab.unittest.TestCase
% curbRoadSupportTest: Distinguish road-adjacent steps from rough raised edges.
    methods (TestClassSetup)
        function paths(~)
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function preservesFlatRoadBesideRoughUpperSurface(testCase)
            [xyz,ground,boundaries]=singleStep(false);cfg=finePerceptionConfig();
            actual=validateCurbRoadSupport(xyz,ground,xyCloud(ground),boundaries,cfg);
            testCase.verifyEqual(actual,boundaries);
        end
        function rejectsRoughLowerSurface(testCase)
            [xyz,ground,boundaries]=singleStep(true);cfg=finePerceptionConfig();
            actual=validateCurbRoadSupport(xyz,ground,xyCloud(ground),boundaries,cfg);
            testCase.verifyEmpty(actual);
        end
        function rejectsRaisedParallelBoundaryWithWeakerRoadSupport(testCase)
            [xyz,ground,boundaries]=parallelSteps(.04);cfg=finePerceptionConfig();
            actual=validateCurbRoadSupport(xyz,ground,xyCloud(ground),boundaries,cfg);
            testCase.verifyEqual(actual,boundaries(1));
        end
        function preservesEquallyCleanRoadSurfaces(testCase)
            [xyz,ground,boundaries]=parallelSteps(0);cfg=finePerceptionConfig();
            actual=validateCurbRoadSupport(xyz,ground,xyCloud(ground),boundaries,cfg);
            testCase.verifyEqual(actual,boundaries);
        end
        function preservesInconclusiveSparseSupport(testCase)
            xyz=[0 0 0;.2 0 0;.4 0 0];ground=[0 -.2 0;0 .2 .15];boundaries={(1:3).'};
            testCase.verifyEqual(validateCurbRoadSupport(xyz,ground,xyCloud(ground),boundaries,finePerceptionConfig()),boundaries);
        end
        function isInvariantToHorizontalRotationAndTranslation(testCase)
            [xyz,ground,boundaries]=parallelSteps(.04);cfg=finePerceptionConfig();
            angle=.73;rotation=[cos(angle) -sin(angle) 0;sin(angle) cos(angle) 0;0 0 1];
            xyz=xyz*rotation+[7 -11 .8];ground=ground*rotation+[7 -11 .8];
            testCase.verifyEqual(validateCurbRoadSupport(xyz,ground,xyCloud(ground),boundaries,cfg),boundaries(1));
        end
        function rejectsReportedClutterAndPreservesRightBoundary943(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            file=fullfile(root,'data','raw','MissisipiPointClouds.mat');testCase.assumeTrue(isfile(file));
            annotation=jsondecode(fileread(fullfile(root,'tests','reference','curbBoundaryVicinity943.json')));
            frame=loadPointCloudFrame(file,943);cfg=perceptionConfig();cfg.executionMode="offline";
            previous=cfg;previous.fine.curbMaximumRoadSurfaceResidualMeters=Inf;
            previous.fine.curbCompetingBoundaryRoadResidualRatio=Inf;
            before=perceiveFrame(frame,previous);actual=perceiveFrame(frame,cfg);
            xyz=double([frame.x(:),frame.y(:),frame.z(:)]);near=xyz(annotation.rightBoundaryVicinityIndices,:);
            expected=before.featureMasks.curb & xyz(:,2)>=min(near(:,2))-.08 & ...
                xyz(:,2)<=max(near(:,2))+.08 & xyz(:,1)>=min(near(:,1))-.4 & xyz(:,1)<=max(near(:,1))+.4;
            testCase.verifyEqual(actual.featureMasks.curb,expected);
            testCase.verifyEqual(nnz(expected),48);
            testCase.verifyFalse(any(actual.featureMasks.curb(xyz(:,2)>0)));
            testCase.verifyGreaterThan(nnz(before.featureMasks.curb(xyz(:,2)>0)),50);
            testCase.verifyEqual(rmfield(actual.featureMasks,'curb'),rmfield(before.featureMasks,'curb'));
            testCase.verifyEqual(actual.probabilityCloud,before.probabilityCloud);
            stream=RandStream('mt19937ar','Seed',94335047);order=randperm(stream,numel(frame.x));
            shuffled=struct('x',frame.x(order).','y',frame.y(order).','z',frame.z(order).');
            cfg.featureNames="curb";reordered=perceiveFrame(shuffled,cfg);
            restored=false(numel(order),1);restored(order)=reordered.featureMasks.curb;
            testCase.verifyEqual(restored,actual.featureMasks.curb);
        end
    end
end

function cloud=xyCloud(ground)
    cloud=pointCloud([ground(:,1:2),zeros(size(ground,1),1)]);
end

function [xyz,ground,boundaries]=singleStep(roughLower)
    [x,y]=meshgrid(-1:.05:4,-1:.05:1);z=.15*(y>0);
    noise=.12*sin(19*x).*cos(17*y);
    z(y>0)=z(y>0)+noise(y>0);
    if roughLower,z(y<0)=z(y<0)+noise(y<0);end
    ground=[x(:),y(:),z(:)];along=(0:.15:3).';
    xyz=[along,zeros(size(along)),.075*ones(size(along))];boundaries={(1:numel(along)).'};
end

function [xyz,ground,boundaries]=parallelSteps(noiseAmplitude)
    [x,y]=meshgrid(-1:.05:4,-1:.05:1.8);z=.15*(y>0)+.10*(y>.8);
    middle=y>0 & y<.8;noise=noiseAmplitude*sin(19*x).*cos(17*y);z(middle)=z(middle)+noise(middle);
    ground=[x(:),y(:),z(:)];along=(0:.15:3).';n=numel(along);
    xyz=[along,zeros(n,1),.075*ones(n,1);along,.8*ones(n,1),.20*ones(n,1)];
    boundaries={(1:n).',(n+1:2*n).'};
end
