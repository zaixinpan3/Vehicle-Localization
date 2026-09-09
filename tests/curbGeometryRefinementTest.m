classdef curbGeometryRefinementTest < matlab.unittest.TestCase
% curbGeometryRefinementTest: Unorganized geometry, invariance, and curb width.
    methods (TestClassSetup)
        function paths(~)
            run(fullfile(fileparts(fileparts(mfilename('fullpath'))),'setupVehicleLocalization.m'));
        end
    end
    methods (Test)
        function broadFacesBecomeNarrowOriginalPointBoundaries(testCase)
            points=curbScene([-3 3]);
            [mask,detail]=refineCurbGeometry(points,(1:size(points,1)).',true(size(points,1),1),finePerceptionConfig());
            testCase.verifyGreaterThan(nnz(mask),30);
            testCase.verifyLessThan(nnz(mask),150);
            testCase.verifyLessThan(max(abs(abs(points(mask,2))-3)),0.10);
            testCase.verifyEqual(sort(vertcat(detail.boundaryPointIndices{:})),find(mask));
        end
        function pointOrderAndCandidateOrderDoNotAffectGeometry(testCase)
            points=curbScene([-3 3]);ids=(1:size(points,1)).';cfg=finePerceptionConfig();
            reference=refineCurbGeometry(points,ids,true(size(ids)),cfg);
            stream=RandStream('mt19937ar','Seed',425);
            permutation=randperm(stream,numel(ids));candidateOrder=randperm(stream,numel(ids));
            actual=refineCurbGeometry(points(permutation,:),candidateOrder.',true(size(ids)),cfg);
            restored=false(size(ids));restored(permutation(candidateOrder))=actual;
            testCase.verifyEqual(restored,reference);
        end
        function flatAndSlopedPlanesDoNotReceiveForcedPoints(testCase)
            points=curbScene([-3 3]);points(:,3)=-2+0.15*points(:,1)+0.3*points(:,2);
            mask=refineCurbGeometry(points,(1:size(points,1)).',true(size(points,1),1),finePerceptionConfig());
            testCase.verifyFalse(any(mask));
        end
        function terrainGradeDoesNotBecomeCurbHeight(testCase)
            points=curbScene(3);points(:,3)=points(:,3)+0.25*points(:,1);
            mask=refineCurbGeometry(points,(1:size(points,1)).',true(size(points,1),1),finePerceptionConfig());
            testCase.verifyGreaterThan(nnz(mask),30);
            testCase.verifyGreaterThan(max(points(mask,1))-min(points(mask,1)),6);
            testCase.verifyLessThan(max(abs(points(mask,2)-3)),0.10);
        end
        function tallStepDoesNotQualifyAsCurb(testCase)
            points=curbScene(3);points(:,3)=5*(points(:,3)-0.015*points(:,1))+0.25*points(:,1);
            mask=refineCurbGeometry(points,(1:size(points,1)).',true(size(points,1),1),finePerceptionConfig());
            testCase.verifyFalse(any(mask));
        end
        function oppositeFacingMedianEdgesRemainSeparate(testCase)
            points=curbScene([-0.4 0.4]);points(:,3)=-points(:,3);
            [mask,detail]=refineCurbGeometry(points,(1:size(points,1)).',true(size(points,1),1),finePerceptionConfig());
            testCase.verifyGreaterThan(nnz(mask & points(:,2)>0),15);
            testCase.verifyGreaterThan(nnz(mask & points(:,2)<0),15);
            testCase.verifyEqual(numel(detail.boundaryPointIndices),2);
        end
        function rigidXYTransformPreservesBoundaryLocation(testCase)
            points=curbScene([-3 3]);angle=0.6;R=[cos(angle),-sin(angle);sin(angle),cos(angle)];
            points(:,1:2)=points(:,1:2)*R.'+[8 -4];
            mask=refineCurbGeometry(points,(1:size(points,1)).',true(size(points,1),1),finePerceptionConfig());
            original=(points(mask,1:2)-[8 -4])*R;
            testCase.verifyGreaterThan(nnz(mask),30);
            testCase.verifyLessThan(max(abs(abs(original(:,2))-3)),0.10);
        end
        function disconnectedFragmentsDoNotCreateALongBoundary(testCase)
            points=curbScene(3);points=points(abs(points(:,1))<0.6,:);
            points=[points;points+[20 0 0]];
            mask=refineCurbGeometry(points,(1:size(points,1)).',true(size(points,1),1),finePerceptionConfig());
            testCase.verifyFalse(any(mask));
        end
        function curvedBoundaryFollowsMeasuredGeometry(testCase)
            points=curbScene(3);points(:,2)=points(:,2)+0.025*points(:,1).^2;
            mask=refineCurbGeometry(points,(1:size(points,1)).',true(size(points,1),1),finePerceptionConfig());
            testCase.verifyGreaterThan(nnz(mask),20);
            testCase.verifyGreaterThan(max(points(mask,1))-min(points(mask,1)),5);
            testCase.verifyLessThan(max(abs(points(mask,2)-0.025*points(mask,1).^2-3)),0.10);
        end
        function recordedCloudAcceptsFlatteningAndPermutation(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            path=fullfile(root,'data','raw','MissisipiPointClouds.mat');testCase.assumeTrue(isfile(path));
            frame=loadPointCloudFrame(path,425);cfg=perceptionConfig();cfg.executionMode="offline";
            reference=perceiveFrame(frame,cfg);count=numel(frame.x);
            stream=RandStream('mt19937ar','Seed',425);permutation=randperm(stream,count);
            unordered=frame;
            for name=string(fieldnames(frame)).'
                if numel(frame.(name))==count,v=frame.(name);unordered.(name)=reshape(v(permutation),[],1);end
            end
            actual=perceiveFrame(unordered,cfg);
            for name=string(fieldnames(reference.featureMasks)).'
                testCase.verifyEqual(actual.featureMasks.(name),reference.featureMasks.(name)(permutation));
            end
            testCase.verifyEqual(actual.candidates,reference.candidates);
        end
        function recordedBoundaryIsThinAndNearUserVicinity(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            matPath=fullfile(root,'data','raw','MissisipiPointClouds.mat');testCase.assumeTrue(isfile(matPath));
            annotation=jsondecode(fileread(fullfile(root,'tests','reference','curbBoundaryVicinity.json')));
            frame=loadPointCloudFrame(matPath,425);cfg=perceptionConfig();cfg.executionMode="offline";
            actual=perceiveFrame(frame,cfg);ids=find(actual.featureMasks.curb);
            testCase.verifyFalse(any(actual.featureMasks.curb(annotation.falsePositiveIndices)));
            for side=[1 -1]
                if side>0,anchors=annotation.leftBoundaryVicinityIndices;else,anchors=annotation.rightBoundaryVicinityIndices;end
                selected=ids(side*frame.y(ids)>0);
                % Ring counts are an output-density audit, never detector inputs.
                counts=accumarray(mod(selected-1,size(frame.x,1))+1,1,[size(frame.x,1),1]);
                testCase.verifyLessThanOrEqual(max(counts),5);
                testCase.verifyGreaterThanOrEqual(nnz(counts),25);
                within=frame.x(selected)>=min(frame.x(anchors)) & frame.x(selected)<=max(frame.x(anchors));
                boundary=polyfit(double(frame.x(anchors)),double(frame.y(anchors)),2);
                distance=abs(double(frame.y(selected(within)))-polyval(boundary,double(frame.x(selected(within)))));
                testCase.verifyLessThan(max(distance),0.15);
            end
            testCase.verifyLessThan(numel(ids),160);
        end
    end
end

function points=curbScene(boundaries)
% Unorganized samples of two surfaces and a curb face, in meters.
    points=zeros(0,3);
    for boundary=boundaries
        side=sign(boundary);
        [x,y]=ndgrid(-4:0.15:4,[-0.9:0.09:-0.09,0.09:0.09:0.9]);
        z=-2+0.15*(side*y>0)+0.015*x;
        [faceX,faceZ]=ndgrid(-4:0.15:4,-2:0.025:-1.85);
        points=[points;[x(:),boundary+y(:),z(:)]; ...
            [faceX(:),repmat(boundary,numel(faceX),1),faceZ(:)+0.015*faceX(:)]]; %#ok<AGROW>
    end
end
