classdef curbCompetingEdgeTest < matlab.unittest.TestCase
% curbCompetingEdgeTest: Preserve stronger low steps beside weak raised clutter.
    methods (TestClassSetup)
        function paths(~)
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function requiresSeveralSupportedCellsAndSeparation(testCase)
            [xyz,g]=parallelEdges();cfg=finePerceptionConfig();ids=(1:size(xyz,1)).';
            rejected=rejectWeakerRaisedCurbEdges(xyz,ids,ones(size(ids)),g,cfg);
            testCase.verifyFalse(any(rejected(1:12)));
            testCase.verifyGreaterThanOrEqual(nnz(rejected(13:end)),6);
            % One nearby strong point cannot suppress a measured edge.
            keep=[1,13:24];small=xyz(keep,:);
            actual=rejectWeakerRaisedCurbEdges(small,(1:13).',ones(13,1),g(keep,:),cfg);
            testCase.verifyFalse(any(actual));
            xyz(13:end,2)=0.2;
            actual=rejectWeakerRaisedCurbEdges(xyz,ids,ones(size(ids)),g,cfg);
            testCase.verifyFalse(any(actual));
        end
        function resolvesModestlyStrongerLowerStep(testCase)
            [xyz,g]=parallelEdges();g(13:end,2)=-0.27;
            cfg=finePerceptionConfig();cfg.curbCompetingEdgeGradientRatio=cfg.curbAlternativeEdgeGradientRatio;ids=(1:size(xyz,1)).';
            previous=cfg;previous.curbCompetingEdgeGradientRatio=1.4;
            testCase.verifyFalse(any(rejectWeakerRaisedCurbEdges(xyz,ids,ones(size(ids)),g,previous)));
            actual=rejectWeakerRaisedCurbEdges(xyz,ids,ones(size(ids)),g,cfg);
            testCase.verifyFalse(any(actual(1:12)));
            testCase.verifyGreaterThanOrEqual(nnz(actual(13:end)),6);
        end
        function recoversLeftBoundaryWithoutResamplingRightBoundary(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            file=fullfile(root,'data','raw','MissisipiPointClouds.mat');testCase.assumeTrue(isfile(file));
            annotation=jsondecode(fileread(fullfile(root,'tests','reference','curbBoundaryVicinity276.json')));
            frame=loadPointCloudFrame(file,276);cfg=perceptionConfig();cfg.executionMode="offline";
            previous=cfg;previous.fine.curbAlternativeEdgeGradientRatio=1.4;
            before=perceiveFrame(frame,previous);actual=perceiveFrame(frame,cfg);
            testCase.verifyTrue(all(before.featureMasks.curb(annotation.falsePositiveIndices)));
            testCase.verifyFalse(any(actual.featureMasks.curb(annotation.falsePositiveIndices)));
            xyz=double([frame.x(:),frame.y(:),frame.z(:)]);
            near=annotation.leftBoundaryVicinityIndices;ids=find(actual.featureMasks.curb);
            distance=min(hypot(xyz(near,1)-xyz(ids,1).',xyz(near,2)-xyz(ids,2).'),[],2);
            testCase.verifyLessThan(max(distance),0.13);
            testCase.verifyEqual(actual.featureMasks.curb(xyz(:,2)<0),before.featureMasks.curb(xyz(:,2)<0));
            testCase.verifyLessThan(nnz(actual.featureMasks.curb),180);
            testCase.verifyEqual(rmfield(actual.featureMasks,'curb'),rmfield(before.featureMasks,'curb'));
            testCase.verifyEqual(actual.probabilityCloud,before.probabilityCloud);
            stream=RandStream('mt19937ar','Seed',27625329);order=randperm(stream,numel(frame.x));
            shuffled=struct('x',frame.x(order).','y',frame.y(order).','z',frame.z(order).');
            cfg.featureNames="curb";reordered=perceiveFrame(shuffled,cfg);
            restored=false(numel(order),1);restored(order)=reordered.featureMasks.curb;
            testCase.verifyEqual(restored,actual.featureMasks.curb);
        end
        function equalStrengthOrEqualHeightEdgesRemainSeparate(testCase)
            [xyz,g]=parallelEdges();cfg=finePerceptionConfig();ids=(1:size(xyz,1)).';
            xyz(:,3)=0;
            testCase.verifyFalse(any(rejectWeakerRaisedCurbEdges(xyz,ids,ones(size(ids)),g,cfg)));
            [xyz,g]=parallelEdges();g(13:end,:)=-g(1:12,:);
            testCase.verifyFalse(any(rejectWeakerRaisedCurbEdges(xyz,ids,ones(size(ids)),g,cfg)));
        end
        function localRidgeFollowsBendWithoutJoiningSparseParallelPoints(testCase)
            [points,score,gradients]=ridgeScene();cfg=finePerceptionConfig();
            ids=(1:size(points,1)).';
            selected=traceCurbRidges(points,ids,score,gradients,true(size(ids)),.04*ones(size(ids)),cfg);
            testCase.verifyGreaterThan(nnz(selected),15);
            testCase.verifyFalse(any(selected(62:end)));
            testCase.verifyGreaterThan(max(points(selected,1))-min(points(selected,1)),5);
            testCase.verifyLessThan(max(abs(points(selected,2)-3-.04*points(selected,1).^2)),1e-12);
        end
        function ridgeSelectionIsIndependentOfInputOrder(testCase)
            [points,score,gradients]=ridgeScene();cfg=finePerceptionConfig();n=size(points,1);
            expected=traceCurbRidges(points,(1:n).',score,gradients,true(n,1),.04*ones(n,1),cfg);
            stream=RandStream('mt19937ar','Seed',83239295);order=randperm(stream,n);
            actual=traceCurbRidges(points(order,:),(1:n).',score(order),gradients(order,:),true(n,1),.04*ones(n,1),cfg);
            restored=false(n,1);restored(order)=actual;
            testCase.verifyEqual(restored,expected);
        end
        function followsReportedBendWithoutOuterShortcuts(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            file=fullfile(root,'data','raw','MissisipiPointClouds.mat');testCase.assumeTrue(isfile(file));
            annotation=jsondecode(fileread(fullfile(root,'tests','reference','curbBoundaryVicinity832.json')));
            frame=loadPointCloudFrame(file,832);cfg=perceptionConfig();cfg.executionMode="offline";
            actual=perceiveFrame(frame,cfg);xyz=double([frame.x(:),frame.y(:),frame.z(:)]);
            testCase.verifyFalse(any(actual.featureMasks.curb(annotation.falsePositiveIndices)));
            for picks={annotation.followupBoundaryVicinityIndices,annotation.nearEndBoundaryVicinityIndices}
                near=picks{1};
                distance=boundaryDistance(xyz(near,1:2),xyz,actual.refinement.curb.geometry.boundaryPointIndices);
                testCase.verifyLessThan(max(distance),0.06);
                testCase.verifyLessThan(median(distance),0.02);
                ids=find(actual.featureMasks.curb);
                nearest=min(hypot(xyz(near,1)-xyz(ids,1).',xyz(near,2)-xyz(ids,2).'),[],2);
                testCase.verifyLessThan(max(nearest),0.10);
            end
        end
        function recoversRecordedVicinityAndRejectsReportedPoints(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            file=fullfile(root,'data','raw','MissisipiPointClouds.mat');testCase.assumeTrue(isfile(file));
            annotation=jsondecode(fileread(fullfile(root,'tests','reference','curbBoundaryVicinity832.json')));
            frame=loadPointCloudFrame(file,832);cfg=perceptionConfig();cfg.executionMode="offline";
            actual=perceiveFrame(frame,cfg);ids=find(actual.featureMasks.curb);
            testCase.verifyFalse(any(actual.featureMasks.curb(annotation.falsePositiveIndices)));
            near=annotation.rightBoundaryVicinityIndices;
            distance=min(hypot(double(frame.x(near))-double(frame.x(ids)).', ...
                double(frame.y(near))-double(frame.y(ids)).'),[],2);
            testCase.verifyLessThan(max(distance),0.20);
            testCase.verifyLessThan(nnz(actual.featureMasks.curb),160);
            stream=RandStream('mt19937ar','Seed',83234801);order=randperm(stream,numel(frame.x));
            shuffled=frame;
            for name=string(fieldnames(frame)).'
                if numel(frame.(name))==numel(order),v=frame.(name);shuffled.(name)=reshape(v(order),[],1);end
            end
            reordered=perceiveFrame(shuffled,cfg);
            restored=false(numel(order),1);restored(order)=reordered.featureMasks.curb;
            testCase.verifyEqual(restored,actual.featureMasks.curb);
        end
    end
end

function [xyz,g]=parallelEdges()
    x=linspace(-.55,.55,12).';
    xyz=[x,zeros(12,2);x,.8*ones(12,1),.15*ones(12,1)];
    g=[zeros(12,1),.3*ones(12,1);zeros(12,1),-.1*ones(12,1)];
end

function distance=boundaryDistance(query,xyz,boundaries)
% Measure only interpolation between nearby, actually detected curb returns.
    distance=inf(size(query,1),1);
    for j=1:numel(boundaries)
        p=xyz(boundaries{j},1:2);
        [~,~,v]=svd(p-mean(p),0);[~,order]=sort(p*v(:,1));p=p(order,:);
        for k=1:size(p,1)-1
            direction=p(k+1,:)-p(k,:);lengthSquared=sum(direction.^2);
            if lengthSquared>1 || lengthSquared<=eps,continue;end
            t=max(0,min(1,(query-p(k,:))*direction.'/lengthSquared));
            distance=min(distance,vecnorm(query-p(k,:)-t.*direction,2,2));
        end
    end
end

function [points,score,gradients]=ridgeScene()
    x=(-3:.1:3).';curve=[x,3+.04*x.^2,-2*ones(size(x))];
    normal=[-.08*x,ones(size(x))];normal=normal./vecnorm(normal,2,2);
    face=curve+[.1*normal,zeros(size(x))];
    outerX=(-2:2).';outer=[outerX,3.6+.04*outerX.^2,-1.8*ones(size(outerX))];
    outerNormal=[-.08*outerX,ones(size(outerX))];outerNormal=outerNormal./vecnorm(outerNormal,2,2);
    points=[curve;face;outer];score=[ones(size(x));.8*ones(size(x));ones(size(outerX))];
    gradients=.3*[normal;normal;outerNormal];
end
