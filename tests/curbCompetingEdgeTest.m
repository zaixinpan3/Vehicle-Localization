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
            [rejected,protected]=rejectWeakerRaisedCurbEdges(xyz,ids,ones(size(ids)),g,cfg);
            testCase.verifyFalse(any(rejected(1:12)));
            testCase.verifyGreaterThanOrEqual(nnz(rejected(13:end)),6);
            testCase.verifyTrue(any(protected(1:12)));
            % One nearby strong point cannot suppress a measured edge.
            keep=[1,13:24];small=xyz(keep,:);
            actual=rejectWeakerRaisedCurbEdges(small,(1:13).',ones(13,1),g(keep,:),cfg);
            testCase.verifyFalse(any(actual));
            xyz(13:end,2)=0.2;
            actual=rejectWeakerRaisedCurbEdges(xyz,ids,ones(size(ids)),g,cfg);
            testCase.verifyFalse(any(actual));
        end
        function equalStrengthOrEqualHeightEdgesRemainSeparate(testCase)
            [xyz,g]=parallelEdges();cfg=finePerceptionConfig();ids=(1:size(xyz,1)).';
            xyz(:,3)=0;
            testCase.verifyFalse(any(rejectWeakerRaisedCurbEdges(xyz,ids,ones(size(ids)),g,cfg)));
            [xyz,g]=parallelEdges();g(13:end,:)=-g(1:12,:);
            testCase.verifyFalse(any(rejectWeakerRaisedCurbEdges(xyz,ids,ones(size(ids)),g,cfg)));
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
