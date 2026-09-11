classdef curbGuidedExtensionTest < matlab.unittest.TestCase
% curbGuidedExtensionTest: Extend reviewed curbs from measured XYZ support.
    methods (TestClassSetup)
        function paths(~)
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function followsLocalStepBesideTallerTerrain(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            file=fullfile(root,'data','raw','MissisipiPointClouds.mat');
            testCase.assumeTrue(isfile(file),'Recorded source data are required.');
            frame=loadPointCloudFrame(file,1137);cfg=perceptionConfig();cfg.executionMode="offline";
            actual=perceiveFrame(frame,cfg);
            left=[27818 28203 28139 28075 28011 27947 27883 28268 27819 28204 28140 ...
                28076 28012 27948 26861 26797 26733 26669 26605 26990 26926 26862 ...
                26798 26734 27183 27119 27055 26991 26927 26863 27248 27184 27120 ...
                27056 25905 25841 25777 25649];
            right=[35177 35241 35305 35369 35690 35754 35818 35882 35946 36267 ...
                36331 36395 36716 36780 36844 36908 35757 36972 35821 37036 ...
                35885 37100 35949 36013 36334 36398];
            falsePoints=[36199 36263 36648 36776 35625 35881 36202 36266 36330 ...
                36458 36779 36971 37356];
            testCase.verifyGreaterThanOrEqual(nnz(actual.featureMasks.curb(left)),20);
            testCase.verifyGreaterThanOrEqual(nnz(actual.featureMasks.curb(right)),14);
            testCase.verifyFalse(any(actual.featureMasks.curb(falsePoints)));
            testCase.verifyEqual(nnz(actual.featureMasks.curb),142);
            stream=RandStream('mt19937ar','Seed',113727818);order=randperm(stream,numel(frame.x));
            shuffled=struct('x',frame.x(order).','y',frame.y(order).','z',frame.z(order).');
            cfg.featureNames="curb";reordered=perceiveFrame(shuffled,cfg);
            restored=false(numel(order),1);restored(order)=reordered.featureMasks.curb;
            testCase.verifyEqual(restored,actual.featureMasks.curb);
        end
        function followsReportedCurvedContinuation(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            file=fullfile(root,'data','raw','MissisipiPointClouds.mat');
            testCase.assumeTrue(isfile(file),'Recorded source data are required.');
            frame=loadPointCloudFrame(file,214);cfg=perceptionConfig();cfg.executionMode="offline";
            beforeCfg=cfg;beforeCfg.fine.curbContinuationLengthMeters=0;
            before=perceiveFrame(frame,beforeCfg);actual=perceiveFrame(frame,cfg);
            picks=[22962 22898 22834 22770 23219 23155 23091 23027 22963 22899 ...
                23476 23348 23284 23156 22069 22005 21877 21749];
            testCase.verifyFalse(any(before.featureMasks.curb(picks)));
            testCase.verifyTrue(all(actual.featureMasks.curb(before.featureMasks.curb)));
            testCase.verifyGreaterThanOrEqual(nnz(actual.featureMasks.curb(picks)),9);
            xyz=double([frame.x(:),frame.y(:),frame.z(:)]);ids=find(actual.featureMasks.curb);
            nearest=min(hypot(xyz(picks,1)-xyz(ids,1).',xyz(picks,2)-xyz(ids,2).'),[],2);
            testCase.verifyLessThan(max(nearest),0.20);
            testCase.verifyEqual(rmfield(actual.featureMasks,'curb'),rmfield(before.featureMasks,'curb'));
            testCase.verifyEqual(actual.probabilityCloud,before.probabilityCloud);
            stream=RandStream('mt19937ar','Seed',21422962);order=randperm(stream,numel(frame.x));
            shuffled=frame;
            for name=string(fieldnames(frame)).'
                if numel(frame.(name))==numel(order)
                    values=frame.(name);shuffled.(name)=reshape(values(order),[],1);
                end
            end
            reordered=perceiveFrame(shuffled,cfg);
            restored=false(numel(order),1);restored(order)=reordered.featureMasks.curb;
            testCase.verifyEqual(restored,actual.featureMasks.curb);
        end
        function rejectsReversedFalseBoundaryWithoutLosingTrueCurb(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            file=fullfile(root,'data','raw','MissisipiPointClouds.mat');
            testCase.assumeTrue(isfile(file),'Recorded source data are required.');
            frame=loadPointCloudFrame(file,963);cfg=perceptionConfig();cfg.executionMode="offline";
            % Isolate reversal rejection from the independent local competitor path.
            cfg.fine.curbAlternativeEdgeGradientRatio=cfg.fine.curbCompetingEdgeGradientRatio;
            beforeCfg=cfg;beforeCfg.fine.curbGradientReversalFraction=Inf;
            before=perceiveFrame(frame,beforeCfg);actual=perceiveFrame(frame,cfg);
            falsePoints=[36718 38776 37881 38840 38394 38971 39548 38525 39102 38461 39551 40256];
            testCase.verifyTrue(all(before.featureMasks.curb(falsePoints)));
            expected=before.featureMasks.curb;expected(falsePoints)=false;
            testCase.verifyEqual(actual.featureMasks.curb,expected);
            testCase.verifyEqual(nnz(actual.featureMasks.curb),60);
            testCase.verifyEqual(rmfield(actual.featureMasks,'curb'),rmfield(before.featureMasks,'curb'));
            testCase.verifyEqual(actual.probabilityCloud,before.probabilityCloud);
            stream=RandStream('mt19937ar','Seed',96336718);order=randperm(stream,numel(frame.x));
            shuffled=frame;
            for name=string(fieldnames(frame)).'
                if numel(frame.(name))==numel(order)
                    values=frame.(name);shuffled.(name)=reshape(values(order),[],1);
                end
            end
            reordered=perceiveFrame(shuffled,cfg);
            restored=false(numel(order),1);restored(order)=reordered.featureMasks.curb;
            testCase.verifyEqual(restored,actual.featureMasks.curb);
        end
        function extendsBothReportedSidesWithoutReplacingAcceptedPoints(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            file=fullfile(root,'data','raw','MissisipiPointClouds.mat');
            testCase.assumeTrue(isfile(file),'Recorded source data are required.');
            frame=loadPointCloudFrame(file,538);cfg=perceptionConfig();cfg.executionMode="offline";
            beforeCfg=cfg;beforeCfg.fine.curbContinuationLengthMeters=0;
            before=perceiveFrame(frame,beforeCfg);actual=perceiveFrame(frame,cfg);
            testCase.verifyEqual(nnz(before.featureMasks.curb),131);
            testCase.verifyTrue(all(actual.featureMasks.curb(before.featureMasks.curb)));
            testCase.verifyEqual(rmfield(actual.featureMasks,'curb'),rmfield(before.featureMasks,'curb'));
            testCase.verifyEqual(actual.probabilityCloud,before.probabilityCloud);
            right=[34466 34530 34915 34979 35428 34405];
            left=[29668 29604 29540 29476 29412 29348 28197 28133 28069 28005 27941 ...
                28326 28262 28198 28134 28070 28006 28391 28327 28263 28199 28135 28071 ...
                28456 28392 28328 28264 28200];
            testCase.verifyTrue(all(actual.featureMasks.curb(right)));
            testCase.verifyGreaterThanOrEqual(nnz(actual.featureMasks.curb(left)),18);
            ids=find(actual.featureMasks.curb);nRows=size(frame.x,1);
            for side=[-1 1]
                selected=ids(side*frame.y(ids)>0);
                testCase.verifyGreaterThan(max(frame.x(selected)),17.3);
                % Ring information is used only to audit output density.
                counts=accumarray(mod(selected-1,nRows)+1,1,[nRows 1]);
                previous=find(before.featureMasks.curb & side*frame.y(:)>0);
                previousCounts=accumarray(mod(previous-1,nRows)+1,1,[nRows 1]);
                testCase.verifyLessThanOrEqual(max(counts),max(5,max(previousCounts)));
                added=selected(~before.featureMasks.curb(selected));
                addedCounts=accumarray(mod(added-1,nRows)+1,1,[nRows 1]);
                % Most occupied scan rows remain in the requested approximate
                % 1--5 point range; inference itself never uses scan rows.
                occupiedCounts=addedCounts(addedCounts>0);
                testCase.verifyGreaterThanOrEqual(mean(occupiedCounts<=5),0.8);
            end
            stream=RandStream('mt19937ar','Seed',53829668);order=randperm(stream,numel(frame.x));
            shuffled=frame;
            for name=string(fieldnames(frame)).'
                if numel(frame.(name))==numel(order)
                    values=frame.(name);shuffled.(name)=reshape(values(order),[],1);
                end
            end
            reordered=perceiveFrame(shuffled,cfg);
            restored=false(numel(order),1);restored(order)=reordered.featureMasks.curb;
            testCase.verifyEqual(restored,actual.featureMasks.curb);
        end
    end
end
