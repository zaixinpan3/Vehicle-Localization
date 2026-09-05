function report = evaluateHeightPerceptionRegression(referenceRoot, outputFolder)
% evaluateHeightPerceptionRegression: Compare retained-height perception to
% an explicit immutable source export. Both paths must use the same backend.
% Alternating runs exclude loading/path switches. No source results are cached.
    arguments
        referenceRoot (1,1) string
        outputFolder (1,1) string
    end
    root=string(fileparts(fileparts(mfilename('fullpath'))));
    savedPath=path;
    cleanup=onCleanup(@() path(savedPath));
    frames=[260 550 900];
    loadedFrames=cell(3,1);
    for k=1:numel(frames)
        loadedFrames{k}=loadPointCloudFrame(fullfile(root,'data','raw','MissisipiPointClouds.mat'),frames(k));
    end
    timings=zeros(3,2,3);
    output=cell(3,2);
    fine=cell(3,2);
    roots=[referenceRoot,root];
    for repeat=1:3
        order=[1 2];
        if mod(repeat,2)==0, order=[2 1]; end
        for implementation=order
            path(savedPath);
            run(fullfile(roots(implementation),'setupVehicleLocalization.m'));
            assert(string(which('perceiveFrame'))==fullfile(roots(implementation),'perception','perceiveFrame.m'));
            cfg=perceptionConfig(); cfg.executionBackend="native";
            for k=1:numel(frames)
                frame=loadedFrames{k};
                output{k,implementation}=perceiveFrame(frame,cfg);
                timings(k,implementation,repeat)=1e3*timeit(@() perceiveCoarseProbabilityCloud(frame,cfg));
                if repeat==1
                    offlineCfg=cfg; offlineCfg.executionMode="offline";
                    fine{k,implementation}=perceiveFrame(frame,offlineCfg);
                end
            end
        end
    end
    metrics=zeros(3,7);
    for k=1:numel(frames)
        before=output{k,1}; after=output{k,2};
        assert(isequaln(before.candidates,after.candidates));
        assert(isequaln(fine{k,1}.featureMasks,fine{k,2}.featureMasks));
        for name=string(fieldnames(before.probabilityCloud.components)).'
            assert(isequaln(before.probabilityCloud.components.(name),after.probabilityCloud.components.(name)), ...
                'Existing component field changed: %s',name);
        end
        elapsed=median(timings(k,:,:),3);
        metrics(k,:)=[frames(k),elapsed,elapsed(2)-elapsed(1), ...
            after.probabilityCloud.components.numComponents,1,1];
    end
    report=struct();
    report.metrics=array2table(metrics,'VariableNames',{'frameIndex','referenceMilliseconds','heightMilliseconds', ...
        'additionalMilliseconds','componentCount','candidateAndFineMasksEqual','allExistingComponentFieldsEqual'});
    report.timings=timings;
    report.frames=frames;
    if ~isfolder(outputFolder), mkdir(outputFolder); end
    writetable(report.metrics,fullfile(outputFolder,'perception_regression.csv'));
    save(fullfile(outputFolder,'perception_regression.mat'),'report');
    disp(report.metrics);
end
