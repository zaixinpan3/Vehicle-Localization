function report = compareLocalizationPipelineTiming(inputFolder, outputFolder, baselineFunction, repetitions)
% compareLocalizationPipelineTiming Compare complete calls in randomized pairs.
% Inputs are raw frames and cached maps from benchmarkLocalizationPipeline.
% The reference handle runs a separately named source snapshot. Timing covers
% perception through event output; comparisons, warmup and all I/O are outside.
    arguments
        inputFolder (1,1) string
        outputFolder (1,1) string
        baselineFunction (1,1) function_handle
        repetitions (1,1) double {mustBeInteger,mustBePositive} = 20
    end
    loaded=load(fullfile(inputFolder,'inputs.mat'),'inputs','cfg');
    if ~isfolder(outputFolder), mkdir(outputFolder); end
    starts=[.5,-.4,deg2rad(2);-.5,.4,-deg2rad(2);0,0,0];
    functions={baselineFunction,@localizeLidarFrame};
    methods=["baseline","optimized"];
    cases=3*numel(loaded.inputs);
    rows=cell(2*cases*repetitions,14); warm=cell(4*cases,14);
    stream=RandStream('mt19937ar','Seed',20260906);
    index=0;
    for pass=1:2
        for caseIndex=1:cases
            for method=1:2
                index=index+1;
                warm(index,:)=runCase(functions{method},loaded,starts,caseIndex,methods(method),pass,index);
            end
        end
    end
    index=0; maxPoseDifference=0; exactClouds=true; exactPairs=true;
    for pass=1:repetitions
        for caseIndex=randperm(stream,cases)
            results=cell(2,1);
            for method=randperm(stream,2)
                index=index+1;
                [rows(index,:),results{method}]=runCase(functions{method},loaded,starts,caseIndex,methods(method),pass,index);
            end
            before=results{1}; after=results{2};
            assert(before.accepted==after.accepted && before.reason==after.reason, 'Acceptance changed.');
            maxPoseDifference=max(maxPoseDifference,max(abs(before.poseXYTheta-after.poseXYTheta)));
            exactClouds=exactClouds && isequaln(before.probabilityCloud,after.probabilityCloud);
            exactPairs=exactPairs && isequal(before.correspondences.source,after.correspondences.source) ...
                && isequal(before.correspondences.target,after.correspondences.target);
        end
    end
    names={'method','frame','start','repetition','order','totalMs','perceptionMs', ...
        'registrationMs','accepted','reason','iterations','x','y','psi'};
    report.calls=cell2table(rows,'VariableNames',names);
    report.warmup=cell2table(warm,'VariableNames',names);
    report.quality=struct('maxPoseComponentDifference',maxPoseDifference, ...
        'exactProbabilityClouds',exactClouds,'exactCorrespondenceIndices',exactPairs);
    report.metadata=struct('seed',20260906,'repetitions',repetitions, ...
        'matlabVersion',version,'computationalThreads',maxNumCompThreads, ...
        'baselineFunction',string(func2str(baselineFunction)), ...
        'timedScope',"Preloaded raw frame and cached local map to pose event or rejection", ...
        'order',"Randomized scene/start order and randomized method order within each pair");
    writetable(report.calls,fullfile(outputFolder,'calls.csv'));
    writetable(report.warmup,fullfile(outputFolder,'warmup.csv'));
    save(fullfile(outputFolder,'report.mat'),'report');
    disp(report.quality);
    for method=methods
        values=report.calls.totalMs(report.calls.method==method);
        fprintf('%s: median %.3f ms, maximum %.3f ms, %d/%d above 100 ms.\n', ...
            method,median(values),max(values),nnz(values>100),numel(values));
    end
end

function [row,result]=runCase(f,loaded,starts,caseIndex,method,pass,index)
    scene=floor((caseIndex-1)/3)+1; start=mod(caseIndex-1,3)+1;
    item=loaded.inputs{scene}; cfg=loaded.cfg;
    cfg.perception.coarseProbabilityCloud.projectionRotation=item.tilt;
    timer=tic;
    [event,result]=f(item.frame,item.mapCloud,item.pose+starts(start,:),item.frameIndex,cfg);
    elapsed=1000*toc(timer);
    assert(result.accepted==~isempty(event),'Acceptance/event mismatch.');
    if ~isempty(event)
        assert(isequal(size(event.pose),[1 3]) && all(isfinite(event.pose)),'Invalid accepted pose.');
    end
    row={method,item.frameIndex,start,pass,index,elapsed,1000*result.perceptionSeconds, ...
        1000*result.registrationSeconds,result.accepted,string(result.reason), ...
        result.iterations,result.poseXYTheta(1),result.poseXYTheta(2),result.poseXYTheta(3)};
end
