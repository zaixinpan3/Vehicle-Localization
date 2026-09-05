function report = evaluateCoarsePerceptionSpeed(snapshotFolder, referenceRoot)
% evaluateCoarsePerceptionSpeed: Compare optimized perception with frozen frames.
% Snapshots contain frame, config, coarse, offline, seconds, commit, dataset,
% frameIndex and split. Native and MATLAB backend timing excludes file I/O,
% path setup, warm-up and plotting. No output from a previous call is reused.
% If referenceRoot is supplied, rerun the frozen revision in the same session
% and verify its outputs before measuring it. Restore the caller's MATLAB path.
    if nargin<2, referenceRoot=""; end
    root=fileparts(fileparts(mfilename('fullpath')));
    if nargin<1, snapshotFolder=fullfile(root,'output','coarse_optimization'); end
    files=[dir(fullfile(snapshotFolder,'mississippi_*.mat'));dir(fullfile(snapshotFolder,'downtown_*.mat'))];
    assert(~isempty(files),'No frozen baseline frames were found.');
    cases=cell(numel(files),1);
    for k=1:numel(files)
        loaded=load(fullfile(files(k).folder,files(k).name),'snapshot'); cases{k}=loaded.snapshot;
    end
    repetitions=9;
    referenceTimes=nan(numel(files),repetitions);
    if strlength(string(referenceRoot))>0
        originalPath=path;
        pathCleanup=onCleanup(@() path(originalPath));
        run(fullfile(referenceRoot,'setupVehicleLocalization.m'));
        assert(startsWith(which('perceiveFrame'),char(referenceRoot)),'Wrong reference code on path.');
        for k=1:numel(cases)
            item=cases{k};
            actual=perceiveFrame(item.frame,item.config);
            assert(isequaln(actual,item.coarse),'Reference checkout does not reproduce its frozen output.');
            referenceTimes(k,:)=measureFrame(item.frame,item.config,repetitions);
        end
        clear pathCleanup
    end
    run(fullfile(root,'setupVehicleLocalization.m'));
    cfg=perceptionConfig();
    cfg.executionBackend="native";
    assert(perceptionNativeAvailable("native"),'Build the native backend before evaluation.');
    matlabCfg=cfg; matlabCfg.executionBackend="matlab";
    offlineCfg=cfg; offlineCfg.executionMode="offline";
    featureRows=cell(3*numel(cases),12);
    numericRows=cell(numel(cases),10);
    timingRows=cell(numel(cases),8);
    nativeTimes=zeros(numel(cases),repetitions); matlabTimes=nativeTimes;
    names=["curb","roadMarking","pole"];
    for k=1:numel(cases)
        item=cases{k};
        native=perceiveFrame(item.frame,cfg); fallback=perceiveFrame(item.frame,matlabCfg);
        offline=perceiveFrame(item.frame,offlineCfg);
        for j=1:3
            old=item.coarse.candidates.pillarIndices{j}; now=native.candidates.pillarIndices{j};
            oldPoints=item.offline.featureMasks.(names(j)); newPoints=offline.featureMasks.(names(j));
            tp=numel(intersect(old,now)); fp=numel(setdiff(now,old)); fn=numel(setdiff(old,now));
            featureRows((k-1)*3+j,:)={item.dataset,item.frameIndex,item.split,names(j), ...
                tp,fp,fn,nnz(oldPoints&newPoints),nnz(~oldPoints&newPoints), ...
                nnz(oldPoints&~newPoints),isequal(now,fallback.candidates.pillarIndices{j}), ...
                isequal(item.offline.featureMasks.groundPoint,offline.featureMasks.groundPoint)};
        end
        old=item.coarse.probabilityCloud.components; now=native.probabilityCloud.components;
        sameComponents=isequal(old.semanticId,now.semanticId)&&isequal(old.cellLinIdx,now.cellLinIdx);
        numericRows(k,:)={item.dataset,item.frameIndex,sameComponents, ...
            sameComponents&&isequal(old.count,now.count),difference(old.mean,now.mean), ...
            difference(old.covariance,now.covariance),difference(old.invCovariance,now.invCovariance), ...
            difference(old.semanticProbability,now.semanticProbability), ...
            difference(old.mixtureWeight,now.mixtureWeight), ...
            difference(now.covariance,fallback.probabilityCloud.components.covariance)};
        % Alternate backend order across frames to reduce systematic order bias.
        if mod(k,2)==1
            nativeTimes(k,:)=measureFrame(item.frame,cfg,repetitions);
            matlabTimes(k,:)=measureFrame(item.frame,matlabCfg,repetitions);
        else
            matlabTimes(k,:)=measureFrame(item.frame,matlabCfg,repetitions);
            nativeTimes(k,:)=measureFrame(item.frame,cfg,repetitions);
        end
        timingRows(k,:)={item.dataset,item.frameIndex,item.split,median(item.seconds), ...
            median(referenceTimes(k,:)),median(matlabTimes(k,:)),median(nativeTimes(k,:)),prctile(nativeTimes(k,:),90)};
        fprintf('%s %d: baseline %.1f, MATLAB %.1f, native %.1f ms\n', ...
            item.dataset,item.frameIndex,1000*median(referenceTimes(k,:)), ...
            1000*median(matlabTimes(k,:)),1000*median(nativeTimes(k,:)));
    end
    report.features=cell2table(featureRows,'VariableNames',{'dataset','frameIndex','split','semanticName', ...
        'candidateTP','candidateFP','candidateFN','pointTP','pointFP','pointFN','backendCandidatesEqual','groundPointsEqual'});
    report.numerics=cell2table(numericRows,'VariableNames',{'dataset','frameIndex','sameComponents','sameCounts', ...
        'maxMeanDifference','maxCovarianceDifference','maxInverseDifference','maxEvidenceDifference','maxWeightDifference','backendCovarianceDifference'});
    report.timings=cell2table(timingRows,'VariableNames',{'dataset','frameIndex','split', ...
        'captureBaselineSeconds','repeatedBaselineSeconds','matlabSeconds','nativeSeconds','nativeP90Seconds'});
    report.referenceTimes=referenceTimes; report.nativeTimes=nativeTimes; report.matlabTimes=matlabTimes;
    report.referenceCommit=cases{1}.commit;
    report.matlabVersion=string(version); report.numComputeThreads=maxNumCompThreads;
    report.description="Warmed end-to-end coarse perception; frame loading and plotting excluded. Historical fidelity is not ground-truth accuracy.";
    writetable(report.features,fullfile(snapshotFolder,'feature_fidelity.csv'));
    writetable(report.numerics,fullfile(snapshotFolder,'numerical_fidelity.csv'));
    writetable(report.timings,fullfile(snapshotFolder,'runtime.csv'));
    save(fullfile(snapshotFolder,'evaluation.mat'),'report');
end

function seconds = measureFrame(frame,cfg,repetitions)
    perceiveFrame(frame,cfg);
    seconds=zeros(1,repetitions);
    for repeat=1:repetitions
        timer=tic;
        perceiveFrame(frame,cfg);
        seconds(repeat)=toc(timer);
    end
end

function value = difference(a,b)
    if ~isequal(size(a),size(b)), value=inf; return; end
    if isempty(a), value=0; return; end
    value=max(abs(double(a)-double(b)),[],'all');
end
