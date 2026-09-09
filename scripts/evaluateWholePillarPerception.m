function report=evaluateWholePillarPerception(snapshotFolder)
% evaluateWholePillarPerception: Compare whole-pillar perception with frozen outputs.
% Snapshots contain frame, cfg, baseline (offline result), seconds and frameIndex.
% Agreement measures the old detector, not ground-truth accuracy. Timings are
% warmed perception only; loading, graphics, mapping and registration excluded.
    root=fileparts(fileparts(mfilename('fullpath')));
    if nargin<1, snapshotFolder=fullfile(root,'output','pillar_only_20260908'); end
    files=[dir(fullfile(snapshotFolder,'mississippi_*.mat'));dir(fullfile(snapshotFolder,'downtown_*.mat'))];
    assert(~isempty(files),'No frozen baseline frames found.');
    featureRows={}; candidateRows={}; timingRows={};
    for k=1:numel(files)
        item=load(fullfile(files(k).folder,files(k).name));
        dataset="Mississippi";
        if startsWith(files(k).name,'downtown'), dataset="Downtown"; end
        cfg=perceptionConfig(dataset); cfg.executionMode="offline";
        fine=perceiveFrame(item.frame,cfg);
        for name=cfg.featureNames
            a=fine.featureMasks.(name); b=item.baseline.featureMasks.(name);
            featureRows(end+1,:)={dataset,item.frameIndex,name,nnz(a&b),nnz(a&~b),nnz(~a&b)}; %#ok<AGROW>
            channel=fine.candidates.semanticNames==name;
            oldChannel=item.baseline.candidates.semanticNames==name;
            a=fine.candidates.pillarIndices{channel}; b=item.baseline.candidates.pillarIndices{oldChannel};
            candidateRows(end+1,:)={dataset,item.frameIndex,name, ...
                numel(intersect(a,b)),numel(setdiff(a,b)),numel(setdiff(b,a))}; %#ok<AGROW>
        end
        cfg.executionMode="coarseProbabilityCloud"; perceiveFrame(item.frame,cfg);
        seconds=zeros(1,5);
        for repeat=1:5
            timer=tic; perceiveFrame(item.frame,cfg); seconds(repeat)=toc(timer);
        end
        timingRows(end+1,:)={dataset,item.frameIndex,median(item.seconds),median(seconds),max(seconds)}; %#ok<AGROW>
    end
    variables={'dataset','frameIndex','feature','shared','added','removed'};
    report.fine=cell2table(featureRows,'VariableNames',variables);
    report.coarse=cell2table(candidateRows,'VariableNames',variables);
    report.timing=cell2table(timingRows,'VariableNames', ...
        {'dataset','frameIndex','baselineMedianSeconds','pillarMedianSeconds','pillarMaximumSeconds'});
    report.matlabVersion=string(version);
    report.baselineRevision="9dcd1aeab1a029ce374a27be6d5db37fb8acf066";
    report.scope="Frozen-detector agreement; warm loaded-frame perception; default fine recovery disabled.";
    writetable(report.fine,fullfile(snapshotFolder,'fine_fidelity.csv'));
    writetable(report.coarse,fullfile(snapshotFolder,'coarse_fidelity.csv'));
    writetable(report.timing,fullfile(snapshotFolder,'timing.csv'));
    save(fullfile(snapshotFolder,'evaluation.mat'),'report');
    disp(report.scope);
    fprintf('Fine changed points: added=%d, removed=%d\n',sum(report.fine.added),sum(report.fine.removed));
end
