function captureShaftSpeedBatch(dataset,frames,label,cfg,batchIndex)
% captureShaftSpeedBatch: Bounded optimized replay against stored shaft output.
% Invoke separately for each <=100-frame batch to keep tool calls bounded.
% Baseline references must already exist for every requested frame.
    root=setupVehicleLocalization(); folder=fileparts(mfilename('fullpath'));
    frames=double(frames(:).');
    assert(numel(frames)<=100,'Use separate calls with at most 100 frames.');
    assert(batchIndex>=1 && batchIndex==floor(batchIndex));
    cfg.executionBackend="native";
    [block,references]=shaftSpeedLoadFrames(root,dataset,frames);
    assert(all(~cellfun(@isempty,references)), ...
        'A frozen output is missing; use the paired benchmark for unseen frames.');
    perceiveFrame(block(1),cfg);
    rows=cell(numel(frames),1); records=rows;
    for k=1:numel(frames)
        timer=tic; p=perceiveFrame(block(k),cfg); elapsed=1000*toc(timer);
        records{k}=shaftSpeedRecord(p);
        identity=struct('dataset',string(dataset),'frame',frames(k), ...
            'batch',batchIndex,'milliseconds',elapsed);
        metrics=shaftSpeedMetrics(records{k},references{k},references{k});
        for name=fieldnames(metrics).', identity.(name{1})=metrics.(name{1}); end
        rows{k}=identity;
    end
    T=struct2table(vertcat(rows{:}));
    stem=sprintf('%s_batch_%03d',label,batchIndex);
    writetable(T,fullfile(folder,[stem '.csv']));
    output=fullfile(root,'output','pillar_shaft_speed_20260926');
    if ~isfolder(output),mkdir(output);end
    save(fullfile(output,[stem '.mat']),'frames','dataset','cfg','records','T','-v7.3');
    fprintf('%s: %d frames, lost %d, added %d, exact components %d/%d, median %.3f ms\n', ...
        stem,numel(frames),sum(T.lostCount),sum(T.addedCount), ...
        nnz(T.componentsExact),height(T),median(T.milliseconds));
end
