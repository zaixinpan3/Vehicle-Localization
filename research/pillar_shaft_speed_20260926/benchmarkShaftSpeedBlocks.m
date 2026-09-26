function benchmarkShaftSpeedBlocks(dataset,frames,label,baselineDir,variantNames,variantConfigs,passes)
% benchmarkShaftSpeedBlocks: Paired timing with continuous backend blocks.
% Each backend receives three untimed warm-ups after its path switch, followed
% by uninterrupted frame-order replay. Alternate passes reverse backend order.
% Only perceiveFrame is timed; loading, switching, comparison, and I/O are not.
% Completed passes are saved independently so partial runs remain identifiable.
    root=setupVehicleLocalization(); folder=fileparts(mfilename('fullpath'));
    if nargin<7, passes=2; end
    if nargin<6
        variantNames="optimized"; variantConfigs={perceptionConfig(dataset)};
    end
    variantNames=string(variantNames(:).'); frames=double(frames(:).');
    assert(numel(variantNames)==numel(variantConfigs));
    assert(all(matches(variantNames,regexpPattern('[A-Za-z][A-Za-z0-9_]*'))));
    assert(passes>=1 && passes==floor(passes));
    originalPath=path;
    cleanup=onCleanup(@()shaftSpeedUseBackend(originalPath,baselineDir,false));
    [block,references]=shaftSpeedLoadFrames(root,dataset,frames);
    shaftSpeedUseBackend(originalPath,baselineDir,true);
    baselineCfg=perceptionConfig(dataset); baselineCfg.executionBackend="native";
    configurations=[{baselineCfg},variantConfigs]; names=["baseline",variantNames];
    for j=1:numel(configurations), configurations{j}.executionBackend="native"; end
    rows=cell(numel(frames)*passes*numel(names),1); nextRow=0;
    saved=cell(numel(frames),numel(names),passes);
    output=fullfile(root,'output','pillar_shaft_speed_20260926');
    if ~isfolder(output),mkdir(output);end
    method='continuous backend blocks; 3 warmups; reversed order on alternate passes';
    for pass=1:passes
        order=1:numel(names);
        if mod(pass,2)==0, order=fliplr(order); end
        timings=zeros(numel(frames),numel(names));
        for j=order
            shaftSpeedUseBackend(originalPath,baselineDir,j==1);
            for warmup=1:3
                index=1+mod(warmup-1,numel(frames));
                perceiveFrame(block(index),configurations{j});
            end
            for k=1:numel(frames)
                timer=tic;
                p=perceiveFrame(block(k),configurations{j});
                timings(k,j)=1000*toc(timer);
                saved{k,j,pass}=shaftSpeedRecord(p);
            end
            fprintf('%s pass %d/%d: %s, %d frames, median %.3f ms\n', ...
                label,pass,passes,names(j),numel(frames),median(timings(:,j)));
        end
        for k=1:numel(frames)
            baseline=saved{k,1,pass}; reference=references{k};
            if ~isempty(reference)
                assert(isequal(baseline.poleCells,reference.poleCells), ...
                    'Frozen baseline mask differs from the earlier full-capture output.');
                assert(isequaln(baseline.components,reference.components), ...
                    'Frozen baseline Gaussian output differs from the earlier full capture.');
                assert(isequaln(baseline.sourceSummary,reference.sourceSummary), ...
                    'Frozen baseline input summary differs from the earlier full capture.');
            end
            for j=1:numel(names)
                nextRow=nextRow+1;
                identity=struct('dataset',string(dataset),'frame',frames(k), ...
                    'pass',pass,'variant',names(j),'milliseconds',timings(k,j), ...
                    'baselineMilliseconds',timings(k,1));
                metrics=shaftSpeedMetrics(saved{k,j,pass},baseline,reference);
                for name=fieldnames(metrics).', identity.(name{1})=metrics.(name{1}); end
                rows{nextRow}=identity;
            end
        end
        T=struct2table(vertcat(rows{1:nextRow}));
        completedPasses=pass;
        writetable(T,fullfile(folder,char(string(label)+"_paired.csv")));
        save(fullfile(output,char(string(label)+"_paired.mat")), ...
            'frames','dataset','names','configurations','passes','completedPasses', ...
            'method','saved','T','-v7.3');
    end
    disp(groupsummary(T,'variant',{'median','max'},'milliseconds'));
end
