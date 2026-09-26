function benchmarkShaftSpeed(dataset,frames,label,baselineDir,variantNames,variantConfigs,passes)
% benchmarkShaftSpeed: Paired frozen-baseline and optimized replay timings.
% Build the baseline directory from the documented frozen commit before use.
% Path switching, MEX loading, frame reads, comparisons, and writes are untimed.
% Each timed invocation follows a warm-up of that same backend on that frame.
% Rotating variant order and repeated passes limit systematic order effects.
    root=setupVehicleLocalization(); folder=fileparts(mfilename('fullpath'));
    if nargin<7, passes=1; end
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
    rowCount=numel(frames)*passes*numel(names); rows=cell(rowCount,1); nextRow=0;
    saved=cell(numel(frames),numel(names));
    for pass=1:passes
        for k=1:numel(frames)
            order=circshift(1:numel(names),mod(k+pass-2,numel(names)));
            timings=zeros(1,numel(names));
            for j=order
                shaftSpeedUseBackend(originalPath,baselineDir,j==1);
                perceiveFrame(block(k),configurations{j});
                timer=tic; p=perceiveFrame(block(k),configurations{j}); timings(j)=1000*toc(timer);
                saved{k,j}=shaftSpeedRecord(p);
            end
            baseline=saved{k,1}; reference=references{k};
            if ~isempty(reference)
                assert(isequal(baseline.poleCells,reference.poleCells), ...
                    'Frozen baseline mask differs from the earlier full-capture output.');
                assert(isequaln(baseline.components,reference.components), ...
                    'Frozen baseline Gaussian output differs from the earlier full capture.');
            end
            for j=1:numel(names)
                nextRow=nextRow+1;
                identity=struct('dataset',string(dataset),'frame',frames(k), ...
                    'pass',pass,'variant',names(j),'milliseconds',timings(j), ...
                    'baselineMilliseconds',timings(1));
                rows{nextRow}=mergeStructs(identity,shaftSpeedMetrics(saved{k,j},baseline,reference));
            end
        end
        fprintf('%s paired pass %d/%d, %d frames\n',label,pass,passes,numel(frames));
    end
    T=struct2table(vertcat(rows{:}));
    writetable(T,fullfile(folder,char(string(label)+"_paired.csv")));
    output=fullfile(root,'output','pillar_shaft_speed_20260926');
    if ~isfolder(output),mkdir(output);end
    save(fullfile(output,char(string(label)+"_paired.mat")), ...
        'frames','dataset','names','configurations','passes','saved','T','-v7.3');
    disp(groupsummary(T,'variant',{'median','max'},'milliseconds'));
end

function merged=mergeStructs(left,right)
    merged=left;
    for name=fieldnames(right).', merged.(name{1})=right.(name{1}); end
end
