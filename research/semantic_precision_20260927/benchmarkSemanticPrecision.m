function benchmarkSemanticPrecision()
% benchmarkSemanticPrecision: Paired warm runs with alternating execution order.
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));rows={};
    for dataset=["Mississippi","Downtown"]
        file='MissisipiPointClouds.mat';frames=[100 300 500 800 1000 1150];
        if dataset=="Downtown",file='downTownPointClouds.mat';frames=[1 100 200 375 500 539];end
        current=perceptionConfig(dataset);baseline=current;baseline.semanticPrecision.enabled=false;
        clouds=cell(size(frames));
        for k=1:numel(frames),clouds{k}=loadPointCloudFrame(fullfile(root,'data','raw',file),frames(k));end
        perceiveFrame(clouds{1},current);perceiveFrame(clouds{1},baseline);
        for repeat=1:5
            for k=1:numel(frames)
                configs={baseline,current};labels=["baseline","precision"];
                order=1:2;if mod(k+repeat,2),order=2:-1:1;end
                for j=order
                    start=tic;p=perceiveFrame(clouds{k},configs{j});elapsed=toc(start);
                    rows{end+1,1}=struct('dataset',dataset,'frame',frames(k),'repeat',repeat, ...
                        'mode',labels(j),'milliseconds',1000*elapsed,'components',p.probabilityCloud.components.numComponents); %#ok<AGROW>
                end
            end
        end
    end
    T=struct2table(vertcat(rows{:}));writetable(T,fullfile(folder,'runtime_paired.csv'));
    disp(groupsummary(T,{'dataset','mode'},'median','milliseconds'));
end
