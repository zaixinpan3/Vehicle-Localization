function benchmarkGeometryPipeline()
% benchmarkGeometryPipeline: Alternated warmed comparisons on raw frames.
% Data loading, model loading and configuration are outside measured intervals.
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));rows={};
    for dataset=["Mississippi","Downtown"]
        file='MissisipiPointClouds.mat';last=1170;if dataset=="Downtown",file='downTownPointClouds.mat';last=539;end
        source=matfile(fullfile(root,'data','raw',file));frames=round(linspace(1,last,24));clouds=cell(size(frames));
        for k=1:numel(frames),clouds{k}=source.pointClouds(1,frames(k));end
        cfg=perceptionConfig(dataset);baseline=cfg;baseline.offGroundFeatures.pole.distributionValidation.enabled=false;
        configs={baseline,cfg};
        for warm=1:2,perceiveFrame(clouds{1},baseline);perceiveFrame(clouds{1},cfg);end
        for repeat=1:3
            for k=1:numel(frames)
                order=1:2;if mod(k+repeat,2)==0,order=2:-1:1;end
                for variant=order
                    timer=tic;p=perceiveFrame(clouds{k},configs{variant});milliseconds=1000*toc(timer);
                    rows{end+1,1}=struct('dataset',dataset,'frame',frames(k),'repeat',repeat,'variant',variant, ...
                        'milliseconds',milliseconds,'polePillars',numel(p.candidates.pillarIndices{p.candidates.semanticNames=="pole"})); %#ok<AGROW>
                end
            end
        end
        fprintf('Paired timing complete: %s\n',dataset);
    end
    T=struct2table(vertcat(rows{:}));writetable(T,fullfile(folder,'paired_runtime.csv'));
    disp(groupsummary(T,{'dataset','variant'},{'mean','median'},'milliseconds'));
end
