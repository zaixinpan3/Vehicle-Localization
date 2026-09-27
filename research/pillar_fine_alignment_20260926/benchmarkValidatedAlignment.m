function T=benchmarkValidatedAlignment()
% benchmarkValidatedAlignment: Alternate warmed baseline and revised pipelines.
% Raw-frame loading and configuration are outside the timed intervals.
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));rows={};
    datasets=["Mississippi","Downtown"];counts=[1170 539];
    files=["MissisipiPointClouds.mat","downTownPointClouds.mat"];
    for d=1:2
        source=matfile(fullfile(root,'data','raw',files(d)));
        frames=round(linspace(1,counts(d),24));clouds=cell(size(frames));
        for k=1:numel(frames),clouds{k}=source.pointClouds(1,frames(k));end
        cfg=perceptionConfig(datasets(d));baseline=cfg;
        baseline.offGroundFeatures.pole.detector="shaft";
        baseline.offGroundFeatures.pole.probabilityEvidence="shaft";
        configurations={baseline,cfg};
        for warm=1:2
            perceiveFrame(clouds{1},baseline);perceiveFrame(clouds{1},cfg);
        end
        for repeat=1:3
            for k=1:numel(frames)
                order=1:2;if mod(k+repeat,2)==0,order=2:-1:1;end
                for variant=order
                    timer=tic;p=perceiveFrame(clouds{k},configurations{variant});milliseconds=1000*toc(timer);
                    rows{end+1,1}=struct('dataset',datasets(d),'frame',frames(k), ...
                        'repeat',repeat,'variant',variant,'milliseconds',milliseconds, ...
                        'polePillars',numel(p.candidates.pillarIndices{p.candidates.semanticNames=="pole"})); %#ok<AGROW>
                end
            end
        end
        fprintf('Paired timing complete: %s\n',datasets(d));
    end
    T=struct2table(vertcat(rows{:}));writetable(T,fullfile(folder,'paired_runtime.csv'));
    disp(groupsummary(T,{'dataset','variant'},{'mean','median'},'milliseconds'));
end
