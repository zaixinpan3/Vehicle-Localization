function benchmarkCurbRecovery()
% benchmarkCurbRecovery: Paired warm all-channel Mississippi perception timing.
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));frames=[100 300 500 800 1000 1150];
    cfg=perceptionConfig('Mississippi');before=cfg;before.semanticPrecision.modelFiles=struct();
    clouds=cell(size(frames));for k=1:numel(frames),clouds{k}=loadPointCloudFrame(fullfile(root,'data','raw','MissisipiPointClouds.mat'),frames(k));end
    perceiveFrame(clouds{1},cfg);perceiveFrame(clouds{1},before);rows={};
    for repeat=1:5
        for k=1:numel(frames)
            configurations={before,cfg};labels=["previousPrecision","curbRecovery"];order=1:2;if mod(k+repeat,2),order=2:-1:1;end
            for j=order
                timer=tic;p=perceiveFrame(clouds{k},configurations{j});seconds=toc(timer);
                rows{end+1,1}=struct('frame',frames(k),'repeat',repeat,'mode',labels(j),'milliseconds',1000*seconds, ...
                    'components',p.probabilityCloud.components.numComponents); %#ok<AGROW>
            end
        end
    end
    T=struct2table(vertcat(rows{:}));writetable(T,fullfile(folder,'runtime.csv'));
    disp(groupsummary(T,'mode','median','milliseconds'));
end
