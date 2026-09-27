function replaySemanticPrecision(datasets)
% replaySemanticPrecision: Replay selected raw frames against frozen labels.
% Includes every frame denominator and compares model predictions numerically.
    if nargin<1,datasets=["Mississippi","Downtown"];end
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));
    addpath(fullfile(root,'research','pillar_fine_alignment_20260926'));
    output=fullfile(root,'output','semantic_precision_20260927');
    for dataset=string(datasets)
        label=lower(dataset);s=load(fullfile(output,label+"_baseline.mat"));
        filename='MissisipiPointClouds.mat';if dataset=="Downtown",filename='downTownPointClouds.mat';end
        source=matfile(fullfile(root,'data','raw',filename));cfg=perceptionConfig(dataset);
        predictions=struct();thresholds=struct();
        for name=s.names
            if name=="pole",continue;end
            predictions.(name)=readtable(fullfile(output,name+"_predictions.csv"),'TextType','string');
            model=jsondecode(fileread(fullfile(root,'config',name+"PillarPrecisionModel.json")));
            thresholds.(name)=model.decisionThresholds.(label);
        end
        rows={};lastBlock=-1;started=tic;
        for k=1:numel(s.frames)
            frameId=s.frames(k);blockId=floor((frameId-1)/50);
            if blockId~=lastBlock
                first=blockId*50+1;last=min(first+49,max(s.frames));block=source.pointClouds(1,first:last);lastBlock=blockId;
            end
            frame=block(frameId-first+1);timer=tic;p=perceiveFrame(frame,cfg);seconds=toc(timer);
            assert(isequaln(p.sourceSummary,s.records{k}.sourceSummary),'Source preprocessing changed.');
            for j=1:numel(s.names)
                name=s.names(j);ids=double(p.candidates.pillarIndices{p.candidates.semanticNames==name});
                old=s.records{k}.candidates;oldIds=double(old.pillarIndices{old.semanticNames==name});
                assert(all(ismember(ids,oldIds)),'A precision gate must not add proposals.');
                if name=="pole"
                    assert(isequal(ids,oldIds),'Pole selection changed.');
                else
                    prediction=predictions.(name);
                    expected=prediction.pillar(prediction.dataset==label & prediction.frame==frameId & prediction.score>=thresholds.(name));
                    assert(isequal(sort(ids(:)),sort(expected(:))),'Exported inference differs: %s %d %s.',dataset,frameId,name);
                end
                m=measureFinePoleAlignment(frame,s.references{k,j},ids,p.candidates.geometry);
                m.dataset=dataset;m.frame=frameId;m.feature=name;m.coarseSeconds=seconds;
                rows{end+1,1}=m; %#ok<AGROW>
            end
            if mod(k,50)==0||k==numel(s.frames)
                T=struct2table(vertcat(rows{:}));writetable(T,fullfile(folder,label+"_replay.csv"));
                fprintf('%s replay %d/%d %.1fs\n',dataset,k,numel(s.frames),toc(started));
            end
        end
    end
end
