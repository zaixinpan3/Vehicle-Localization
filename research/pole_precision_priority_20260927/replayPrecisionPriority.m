function replayPrecisionPriority()
% replayPrecisionPriority: Verify the precision-first default on raw frames.
% Fine references are evaluation inputs only and never reach perception.
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));
    addpath(fullfile(root,'research','pillar_fine_alignment_20260926'));
    oldThreshold=.1488746496646483;
    frozen=jsondecode(fileread(fullfile(folder,'threshold_freeze.json')));
    predictions=readtable(fullfile(root,'research','pole_geometry_20260927', ...
        'stable_moments_model_predictions.csv'),'TextType','string');
    output=fullfile(root,'output','pole_precision_priority_20260927');
    if ~isfolder(output),mkdir(output);end
    for dataset=["Mississippi","Downtown"]
        label=lower(char(dataset));file='MissisipiPointClouds.mat';if dataset=="Downtown",file='downTownPointClouds.mat';end
        s=load(fullfile(root,'output','pole_precision_20260927',[label '.mat']),'frames','references');
        source=matfile(fullfile(root,'data','raw',file));cfg=perceptionConfig(dataset);
        assert(cfg.offGroundFeatures.pole.distributionValidation.minimumScore==frozen.choices.cap05.threshold);
        baseline=cfg;baseline.offGroundFeatures.pole.distributionValidation.minimumScore=oldThreshold;
        rows=cell(numel(s.frames),1);before=rows;lastBlock=-1;started=tic;
        for k=1:numel(s.frames)
            blockId=floor((s.frames(k)-1)/50);
            if blockId~=lastBlock
                first=blockId*50+1;last=min(first+49,max(s.frames));block=source.pointClouds(1,first:last);lastBlock=blockId;
            end
            frame=block(s.frames(k)-first+1);old=perceiveFrame(frame,baseline);p=perceiveFrame(frame,cfg);
            selected=double(p.candidates.pillarIndices{p.candidates.semanticNames=="pole"});
            oldIds=double(old.candidates.pillarIndices{old.candidates.semanticNames=="pole"});
            expected=predictions.pillar(predictions.dataset==dataset & predictions.frame==s.frames(k) & ...
                predictions.score>=cfg.offGroundFeatures.pole.distributionValidation.minimumScore);
            assert(isequal(sort(selected(:)),sort(expected(:))),'Frozen selections differ: %s %d.',dataset,s.frames(k));
            assert(all(ismember(selected,oldIds)),'A stricter threshold added a selection.');
            assert(isequaln(p.sourceSummary,old.sourceSummary),'Ground/filter counts changed.');
            verifyNonPole(p.probabilityCloud.components,old.probabilityCloud.components);
            ref=s.references{k};m=measureFinePoleAlignment(frame,ref.pointIndices,selected,p.candidates.geometry);
            m.frame=s.frames(k);m.nonPoleComponentsUnchanged=true;m.sameFrozenSelections=true;rows{k}=m;
            m=measureFinePoleAlignment(frame,ref.pointIndices,oldIds,p.candidates.geometry);m.frame=s.frames(k);before{k}=m;
            if mod(k,50)==0||k==numel(s.frames)
                T=struct2table(vertcat(rows{1:k}));B=struct2table(vertcat(before{1:k}));
                writetable(T,fullfile(folder,[label '_replay.csv']));writetable(B,fullfile(folder,[label '_previous_replay.csv']));
                fprintf('%s %d/%d %.1fs: FP %.4f, coverage %.4f\n',dataset,k,numel(s.frames),toc(started), ...
                    sum(T.extraPillarCount)/max(sum(T.candidatePillarCount),1),sum(T.coveredFinePointCount)/sum(T.finePointCountInRoi));
            end
        end
        save(fullfile(output,[label '_replay.mat']),'cfg','baseline','T','B');
    end
end

function verifyNonPole(current,baseline)
    a=current.semanticName~="pole";b=baseline.semanticName~="pole";
    for name=fieldnames(current).'
        key=name{1};if any(strcmp(key,{'mixtureWeight','numComponents'})),continue;end
        x=current.(key);y=baseline.(key);
        if any(strcmp(key,{'covarianceXYZ','covariance','invCovariance'})),same=isequaln(x(:,:,a),y(:,:,b));
        else,same=isequaln(x(a,:),y(b,:));end
        assert(same,'Non-pole component changed: %s',key);
    end
end
