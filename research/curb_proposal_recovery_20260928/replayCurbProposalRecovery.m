function replayCurbProposalRecovery()
% replayCurbProposalRecovery: Exact raw replay against frozen references and exports.
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));
    addpath(fullfile(root,'research','pillar_fine_alignment_20260926'));
    out=fullfile(root,'output','curb_recovery_20260928');old=fullfile(root,'output','semantic_precision_20260927');
    frozen=load(fullfile(old,'mississippi_baseline.mat'));cfg=perceptionConfig('Mississippi');
    prediction=readtable(fullfile(out,'forest_predictions.csv'));extra=readtable(fullfile(root,'output','curb_proposal_recovery_20260928','reviewed_chain_predictions.csv'));
    extra.accepted=strcmpi(string(extra.accepted),"true");
    model=jsondecode(fileread(fullfile(root,'config','mississippiCurbPillarPrecisionModel.json')));
    signPrediction=readtable(fullfile(old,'trafficSign_predictions.csv'),'TextType','string');signPrediction=signPrediction(signPrediction.dataset=="mississippi",:);
    signModel=jsondecode(fileread(fullfile(root,'config','trafficSignPillarPrecisionModel.json')));
    source=matfile(fullfile(root,'data','raw','MissisipiPointClouds.mat'));lastBlock=-1;rows={};started=tic;
    for k=1:numel(frozen.frames)
        frameId=frozen.frames(k);blockId=floor((frameId-1)/50);
        if blockId~=lastBlock
            first=blockId*50+1;last=min(first+49,max(frozen.frames));block=source.pointClouds(1,first:last);lastBlock=blockId;
        end
        frame=block(frameId-first+1);timer=tic;p=perceiveFrame(frame,cfg);elapsed=toc(timer);
        assert(isequaln(p.sourceSummary,frozen.records{k}.sourceSummary),'Source preprocessing changed.');
        for j=1:numel(frozen.names)
            name=frozen.names(j);ids=double(p.candidates.pillarIndices{p.candidates.semanticNames==name});
            oldCandidates=frozen.records{k}.candidates;oldIds=double(oldCandidates.pillarIndices{oldCandidates.semanticNames==name});
            if name~="curb",assert(all(ismember(ids,oldIds)),'Non-curb candidates changed.');end
            if name=="curb"
                expected=[prediction.pillar(prediction.frame==frameId & prediction.score>=model.decisionThreshold);extra.pillar(extra.frame==frameId & extra.accepted)];
            elseif name=="trafficSign"
                expected=signPrediction.pillar(signPrediction.frame==frameId & signPrediction.score>=signModel.decisionThresholds.mississippi);
            else
                expected=oldIds;
            end
            assert(isequal(sort(ids(:)),sort(expected(:))),'Candidate mismatch: frame %d %s.',frameId,name);
            metric=measureFinePoleAlignment(frame,frozen.references{k,j},ids,p.candidates.geometry);
            metric.frame=frameId;metric.feature=name;metric.coarseSeconds=elapsed;rows{end+1,1}=metric; %#ok<AGROW>
        end
        if mod(k,100)==0 || k==numel(frozen.frames)
            writetable(struct2table(vertcat(rows{:})),fullfile(folder,'replay.csv'));
            fprintf('Mississippi replay %d/%d %.1fs\n',k,numel(frozen.frames),toc(started));
        end
    end
end
