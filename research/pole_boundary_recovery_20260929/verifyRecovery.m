function verifyRecovery()
% verifyRecovery Verify the Mississippi operating point on raw frames.
    root=setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));addpath('research/pillar_fine_alignment_20260926');
    frozen=load('output/pole_precision_20260927/mississippi.mat','frames','references');
    prediction=readtable(fullfile(dest,'expected.csv'));
    prediction.selected=strcmpi(string(prediction.selected),'true');
    cfg=perceptionConfig('Mississippi');cfg.featureNames="pole";
    store=matfile(fullfile(root,'data/raw/MissisipiPointClouds.mat'));rows=cell(0,1);started=tic;block=[];first=0;
    for k=1:numel(frozen.frames)
        frameId=frozen.frames(k);
        if isempty(block)||frameId>=first+numel(block),first=frameId;block=store.pointClouds(1,first:min(first+49,1170));end
        frame=block(frameId-first+1);p=perceiveFrame(frame,cfg);ids=double(p.candidates.pillarIndices{1});
        expected=prediction.pillar(prediction.frame==frameId & prediction.selected);
        assert(isequal(sort(ids(:)),sort(expected(:))),'Pole mismatch at %d',frameId);
        metric=measureFinePoleAlignment(frame,frozen.references{k}.pointIndices,ids,p.candidates.geometry);metric.frame=frameId;
        rows{end+1,1}=metric; %#ok<AGROW>
        if mod(k,100)==0 || k==numel(frozen.frames)
            writetable(struct2table(vertcat(rows{:})),fullfile(dest,'deployed_pole_replay.csv'));
            fprintf('Pole replay %d/%d %.1fs\n',k,numel(frozen.frames),toc(started));
        end
    end
    fprintf('POLE_CALIBRATION_REPLAY_COMPLETED\n');
end
