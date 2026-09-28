function diagnoseCurbProposals(frameIndex)
% diagnoseCurbProposals: Attribute reference coverage loss before acceptance.
    if nargin<1,frameIndex=900;end
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));
    out=fullfile(root,'output','curb_proposal_recovery_20260928');if ~isfolder(out),mkdir(out);end
    cache=load(fullfile(root,'output','semantic_precision_20260927','mississippi_baseline.mat'),'frames','references','names');
    at=cache.frames==frameIndex;fine=double(cache.references{at,cache.names=="curb"}(:));
    frame=loadPointCloudFrame(fullfile(root,'data','raw','MissisipiPointClouds.mat'),frameIndex);
    cfg=perceptionConfig('Mississippi');cfg.coarseProbabilityCloud.storeDiagnostics=true;
    precision=cfg.semanticPrecision;
    if isfield(precision.modelFiles,'curbRecovery'),precision.modelFiles=rmfield(precision.modelFiles,'curbRecovery');end
    cfg.semanticPrecision.enabled=false;p=perceiveFrame(frame,cfg);ground=p.diagnostics.ground;gc=p.diagnostics.groundPointContext;
    localGeometry=struct('origin',ground.cellOrigin,'cellSize',ground.cellSize,'mapSize',ground.cellMapSize);
    addpath(fullfile(root,'research','pillar_fine_alignment_20260926'));
    [~,detail]=measureFinePoleAlignment(frame,fine,find(ground.curbCellMask),localGeometry);
    target=detail.finePointPillarIds;assert(all(target>0),'Reference outside branch raster.');
    count=accumarray(target,1,[prod(ground.cellMapSize),1]);inGround=ismember(fine,gc.groundOriginalPointIdx);
    groundCount=accumarray(target,double(inGround),[prod(ground.cellMapSize),1]);
    [current,~]=filterSemanticPillarCandidates(ground,struct(),"curb",precision,gc,struct());
    ids=find(count>0 | ground.curbCellMask(:));[r,c]=ind2sub(ground.cellMapSize,ids);
    xy=ground.cellOrigin+([c r]-.5).*ground.cellSize;globalBins=floor((xy-p.candidates.geometry.origin)./p.candidates.geometry.cellSize)+1;
    globalIds=sub2ind(p.candidates.geometry.mapSize,globalBins(:,2),globalBins(:,1));
    T=table(globalIds,xy(:,1),xy(:,2),count(ids),groundCount(ids),ground.curbCellMask(ids),current.curbCellMask(ids), ...
        'VariableNames',{'pillar','x','y','fineCount','fineInGround','proposal','selected'});
    distance=bwdist(ground.initialRoadResult.roadCellMask,'chessboard');
    T.initialRoadDistanceCells=double(distance(ids));
    for groupName=["stats","energyMaps"]
        fields=ground.(groupName);
        for field=string(fieldnames(fields)).'
            value=fields.(field);
            if (isnumeric(value)||islogical(value))&&isequal(size(value),ground.cellMapSize)
                T.(groupName+"_"+field)=double(value(ids));
            end
        end
    end
    writetable(T,fullfile(folder,sprintf('frame%d_cells.csv',frameIndex)));
    fprintf('Frame %d: reference %d, ground %d, proposals %d, selected %d\n',frameIndex,numel(fine),nnz(inGround),sum(count(ground.curbCellMask)),sum(count(current.curbCellMask)));
    % Evaluate removal of each enabled topology stage without retraining.
    rows={};fields=string(fieldnames(cfg.groundFeatures.curb));
    stages=fields(endsWith(fields,"Enabled"));
    for stage=["baseline";stages(:)]'
        if stage~="baseline" && ~cfg.groundFeatures.curb.(stage),continue;end
        trial=cfg;if stage~="baseline",trial.coarseProbabilityCloud.curbDisabledRefinementStages=stage;end
        a=perceiveFrame(frame,trial);b=a.diagnostics.ground;
        % Disabling adjacency omits its input-mask diagnostic. Restore that
        % unchanged pre-adjacency evidence for the existing model schema.
        if ~isfield(b.energyMaps,'rawExtractedMask')
            b.energyMaps.rawExtractedMask=b.energyMaps.extractedMask;
        end
        [b,~]=filterSemanticPillarCandidates(b,struct(),"curb",precision,a.diagnostics.groundPointContext,struct());
        selected=b.curbCellMask;
        rows{end+1,1}=struct('disabledStage',stage,'proposalCount',nnz(a.diagnostics.ground.curbCellMask), ...
            'proposalCovered',sum(count(a.diagnostics.ground.curbCellMask)), ...
            'selectedCount',nnz(selected),'falseCount',nnz(selected & reshape(count,ground.cellMapSize)==0), ...
            'covered',sum(count(selected))); %#ok<AGROW>
    end
    ablations=struct2table(vertcat(rows{:}));writetable(ablations,fullfile(folder,sprintf('frame%d_ablations.csv',frameIndex)));disp(ablations);
    save(fullfile(out,sprintf('frame%d_diagnostics.mat',frameIndex)),'p','frame','fine','cfg','count','current','-v7.3');
end
