function T=capturePrecisionTraining(dataset,frames,label)
% capturePrecisionTraining: Freeze distribution features before joining labels.
% Original fine detection is used only to create additional offline references.
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));
    addpath(fullfile(root,'research','pole_context_20260927'));
    addpath(fullfile(root,'research','pillar_fine_alignment_20260926'));
    out=fullfile(root,'output','pole_precision_20260927');old=fullfile(root,'output','pillar_fine_alignment_20260926');
    cfg=perceptionConfig(dataset);cfg.voxel.useNativeKernels=true;cfg.groundSegmentation.useNativeKernels=true;
    cfg.offGroundFeatures.useNativeKernels=true;
    % Keep this capture on the original broad proposal profile after a newer
    % production distribution validator is installed.
    if isfield(cfg.offGroundFeatures.pole,'distributionValidation')
        cfg.offGroundFeatures.pole.distributionValidation.enabled=false;
    end
    cfg.offGroundFeatures.pole.validation.sparseSupportPointThreshold=0;
    cfg.offGroundFeatures.pole.validation.sparseOwnerPointThreshold=0;
    cloud=coarseSemanticProbabilityCloudConfig(cfg.voxel);cloud.semanticNames="pole";
    if strcmpi(dataset,'Downtown')
        reference=load(fullfile(old,'downtown_reference.mat'),'references','frames');
        source=matfile(fullfile(root,'data','raw','downTownPointClouds.mat'));
    else
        reference=load(fullfile(old,'reference.mat'),'references');reference.frames=1:1170;
        source=matfile(fullfile(root,'data','raw','MissisipiPointClouds.mat'));
    end
    fineCfg=perceptionConfig(dataset,'offline');rows={};denominators={};records=cell(numel(frames),1);references=records;
    started=tic;
    for k=1:numel(frames)
        if mod(k-1,50)==0
            selected=frames(k:min(k+49,numel(frames)));block=[];
            if numel(selected)<3 || all(diff(selected)==selected(2)-selected(1)),block=source.pointClouds(1,selected);end
        end
        if isempty(block),frame=source.pointClouds(1,frames(k));else,frame=block(mod(k-1,50)+1);end
        g=pillarizePointCloud(frame,cfg.voxel);ground=segmentGround(g,cfg.groundSegmentation);
        selected=~ismember(double(g.pointIndices),double(ground));
        p=struct('points',g.points(selected,:),'pointPillarLinIdx',g.pointPillarLinIdx(selected), ...
            'pillarGeometry',g.pillarGeometry,'pointAttributes',struct());
        for name=fieldnames(g.pointAttributes).',p.pointAttributes.(name{1})=g.pointAttributes.(name{1})(selected,:);end
        off=analyzeStructuralPillars(p,cfg.offGroundFeatures,cloud);m=off.columnMaps;
        mask=~(isfinite(p.pointAttributes.intensity) & p.pointAttributes.intensity>cfg.offGroundFeatures.trafficSignIntensityThreshold);
        [~,h]=validatePillarPoleSupport(p.points,p.pointPillarLinIdx,p.pillarGeometry,m.poleValidationProposals, ...
            cfg.offGroundFeatures.pole.validation,mask,double(m.pointScore(double(m.statistics.pillarIndices))));
        [features,owners,hypIds]=measurePrecisionFeatures(p.points,p.pointPillarLinIdx,p.pillarGeometry,h,mask,m.pointScore,m.lineScore,cfg.offGroundFeatures.pole.validation);
        % Frozen labels enter only here, after every candidate feature exists.
        j=find(reference.frames==frames(k),1);
        if isempty(j)
            fine=perceiveFrame(frame,fineCfg);pointIndices=find(fine.featureMasks.pole);
            [~,detail]=measureFinePoleAlignment(frame,pointIndices,[],g.pillarGeometry);
            xyz=double([frame.x(:),frame.y(:),frame.z(:)]);
            ref=struct('frame',frames(k),'pointIndices',pointIndices,'pointPillarIds',detail.finePointPillarIds, ...
                'finePillarIds',detail.finePillarIds,'points',xyz(pointIndices,:));
        else
            ref=reference.references{j};
        end
        ids=ref.pointPillarIds;references{k}=ref;
        for j=1:numel(features)
            r=features{j};r.dataset=string(dataset);r.frame=frames(k);r.pillar=owners(j);r.hypothesis=hypIds(j);r.finePointCount=nnz(ids==owners(j));
            rows{end+1,1}=r; %#ok<AGROW>
        end
        denominators{k,1}=struct('dataset',string(dataset),'frame',frames(k),'finePoints',nnz(ids>0),'finePillars',numel(unique(ids(ids>0)))); %#ok<AGROW>
        records{k}=struct('frame',frames(k),'owners',owners,'hypothesisIds',hypIds,'hypotheses',h,'candidateIds',find(off.poleCellMask));
        assert(isequal(unique(owners),find(off.poleCellMask)),'Feature export must cover all broad-profile candidate owners.');
        if mod(k,50)==0 || k==numel(frames)
            T=struct2table(vertcat(rows{:}));D=struct2table(vertcat(denominators{:}));processed=k;
            writetable(T,fullfile(folder,[label '_features.csv']));writetable(D,fullfile(folder,[label '_frames.csv']));
            save(fullfile(out,[label '.mat']),'records','references','frames','cfg','fineCfg','T','D','processed','-v7.3');
            fprintf('%s %d/%d: %d rows, %.1f seconds\n',label,k,numel(frames),height(T),toc(started));
        end
    end
end
