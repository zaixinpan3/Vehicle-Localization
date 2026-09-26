function captureShaftStudy(label,cfg,indices)
% captureShaftStudy: Export every shaft mode and separate algorithmic references.
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));
    if nargin<1,label='prototype';end
    if nargin<2,cfg=pillarShaftConfig();end
    data=load(fullfile(root,'output','pole_miss_analysis_20260925','cases.mat'));
    reference=load(fullfile(root,'output','pole_distribution_20260925','point_distributions.mat'));
    if nargin<3,indices=1:numel(data.cases);end
    cfg.useNativeKernels=true;rows=cell(numel(indices),1);frames=rows;results=rows;
    for k=1:numel(indices)
        i=indices(k);r=data.cases{i};ref=reference.records{i};timer=tic;
        e=findPillarShaftModes(r.points,r.pillarIds,r.geometry,cfg);
        [assigned,groups]=assignPillarShaftSupport(r.points,r.pillarIds,r.geometry,e,cfg);elapsed=toc(timer);
        accepted=e.pillarIndices(e.found);t=struct2table(e);t.frame=repmat(r.frame,height(t),1);
        t.fine=ismember(e.pillarIndices,ref.fineCells);t.coarse=ismember(e.pillarIndices,ref.baselineCells);
        t.previous=ismember(e.pillarIndices,find(r.before.poleCellMask));
        t.subset=ismember(e.pillarIndices,find(r.after.poleCellMask));
        t.assigned=assigned.found;t.assignedScore=assigned.score;
        t.pointScore=r.before.columnMaps.pointScore(e.pillarIndices);
        t.lineScore=r.before.columnMaps.lineScore(e.pillarIndices);
        rows{k}=t;results{k}=e;
        frames{k}=struct('frame',r.frame,'seconds',elapsed,'candidates',numel(accepted), ...
            'fine',numel(ref.fineCells),'fineMatched',numel(intersect(accepted,ref.fineCells)), ...
            'coarse',numel(ref.baselineCells),'coarseMatched',numel(intersect(accepted,ref.baselineCells)), ...
            'assignedCount',nnz(assigned.found),'assignedFine',nnz(assigned.found & t.fine), ...
            'shaftGroups',numel(groups.representativeRows));
        if mod(k,20)==0,fprintf('%s %d/%d\n',label,k,numel(indices));end
    end
    out=fullfile(root,'output','pillar_shaft_20260926');if ~isfolder(out),mkdir(out);end
    tables=fullfile(out,'development_tables');if ~isfolder(tables),mkdir(tables);end
    writetable(vertcat(rows{:}),fullfile(tables,[label '_pillars.csv']));
    writetable(struct2table(vertcat(frames{:})),fullfile(folder,[label '_frames.csv']));
    save(fullfile(out,[label '.mat']),'results','frames','cfg','indices','-v7.3');
    f=vertcat(frames{:});fprintf('%s: %d candidates; %d/%d fine; median %.1f ms\n',label,sum([f.candidates]),sum([f.fineMatched]),sum([f.fine]),1000*median([f.seconds]));
end
