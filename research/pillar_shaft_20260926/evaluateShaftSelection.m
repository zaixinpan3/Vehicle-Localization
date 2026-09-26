function evaluateShaftSelection(label)
% evaluateShaftSelection: Measure axis attribution and contextual confidence.
    root=setupVehicleLocalization();
    data=load(fullfile(root,'output','pole_miss_analysis_20260925','cases.mat'));
    reference=load(fullfile(root,'output','pole_distribution_20260925','point_distributions.mat'));
    modes=load(fullfile(root,'output','pillar_shaft_20260926',[label '.mat']));
    rows=cell(numel(data.cases),1);results=rows;
    for k=1:numel(data.cases)
        r=data.cases{k};e=modes.results{k};ids=e.pillarIndices;cfg=pillarShaftConfig();
        e.found=e.found & (e.score>=cfg.strongScore | e.score.*double(r.before.columnMaps.pointScore(ids))>=cfg.minimumShapeProduct);
        [a,g]=assignPillarShaftSupport(r.points,r.pillarIds,r.geometry,e,cfg);
        accepted=union(ids(a.found),find(r.before.poleCellMask));ref=reference.records{k};
        t=struct2table(a);t.frame=repmat(r.frame,height(t),1);t.fine=ismember(ids,ref.fineCells);
        t.coarse=ismember(ids,ref.baselineCells);t.accepted=ismember(ids,accepted);t.previous=r.before.poleCellMask(ids);
        rows{k}=t(t.accepted | t.fine | t.coarse,:);
        results{k}=struct('frame',r.frame,'accepted',accepted,'groups',g,'evidence',a);
    end
    T=vertcat(rows{:});writetable(T,fullfile(root,'output','pillar_shaft_20260926','development_tables',[label '_selected.csv']));
    save(fullfile(root,'output','pillar_shaft_20260926',[label '_selected.mat']),'results','cfg','-v7.3');
    fprintf('%s selected %d, fine %d/%d, retained previous %d/%d\n',label,nnz(T.accepted),nnz(T.accepted&T.fine),nnz(T.fine),nnz(T.accepted&T.previous),nnz(T.previous));
end
