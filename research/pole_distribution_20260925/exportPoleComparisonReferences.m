function exportPoleComparisonReferences()
% exportPoleComparisonReferences: Export full evaluation denominators and seeds.
% Frozen labels and previous outputs are never used to measure distributions.
    root=setupVehicleLocalization(); out=fullfile(root,'output','pole_distribution_20260925');
    folder=fullfile(root,'output','coarse_lattice_20260924');
    baseline=load(fullfile(folder,'baseline_sequence.mat'),'records','geometry');
    current=load(fullfile(folder,'metricCoreFull_sequence.mat'),'records','geometry');
    g=current.geometry; baseRows=cell(numel(baseline.records),1); currentRows=baseRows;
    for k=1:numel(baseline.records)
        b=baseline.records{k}; ids=double(b.pillarIndices{b.semanticNames=="pole"});
        [r,c]=ind2sub(baseline.geometry.mapSize,ids);
        xy=baseline.geometry.origin+([c r]-.5).*baseline.geometry.cellSize;
        bins=floor((xy-g.origin)./g.cellSize)+1;
        inside=all(bins>=1,2)&bins(:,1)<=g.mapSize(2)&bins(:,2)<=g.mapSize(1);
        ids=unique(sub2ind(g.mapSize,bins(inside,2),bins(inside,1)));
        baseRows{k}=[repmat(b.frame,numel(ids),1),ids];
        r=current.records{k}; ids=double(r.pillarIndices{r.semanticNames=="pole"});
        currentRows{k}=[repmat(r.frame,numel(ids),1),ids];
    end
    writetable(array2table(vertcat(baseRows{:}),'VariableNames',{'frame','pillar'}),fullfile(out,'baseline_cells.csv'));
    writetable(array2table(vertcat(currentRows{:}),'VariableNames',{'frame','pillar'}),fullfile(out,'current_cells.csv'));
    fine=load(fullfile(folder,'fine_pole_pillars.mat'),'fine'); rows=cell(numel(fine.fine),1);
    for k=1:numel(fine.fine)
        ids=double(fine.fine(k).polePillars(:)); rows{k}=[repmat(fine.fine(k).frame,numel(ids),1),ids];
    end
    writetable(array2table(vertcat(rows{:}),'VariableNames',{'frame','pillar'}),fullfile(out,'fine_cells.csv'));

end
