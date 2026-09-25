function T=captureAxialDistributions(settings,label)
% captureAxialDistributions: Evaluate geometry hypotheses without reference inputs.
    if nargin<1, settings=struct(); end
    if nargin<2, label='axial'; end
    root=setupVehicleLocalization(); folder=fullfile(root,'output','pole_distribution_20260925');
    data=load(fullfile(folder,'point_distributions.mat'),'records'); tables=cell(numel(data.records),1);
    for k=1:numel(data.records)
        r=data.records{k}; timer=tic;
        a=measureAxialPillars(r.points,r.pillarIds,r.geometry,settings); elapsed=toc(timer);
        t=struct2table(a); t.frame=repmat(r.frame,height(t),1); t.elapsed=repmat(elapsed,height(t),1);
        t.baseline=ismember(t.pillarIndices,r.baselineCells); t.fine=ismember(t.pillarIndices,r.fineCells);
        t.current=ismember(t.pillarIndices,r.currentCells); tables{k}=t;
        if mod(k,10)==0, fprintf('Axial distributions %d/%d, %.3f s/frame\n',k,numel(data.records),elapsed); end
    end
    T=vertcat(tables{:}); writetable(T,fullfile(folder,[label '.csv']));
    save(fullfile(folder,[label '.mat']),'T','settings','-v7.3');
end
