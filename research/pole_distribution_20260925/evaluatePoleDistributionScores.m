function T=evaluatePoleDistributionScores()
% evaluatePoleDistributionScores: Compare evidence ranking with fine references.
% The fine detector is an imperfect reference; no physical accuracy is claimed.
    root=setupVehicleLocalization(); folder=fullfile(root,'output','pole_distribution_20260925');
    d=load(fullfile(folder,'point_distributions.mat'),'records');
    previous=load(fullfile(root,'output','coarse_lattice_20260924','metricCoreFull_sequence.mat'),'records');
    rows=cell(numel(d.records),1);
    for k=1:numel(d.records)
        r=d.records{k}; occupied=unique(double(r.pillarIds)); selected=ismember(occupied,r.currentCells);
        [~,~,~,peak]=computePillarDensityCore(r.points,r.pillarIds,r.geometry,.1,.15,.6,selected);
        timer=tic; e=scorePolePillarDistributions(r.points,r.pillarIds,r.geometry,peak,selected); elapsed=toc(timer);
        old=previous.records{r.frame}; [found,index]=ismember(occupied(selected),double(old.poleCells)); assert(all(found));
        t=table(repmat(r.frame,nnz(selected),1),occupied(selected), ...
            ismember(occupied(selected),r.fineCells),ismember(occupied(selected),r.baselineCells), ...
            double(old.poleProbability(index)),e.score(selected),e.concentration(selected),e.continuity(selected), ...
            e.robustHeight(selected),e.coreCount(selected),repmat(elapsed,nnz(selected),1), ...
            'VariableNames',{'frame','pillar','fine','baseline','oldProbability','distributionScore','concentration','continuity','robustHeight','coreCount','elapsed'});
        rows{k}=t;
    end
    T=vertcat(rows{:}); writetable(T,fullfile(root,'research','pole_distribution_20260925','pole_evidence_scores.csv'));
end
