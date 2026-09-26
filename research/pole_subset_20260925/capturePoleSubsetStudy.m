function summary=capturePoleSubsetStudy(label,modify)
% capturePoleSubsetStudy: Evaluate existential support on cached raw pillars.
    if nargin<1,label='subset';end
    if nargin<2,modify=[];end
    root=setupVehicleLocalization(); d=load(fullfile(root,'output','pole_distribution_20260925','point_distributions.mat'));
    source=matfile(fullfile(root,'data','raw','MissisipiPointClouds.mat'));
    frames=cellfun(@(r)r.frame,d.records);block=source.pointClouds(1,frames);
    cfg=structuralPillarConfig(.6);cfg.pole.detector="subset";cfg.pole.probabilityEvidence="subset";
    if ~isempty(modify),cfg=modify(cfg);end
    clouds=coarseSemanticProbabilityCloudConfig(); clouds.semanticNames="pole";
    records=cell(numel(d.records),1); tables=records; timing=zeros(numel(records),1);
    for k=1:numel(records)
        % Rehydrate original precision: the diagnostic cache stores single
        % XYZ, which can change ties between equally supported hypotheses.
        r=d.records{k};f=block(k);index=double(r.originalIndices);
        points=double([f.x(index),f.y(index),f.z(index)]);
        grid=struct('points',points,'pointPillarLinIdx',r.pillarIds, ...
            'pointAttributes',struct(),'pillarGeometry',r.geometry);
        timer=tic; result=analyzeStructuralPillars(grid,cfg,clouds);timing(k)=toc(timer);
        records{k}=struct('frame',r.frame,'detected',find(result.poleCellMask),'baseline',r.baselineCells,'fine',r.fineCells,'previous',r.currentCells);
        e=result.columnMaps.poleSubset; t=struct2table(rmfield(e,{'axisXY','slopeXY'})); t.frame=repmat(r.frame,height(t),1);
        t.pillarPointCount=result.columnMaps.statistics.count;
        t.ownSupportFraction=e.ownCount./max(t.pillarPointCount,1); % Diagnostic only; never a gate.
        t.accepted=ismember(t.pillarIndices,records{k}.detected);t.baseline=ismember(t.pillarIndices,r.baselineCells);t.fine=ismember(t.pillarIndices,r.fineCells);t.previous=ismember(t.pillarIndices,r.currentCells);
        tables{k}=t(e.found|t.baseline|t.fine|t.previous,:);
        if mod(k,10)==0,fprintf('%s %d/%d, %.3f s\n',label,k,numel(records),timing(k));end
    end
    out=fullfile(root,'output','pole_subset_20260925');if ~isfolder(out),mkdir(out);end
    save(fullfile(out,[label '.mat']),'records','cfg','timing','-v7.3');
    writetable(vertcat(tables{:}),fullfile(root,'research','pole_subset_20260925',[label '_pillars.csv']));
    summary=struct('medianMs',1000*median(timing),'detected',sum(cellfun(@(r)numel(r.detected),records)));disp(summary);
end
