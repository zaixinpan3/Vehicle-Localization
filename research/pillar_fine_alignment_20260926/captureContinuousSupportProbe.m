function records=captureContinuousSupportProbe(cases,assignments,label,settings)
% captureContinuousSupportProbe: Development-only raw-point support measurements.
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));
    if nargin<3,label='continuous_support';end
    if nargin<4,settings=struct('radii',[.20 .25 .30 .35],'contextRadius',.75, ...
            'halfWindow',.25,'minimumWindowPoints',3,'minimumWindowFraction',.60);end
    selected=find(cellfun(@(c)c.frame<=780,cases));records=cell(numel(selected),1);rows={};
    for k=1:numel(selected)
        j=selected(k);c=cases{j};timer=tic;
        mask=true(size(c.points,1),1);if isfield(c,'structuralMask'),mask=c.structuralMask;end
        h=probeContinuousPoleSupport(c.points,c.pillarIds,c.geometry,assignments{j}.assigned,settings,mask);
        records{k}=struct('frame',c.frame,'hypotheses',h,'fineIds',c.fineIds,'milliseconds',1000*toc(timer));
        for b=1:numel(h)
            t=rmfield(h(b),{'axisXY','axisZ','slopeXY','ownerIds','ownerCount','ownerHeight'});
            t.frame=c.frame;t.hypothesis=b;
            for owner=1:numel(h(b).ownerIds)
                v=t;v.ownerId=h(b).ownerIds(owner);v.ownerCount=h(b).ownerCount(owner);v.ownerHeight=h(b).ownerHeight(owner);
                v.pointScore=double(c.before.columnMaps.pointScore(v.ownerId));
                v.coreIsolation=double(c.before.columnMaps.coreIsolation(v.ownerId));
                v.wholeHeight=c.before.columnMaps.pillarZRange(v.ownerId);
                v.finePointCount=nnz(c.fineIds==v.ownerId);rows{end+1,1}=v; %#ok<AGROW>
            end
        end
        if mod(k,10)==0,fprintf('Continuous support probe %d/%d\n',k,numel(selected));end
    end
    T=struct2table(vertcat(rows{:}));writetable(T,fullfile(folder,[label '_hypotheses.csv']));
    save(fullfile(root,'output','pillar_fine_alignment_20260926',[label '_probe.mat']),'records','T','settings','-v7.3');
end
