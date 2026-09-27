function T=exportDevelopmentRanges()
% exportDevelopmentRanges: Export raw sensor-range statistics for ablation.
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));
    previous=load(fullfile(root,'output','pole_context_20260927','development.mat'),'cache');rows={};
    for k=1:numel(previous.cache)
        c=previous.cache{k};
        for j=1:numel(c.rows)
            r=c.rows{j};h=c.hypotheses(r.hypothesis);
            rows{end+1,1}=struct('frame',r.frame,'pillar',r.pillar, ...
                'hypothesis',r.hypothesis,'axisRange',norm(h.axisXY)); %#ok<AGROW>
        end
    end
    T=struct2table(vertcat(rows{:}));writetable(T,fullfile(folder,'development_ranges.csv'));
end
