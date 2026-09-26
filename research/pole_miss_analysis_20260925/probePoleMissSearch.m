function probePoleMissSearch()
% probePoleMissSearch: Counterfactual search/gate probes on the ten missed fine cells.
% These use all original off-ground returns; fine labels only select the cases.
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));addpath(folder);
    data=load(fullfile(root,'output','pole_miss_analysis_20260925','cases.mat'));
    tableData=readtable(fullfile(folder,'reference_cell_causes.csv'));
    missed=tableData(tableData.fine & ~tableData.subsetAccepted,:);
    cfg=structuralPillarConfig(.6);base=cfg.pole.subset;base.useNativeKernels=true;
    variants={"base",base;"search48x12",withParameter(withParameter(base,'maximumSeeds',48),'heightHypotheses',12); ...
        "fit0p08",withParameter(base,'fitRadius',.08);"fit0p10",withParameter(base,'fitRadius',.10); ...
        "fit0p20",withParameter(base,'fitRadius',.20);"height1p35",withParameter(base,'minimumHeight',1.35); ...
        "peak1p25",withParameter(base,'minimumPeakContrast',1.25);"rms0p101",withParameter(base,'maximumRadialRms',.101); ...
        "points11",withParameter(base,'minimumPoints',11);"contrast3",withParameter(base,'minimumContrast',3)};
    rows=cell(height(missed)*size(variants,1),1);n=0;
    for k=1:numel(data.cases)
        r=data.cases{k};targets=missed.pillarIndices(missed.frame==r.frame);if isempty(targets),continue;end
        occupied=unique(r.pillarIds);selected=ismember(occupied,targets);
        for v=1:size(variants,1)
            e=findPillarPoleSubsets(r.points,r.pillarIds,r.geometry,variants{v,2},selected);
            for j=find(selected).'
                n=n+1;rows{n}=struct('frame',r.frame,'pillar',occupied(j),'variant',variants{v,1}, ...
                    'found',e.found(j),'score',e.score(j),'ownCount',e.ownCount(j), ...
                    'supportCount',e.supportCount(j),'height',e.height(j),'ownHeight',e.ownHeight(j), ...
                    'robustHeight',e.robustHeight(j),'radialRms',e.radialRms(j),'peakContrast',e.peakContrast(j));
            end
        end
    end
    T=struct2table(vertcat(rows{1:n}));writetable(T,fullfile(folder,'missed_case_counterfactuals.csv'));
    disp(T(T.found,{'frame','pillar','variant','score','height','radialRms'}));
end

function cfg=withParameter(cfg,name,value)
    cfg.(name)=value;
end
