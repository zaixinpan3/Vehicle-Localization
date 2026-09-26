function auditPoleFootprintLoss()
% auditPoleFootprintLoss: Separate shaft evidence from later cell competition.
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));addpath(folder);
    data=load(fullfile(root,'output','pole_miss_analysis_20260925','cases.mat'));
    sweep=load(fullfile(root,'output','pole_miss_analysis_20260925','gate_sweep.mat'));
    raw=load(fullfile(root,'output','pole_distribution_20260925','point_distributions.mat'));
    c=structuralPillarConfig(.6);c.useNativeKernels=true;c.pole.detector="subset";c.pole.probabilityEvidence="subset";
    relaxed=c;relaxed.pole.subset.minimumContrast=3;
    cloud=coarseSemanticProbabilityCloudConfig();cloud.semanticNames="pole";
    rows={};n=0;
    for k=1:numel(data.cases)
        r=data.cases{k};fine=double(raw.records{k}.fineCells);before=find(r.after.poleCellMask);
        v=find(cellfun(@(x)x.frame==r.frame && x.variant=="contrast3",sweep.cells),1);
        after=sweep.cells{v}.accepted;lost=intersect(setdiff(before,after),fine);
        originalSuppressed=intersect(setdiff(r.after.columnMaps.poleSubset.pillarIndices(r.after.columnMaps.poleSubset.found),before),fine);
        for mode=1:2
            if mode==1,targets=originalSuppressed;result=r.after;label="subset";else
                targets=lost;label="contrast3";if isempty(targets),continue;end
                grid=struct('points',r.points,'pointPillarLinIdx',r.pillarIds,'pointAttributes',struct(),'pillarGeometry',r.geometry);
                result=analyzeStructuralPillars(grid,relaxed,cloud);
            end
            e=result.columnMaps.poleSubset;components=bwconncomp(result.candidates.coreMask,8);map=labelmatrix(components);
            for target=targets(:).'
                j=find(e.pillarIndices==target);component=double(map(target));members=[];
                if component>0,members=components.PixelIdxList{component};end
                selected=members(result.poleCellMask(members));
                chosen=ismember(e.pillarIndices,selected);
                xy=e.axisXY(chosen,:)+(e.axisZ(j)-e.axisZ(chosen)).*e.slopeXY(chosen,:);
                distance=vecnorm(xy-e.axisXY(j,:),2,2);
                nearest=Inf;if ~isempty(distance),nearest=min(distance);end
                n=n+1;rows{n}=struct('frame',r.frame,'pillar',target,'variant',label, ...
                    'shaftFound',e.found(j),'accepted',result.poleCellMask(target), ...
                    'score',e.score(j),'ownPoints',e.ownCount(j),'coreComponentCells',numel(members), ...
                    'selectedComponentCells',numel(selected),'nearestSelectedAxisMeters',nearest, ...
                    'componentMembers',join(string(sort(members(:))).'," "), ...
                    'selectedCells',join(string(sort(selected(:))).'," ")); %#ok<AGROW>
            end
        end
    end
    T=struct2table(vertcat(rows{:}));writetable(T,fullfile(folder,'footprint_losses.csv'));disp(T(:,1:11));
end
