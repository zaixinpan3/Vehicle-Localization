function sweepPoleMissGates()
% sweepPoleMissGates: Quantify recovery and candidate expansion, changing one gate at a time.
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));addpath(folder);
    data=load(fullfile(root,'output','pole_miss_analysis_20260925','cases.mat'));
    raw=load(fullfile(root,'output','pole_distribution_20260925','point_distributions.mat'));
    old=structuralPillarConfig(.6);old.useNativeKernels=true;subset=old;
    subset.pole.detector="subset";subset.pole.probabilityEvidence="subset";
    configurations={old,old,old,subset,subset,subset,subset,subset,subset};
    names=["default","defaultIsolation0p2","defaultNoIsolation","subset", ...
        "height1p35","peak1p25","rms0p101","points11","contrast3"];
    configurations{2}.pole.minimumCoreIsolation=.2;configurations{3}.pole.minimumCoreIsolation=0;
    configurations{5}.pole.subset.minimumHeight=1.35;configurations{6}.pole.subset.minimumPeakContrast=1.25;
    configurations{7}.pole.subset.maximumRadialRms=.101;configurations{8}.pole.subset.minimumPoints=11;
    configurations{9}.pole.subset.minimumContrast=3;
    cloud=coarseSemanticProbabilityCloudConfig();cloud.semanticNames="pole";
    rows=cell(numel(data.cases)*numel(names),1);cells=rows;n=0;
    for v=1:numel(names)
        for k=1:numel(data.cases)
            r=data.cases{k};reference=raw.records{k};
            if v==1,result=r.before;elseif v==4,result=r.after;else
                grid=struct('points',r.points,'pointPillarLinIdx',r.pillarIds, ...
                    'pointAttributes',struct(),'pillarGeometry',r.geometry);
                result=analyzeStructuralPillars(grid,configurations{v},cloud);
            end
            accepted=find(result.poleCellMask);n=n+1;
            rows{n}=struct('variant',names(v),'frame',r.frame,'candidateCount',numel(accepted), ...
                'fineCount',numel(reference.fineCells),'fineShared',numel(intersect(accepted,reference.fineCells)), ...
                'baselineCount',numel(reference.baselineCells),'baselineShared',numel(intersect(accepted,reference.baselineCells)), ...
                'fineRecallTol',nnz(ismember(reference.fineCells,dilate(accepted,r.geometry.mapSize))), ...
                'baselineRecallTol',nnz(ismember(reference.baselineCells,dilate(accepted,r.geometry.mapSize))), ...
                'baselinePrecisionTolCount',nnz(ismember(accepted,dilate(reference.baselineCells,r.geometry.mapSize))));
            cells{n}=struct('variant',names(v),'frame',r.frame,'accepted',accepted);
        end
        fprintf('Counterfactual %s complete\n',names(v));
    end
    T=struct2table(vertcat(rows{:}));writetable(T,fullfile(folder,'gate_sweep_frames.csv'));
    save(fullfile(root,'output','pole_miss_analysis_20260925','gate_sweep.mat'),'cells','configurations','names','-v7.3');
end

function expanded=dilate(ids,mapSize)
    if isempty(ids),expanded=[];return;end
    [r,c]=ind2sub(mapSize,double(ids(:)));[dr,dc]=meshgrid(-1:1,-1:1);
    rr=r+dr(:).';cc=c+dc(:).';valid=rr>=1 & rr<=mapSize(1) & cc>=1 & cc<=mapSize(2);
    expanded=unique(sub2ind(mapSize,rr(valid),cc(valid)));
end
