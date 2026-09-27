function T=traceFineAlignmentExamples(examples)
% traceFineAlignmentExamples: Trace offline seed gates for development examples.
% This diagnostic invokes the existing fine stage, never the online detector.
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));
    if nargin<1,examples=[111 5343;71 5365;41 8135;491 4367;81 6231;301 4944];end
    assert(all(examples(:,1)<=780),'Use development frames for diagnosis.');
    source=matfile(fullfile(root,'data','raw','MissisipiPointClouds.mat'));
    cfg=perceptionConfig('Mississippi','offline');cfg.voxel.useNativeKernels=true;
    cfg.groundSegmentation.useNativeKernels=true;
    cc=coarseSemanticProbabilityCloudConfig(cfg.voxel);cc.semanticNames=cfg.featureNames;
    sc=fineStructuralConfig();common=intersect(fieldnames(sc),fieldnames(cfg.offGroundFeatures));
    for f=common.',sc.(f{1})=cfg.offGroundFeatures.(f{1});end
    sc.useNativeKernels=true;sc.poleOccupiedLayerMinPoints=cfg.fine.poleSupportMinimumPoints;
    target=struct('origin',[-29.9 -29.9],'cellSize',[.6 .6],'mapSize',[100 100]);
    addpath(fullfile(root,'output','pillar_fine_alignment_20260926','diagnostic'));
    rows={};objects={};snapshots=cell(size(examples,1),1);
    for k=1:size(examples,1)
        frame=source.pointClouds(1,examples(k,1));g=pillarizePointCloud(frame,cfg.voxel);
        ground=segmentGround(g,cfg.groundSegmentation);selected=~ismember(g.pointIndices,ground);
        p=g;p.points=g.points(selected,:);p.pointIndices=g.pointIndices(selected);
        p.pointPillarSub=g.pointPillarSub(selected,:);
        for name=fieldnames(g.pointAttributes).'
            p.pointAttributes.(name{1})=g.pointAttributes.(name{1})(selected,:);
        end
        v=voxelizePillars(p,cfg.fine.poleSupportHeightResolution,min(g.points(:,3)));
        fine=analyzeFineStructuralCandidates(v,sc,cc);m=fine.columnMaps;
        xy=p.points(:,1:2);bins=floor((xy-target.origin)./target.cellSize)+1;
        valid=all(bins>=1 & bins<=[100 100],2);coarse=zeros(size(bins,1),1);
        coarse(valid)=sub2ind([100 100],bins(valid,2),bins(valid,1));
        own=coarse==examples(k,2);fineIds=unique(sub2ind(m.mapSize, ...
            double(p.pointPillarSub(own,2)),double(p.pointPillarSub(own,1))));
        for j=fineIds.'
            row=struct('frame',examples(k,1),'coarsePillar',examples(k,2),'finePillar',j, ...
                'count',double(m.pillarCounts(j)),'run',double(m.runLayerMap(j)), ...
                'layers',double(m.occupiedLayerCount(j)),'pointScore',double(m.pointScore(j)), ...
                'lineScore',double(m.lineScore(j)),'seed',fine.candidates.coreSeedMask(j), ...
                'footprint',fine.candidates.footprintCandidateMask(j), ...
                'context',fine.candidates.contextCandidateMask(j),'candidate',fine.candidates.candidateMask(j), ...
                'postShape',fine.poleCellMask(j),'contextRatio',double(fine.candidates.contextRunLayerRatio(j)));
            rows{end+1,1}=row; %#ok<AGROW>
        end
        [r,c]=ind2sub(target.mapSize,examples(k,2));center=target.origin+([c r]-.5).*target.cellSize;
        near=vecnorm(p.points(:,1:2)-center,2,2)<1.2;
        pointCells=sub2ind(m.mapSize,double(p.pointPillarSub(:,2)),double(p.pointPillarSub(:,1)));
        candidate=fine.poleCellMask(pointCells);
        [~,traces]=traceFinePolePoints(p.points(candidate,:),cfg.fine,fine,v.gridConfig,p.points,false);
        for j=1:numel(traces)
            if hypot(traces(j).x-center(1),traces(j).y-center(2))>1.0,continue;end
            record=traces(j);record.frame=examples(k,1);record.coarsePillar=examples(k,2);
            objects{end+1,1}=record; %#ok<AGROW>
        end
        snapshots{k}=struct('frame',examples(k,1),'pillar',examples(k,2),'points',p.points(near,:), ...
            'intensity',p.pointAttributes.intensity(near),'originalIndices',p.pointIndices(near), ...
            'fineOffGround',fine,'geometry',v.gridConfig);
    end
    T=struct2table(vertcat(rows{:}));writetable(T,fullfile(folder,'development_seed_traces.csv'));
    out=fullfile(root,'output','pillar_fine_alignment_20260926');
    objectTable=struct2table(vertcat(objects{:}));
    writetable(objectTable,fullfile(folder,'development_validator_traces.csv'));
    save(fullfile(out,'development_examples.mat'),'snapshots','examples','T','objectTable','-v7.3');disp(objectTable);
end
