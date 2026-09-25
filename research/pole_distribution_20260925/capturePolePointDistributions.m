function capturePolePointDistributions(frames)
% capturePolePointDistributions: Raw whole-pillar geometry and separate references.
% Fine masks are diagnostic references only, never detector inputs.
    if nargin<1, frames=1:10:1170; end
    root=setupVehicleLocalization(); out=fullfile(root,'output','pole_distribution_20260925');
    if ~isfolder(out), mkdir(out); end
    baseline=load(fullfile(root,'output','coarse_lattice_20260924','baseline_sequence.mat'),'records','geometry');
    fine=load(fullfile(root,'output','coarse_lattice_20260924','fine_pole_pillars.mat'));
    cfg=perceptionConfig(); cfg.featureNames="pole"; cfg.coarseProbabilityCloud.storeDiagnostics=true;
    cfg.voxel.useNativeKernels=perceptionNativeAvailable;
    cfg.groundSegmentation.useNativeKernels=perceptionNativeAvailable;
    source=matfile(fullfile(root,'data','raw','MissisipiPointClouds.mat'));
    records=cell(numel(frames),1);
    for first=1:25:numel(frames)
        selected=first:min(first+24,numel(frames)); block=source.pointClouds(1,frames(selected));
        for k=selected
            frame=block(k-first+1); grid=pillarizePointCloud(frame,cfg.voxel);
            ground=segmentGround(grid,cfg.groundSegmentation);
            groundMask=false(numel(frame.x),1); groundMask(ground)=true;
            off=~groundMask(grid.pointIndices); ids=double(grid.pointPillarLinIdx(off));
            xyz=grid.points(off,:); originals=grid.pointIndices(off);
            [ids,order]=sort(ids); xyz=xyz(order,:); originals=originals(order);
            p=perceiveFrame(frame,cfg); g=grid.pillarGeometry;
            b=baseline.records{frames(k)};
            bIds=double(b.pillarIndices{b.semanticNames=="pole"});
            [r,c]=ind2sub(baseline.geometry.mapSize,bIds);
            xy=baseline.geometry.origin+([c r]-0.5).*baseline.geometry.cellSize;
            bins=floor((xy-g.origin)./g.cellSize)+1;
            inside=all(bins>=1,2)&bins(:,1)<=g.mapSize(2)&bins(:,2)<=g.mapSize(1);
            projected=unique(sub2ind(g.mapSize,bins(inside,2),bins(inside,1)));
            fj=find([fine.fine.frame]==frames(k),1);
            fineCells=[]; finePointIds=[]; signCells=[];
            if ~isempty(fj)
                fineCells=fine.fine(fj).polePillars;
                finePointIds=fine.fine(fj).polePoints;
                signCells=fine.fine(fj).signPillars;
            end
            records{k}=struct('frame',frames(k),'points',single(xyz),'pillarIds',int32(ids), ...
                'originalIndices',originals,'ring',uint16(mod(double(originals)-1,size(frame.x,1))+1), ...
                'baselineCells',int32(projected),'fineCells',int32(fineCells), ...
                'finePointMask',ismember(originals,finePointIds),'signCells',int32(signCells), ...
                'currentCells',p.candidates.pillarIndices{1},'geometry',g);
        end
        fprintf('Point distributions %d/%d\n',selected(end),numel(frames));
    end
    save(fullfile(out,'point_distributions.mat'),'records','cfg','-v7.3');
end
