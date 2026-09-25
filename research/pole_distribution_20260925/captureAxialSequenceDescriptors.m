function captureAxialSequenceDescriptors(frames,label)
% captureAxialSequenceDescriptors: Stream entire drive through axial statistics.
% No reference labels enter the measurements. Join frozen labels separately.
    if nargin<1, frames=1:1170; end
    if nargin<2, label='axial_all'; end
    root=setupVehicleLocalization(); out=fullfile(root,'output','pole_distribution_20260925');
    if ~isfolder(out), mkdir(out); end
    cfg=perceptionConfig(); cfg.voxel.useNativeKernels=perceptionNativeAvailable;
    cfg.groundSegmentation.useNativeKernels=perceptionNativeAvailable;
    source=matfile(fullfile(root,'data','raw','MissisipiPointClouds.mat'));
    settings=struct('outerRadius',0.75,'minimumOwnHeight',1);
    file=fullfile(out,[label '.csv']);
    for first=1:25:numel(frames)
        selected=first:min(first+24,numel(frames)); block=source.pointClouds(1,frames(selected)); tables=cell(numel(selected),1);
        for k=selected
            frame=block(k-first+1); grid=pillarizePointCloud(frame,cfg.voxel);
            ground=segmentGround(grid,cfg.groundSegmentation);
            groundMask=false(numel(frame.x),1); groundMask(ground)=true;
            off=~groundMask(grid.pointIndices); ids=double(grid.pointPillarLinIdx(off));
            xyz=grid.points(off,:); timer=tic;
            a=measureAxialPillars(xyz,ids,grid.pillarGeometry,settings); elapsed=toc(timer);
            t=struct2table(a); t.frame=repmat(frames(k),height(t),1); t.elapsed=repmat(elapsed,height(t),1);
            counts=accumarray(findgroups(ids),1); zmin=accumarray(findgroups(ids),xyz(:,3),[],@min); zmax=accumarray(findgroups(ids),xyz(:,3),[],@max);
            tables{k-first+1}=t(counts>=6 & zmax-zmin>=1,:);
        end
        t=vertcat(tables{:});
        if first==1, writetable(t,file); else, writetable(t,file,'WriteMode','append'); end
        fprintf('%s %d/%d, %.3f s/frame\n',label,selected(end),numel(frames),elapsed);
    end
end
