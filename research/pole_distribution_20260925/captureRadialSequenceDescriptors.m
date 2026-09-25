function captureRadialSequenceDescriptors(frames,label)
% captureRadialSequenceDescriptors: Full-pillar radial distributions on a drive.
    if nargin<1, frames=1:1170; end
    if nargin<2, label='radial_all'; end
    root=setupVehicleLocalization(); out=fullfile(root,'output','pole_distribution_20260925');
    cfg=perceptionConfig(); cfg.voxel.useNativeKernels=perceptionNativeAvailable;
    cfg.groundSegmentation.useNativeKernels=perceptionNativeAvailable;
    source=matfile(fullfile(root,'data','raw','MissisipiPointClouds.mat'));
    file=fullfile(out,[label '.csv']);
    for first=1:50:numel(frames)
        selected=first:min(first+49,numel(frames)); block=source.pointClouds(1,frames(selected)); tables=cell(numel(selected),1);
        for k=selected
            frame=block(k-first+1); grid=pillarizePointCloud(frame,cfg.voxel);
            ground=segmentGround(grid,cfg.groundSegmentation);
            groundMask=false(numel(frame.x),1); groundMask(ground)=true;
            off=~groundMask(grid.pointIndices); ids=double(grid.pointPillarLinIdx(off)); xyz=grid.points(off,:);
            [occupied,~,group]=unique(ids); counts=accumarray(group,1);
            zmin=accumarray(group,xyz(:,3),[],@min); zmax=accumarray(group,xyz(:,3),[],@max);
            keep=counts>=6 & zmax-zmin>=1;
            [~,~,~,peak]=computePillarDensityCore(xyz,ids,grid.pillarGeometry,.1,.15,.6,keep);
            timer=tic; d=computePillarRadialDistribution(xyz,ids,grid.pillarGeometry,peak,keep); elapsed=toc(timer);
            values=[repmat(frames(k),nnz(keep),1),occupied(keep),repmat(elapsed,nnz(keep),1)]; names={'frame','pillar','elapsed'};
            for name=setdiff(fieldnames(d),{'pillarIndices','radii'},'stable').'
                values=[values,d.(name{1})(keep,:)]; %#ok<AGROW>
                names=[names,cellstr(name{1}+"_"+string(round(100*d.radii)))]; %#ok<AGROW>
            end
            tables{k-first+1}=array2table(values,'VariableNames',names);
        end
        t=vertcat(tables{:});
        if first==1, writetable(t,file); else, writetable(t,file,'WriteMode','append'); end
        fprintf('%s %d/%d, %.3f s/frame\n',label,selected(end),numel(frames),elapsed);
    end
end
