function captureOwnerPlacement(dataset,label)
% captureOwnerPlacement: Full 0.6 m owner moments and normalized axis position.
% No finer partition or global XY coordinate is exported to inference.
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));
    out=fullfile(root,'output','pole_precision_20260927');s=load(fullfile(out,[label '.mat']),'records','frames','cfg');
    file='MissisipiPointClouds.mat';if strcmpi(dataset,'Downtown'),file='downTownPointClouds.mat';end
    source=matfile(fullfile(root,'data','raw',file));rows={};started=tic;
    for k=1:numel(s.frames)
        frame=source.pointClouds(1,s.frames(k));g=pillarizePointCloud(frame,s.cfg.voxel);
        ground=segmentGround(g,s.cfg.groundSegmentation);keep=~ismember(g.pointIndices,ground);
        q=double(g.points(keep,:));owners=double(g.pointPillarLinIdx(keep));geometry=g.pillarGeometry;
        record=s.records{k};
        for j=1:numel(record.owners)
            owner=record.owners(j);h=record.hypotheses(record.hypothesisIds(j));
            [y,x]=ind2sub(geometry.mapSize,owner);lower=geometry.origin+([x y]-1).*geometry.cellSize;
            p=q(owners==owner,:);xy=(p(:,1:2)-lower)./geometry.cellSize;
            mu=mean(xy,1);c=cov(xy);axis=(h.axisXY-lower)./geometry.cellSize;
            f=struct('axisOwnerX',axis(1),'axisOwnerY',axis(2), ...
                'slopeX',h.slopeXY(1),'slopeY',h.slopeXY(2), ...
                'ownerMeanX',mu(1),'ownerMeanY',mu(2),'ownerVarianceX',c(1,1), ...
                'ownerVarianceY',c(2,2),'ownerCovarianceXY',c(1,2), ...
                'ownerSkewX',mean((xy(:,1)-mu(1)).^3)/max(c(1,1)^1.5,eps), ...
                'ownerSkewY',mean((xy(:,2)-mu(2)).^3)/max(c(2,2)^1.5,eps));
            f.dataset=string(dataset);f.frame=s.frames(k);f.pillar=owner;f.hypothesis=record.hypothesisIds(j);
            rows{end+1,1}=f; %#ok<AGROW>
        end
        if mod(k,100)==0 || k==numel(s.frames)
            T=struct2table(vertcat(rows{:}));writetable(T,fullfile(folder,[label '_placement_features.csv']));
            fprintf('%s placement %d/%d %.1f seconds\n',label,k,numel(s.frames),toc(started));
        end
    end
end
