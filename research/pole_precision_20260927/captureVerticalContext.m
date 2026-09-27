function captureVerticalContext(dataset,label)
% captureVerticalContext: Extend frozen candidates with label-free context.
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));
    out=fullfile(root,'output','pole_precision_20260927');
    s=load(fullfile(out,[label '.mat']),'records','frames','cfg');
    file='MissisipiPointClouds.mat';if strcmpi(dataset,'Downtown'),file='downTownPointClouds.mat';end
    source=matfile(fullfile(root,'data','raw',file));cfg=s.cfg;
    cloud=coarseSemanticProbabilityCloudConfig(cfg.voxel);cloud.semanticNames=["pole","facade"];
    rows={};started=tic;
    for k=1:numel(s.frames)
        frame=source.pointClouds(1,s.frames(k));g=pillarizePointCloud(frame,cfg.voxel);
        ground=segmentGround(g,cfg.groundSegmentation);keep=~ismember(g.pointIndices,ground);
        p=struct('points',g.points(keep,:),'pointPillarLinIdx',g.pointPillarLinIdx(keep), ...
            'pillarGeometry',g.pillarGeometry,'pointAttributes',struct());
        for name=fieldnames(g.pointAttributes).',p.pointAttributes.(name{1})=g.pointAttributes.(name{1})(keep,:);end
        off=analyzeStructuralPillars(p,cfg.offGroundFeatures,cloud);
        mask=~(isfinite(p.pointAttributes.intensity) & p.pointAttributes.intensity>cfg.offGroundFeatures.trafficSignIntensityThreshold);
        record=s.records{k};hs=unique(record.hypothesisIds);
        for a=hs.'
            h=record.hypotheses(a);near=all(abs(p.points(:,1:2)-h.axisXY)<=2,2);
            f=measureVerticalContext(p.points(near,:),mask(near),h);
            for j=find(record.hypothesisIds==a).'
                r=f;owner=record.owners(j);r.ownerFacade=double(off.facadeCellMask(owner));
                [y,x]=ind2sub(g.pillarGeometry.mapSize,owner);
                neighborhood=off.facadeCellMask(max(1,y-1):min(end,y+1),max(1,x-1):min(end,x+1));
                r.neighborFacadeCount=nnz(neighborhood);
                r.dataset=string(dataset);r.frame=s.frames(k);r.pillar=owner;r.hypothesis=a;
                rows{end+1,1}=r; %#ok<AGROW>
            end
        end
        if mod(k,100)==0 || k==numel(s.frames)
            T=struct2table(vertcat(rows{:}));writetable(T,fullfile(folder,[label '_vertical_features.csv']));
            fprintf('%s vertical %d/%d %.1f seconds\n',label,k,numel(s.frames),toc(started));
        end
    end
end
