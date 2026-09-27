function captureMomentFeatures(dataset,label)
% captureMomentFeatures: Preserve joint XYZ structure without a finer grid.
    root=setupVehicleLocalization();folder=fullfile(root,'output','pole_geometry_20260927');
    s=load(fullfile(root,'output','pole_precision_20260927',[label '.mat']),'records','frames','cfg');cfg=s.cfg;
    tableIn=readtable(fullfile(folder,[label '_features.csv']),'TextType','string');
    file='MissisipiPointClouds.mat';if strcmpi(dataset,'Downtown'),file='downTownPointClouds.mat';end
    source=matfile(fullfile(root,'data','raw',file));rows={};started=tic;lastBlock=-1;
    for k=1:numel(s.frames)
        blockId=floor((s.frames(k)-1)/50);
        if blockId~=lastBlock
            first=blockId*50+1;last=min(first+49,max(s.frames));block=source.pointClouds(1,first:last);lastBlock=blockId;
        end
        g=pillarizePointCloud(block(s.frames(k)-first+1),cfg.voxel);ground=segmentGround(g,cfg.groundSegmentation);
        keep=~ismember(g.pointIndices,ground);q=double(g.points(keep,:));owners=double(g.pointPillarLinIdx(keep));geometry=g.pillarGeometry;
        for j=find(tableIn.frame==s.frames(k)).'
            owner=tableIn.pillar(j);a=tableIn.hypothesis(j);h=s.records{k}.hypotheses(a);
            [y,x]=ind2sub(geometry.mapSize,owner);lower=geometry.origin+([x y]-1).*geometry.cellSize;
            r=measurePillarPoleMoments(q(owners==owner,:),lower,geometry.cellSize,h);
            r.dataset=string(dataset);r.frame=s.frames(k);r.pillar=owner;r.hypothesis=a;rows{end+1,1}=r; %#ok<AGROW>
        end
        if mod(k,100)==0||k==numel(s.frames)
            T=struct2table(vertcat(rows{:}));writetable(T,fullfile(folder,[label '_moments.csv']));
            fprintf('%s moments %d/%d %.1f seconds\n',label,k,numel(s.frames),toc(started));
        end
    end
end
