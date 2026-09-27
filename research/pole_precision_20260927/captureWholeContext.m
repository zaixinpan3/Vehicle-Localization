function captureWholeContext(dataset,label)
% captureWholeContext: Include vertical context beyond the proposed shaft span.
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));
    out=fullfile(root,'output','pole_precision_20260927');s=load(fullfile(out,[label '.mat']),'records','frames','cfg');
    file='MissisipiPointClouds.mat';if strcmpi(dataset,'Downtown'),file='downTownPointClouds.mat';end
    source=matfile(fullfile(root,'data','raw',file));cfg=s.cfg;
    rows={};started=tic;lastBlock=0;
    for k=1:numel(s.frames)
        blockId=floor((s.frames(k)-1)/50);
        if k==1 || blockId~=lastBlock
            first=blockId*50+1;last=min(first+49,max(s.frames));block=source.pointClouds(1,first:last);lastBlock=blockId;
        end
        frame=block(s.frames(k)-first+1);g=pillarizePointCloud(frame,cfg.voxel);
        ground=segmentGround(g,cfg.groundSegmentation);keep=~ismember(g.pointIndices,ground);
        q=double(g.points(keep,:));owners=double(g.pointPillarLinIdx(keep));geometry=g.pillarGeometry;
        intensity=g.pointAttributes.intensity(keep);mask=~(isfinite(intensity) & intensity>cfg.offGroundFeatures.trafficSignIntensityThreshold);
        record=s.records{k};hs=unique(record.hypothesisIds);heightMap=nan(geometry.mapSize);
        for a=hs.'
            h=record.hypotheses(a);near=all(abs(q(:,1:2)-h.axisXY)<=2,2);
            f=measureVerticalContext(q(near,:),mask(near),h,false);base=struct();
            for key=fieldnames(f).',base.(['whole_' key{1}])=f.(key{1});end
            for j=find(record.hypothesisIds==a).'
                r=base;owner=record.owners(j);[y,x]=ind2sub(geometry.mapSize,owner);
                [cc,rr]=meshgrid(max(1,x-1):min(geometry.mapSize(2),x+1),max(1,y-1):min(geometry.mapSize(1),y+1));
                neighbors=sub2ind(geometry.mapSize,rr(:),cc(:));
                for b=neighbors.'
                    if isnan(heightMap(b)),heightMap(b)=countSupport(q(owners==b & mask,3));end
                end
                total=sum(heightMap(neighbors));ownHeight=heightMap(owner);others=heightMap(neighbors(neighbors~=owner));
                r.ownerContinuousHeight=ownHeight;r.ownerContextHeightFraction=ownHeight/max(total,eps);
                r.neighborContinuousHeight=sum(others);r.maximumNeighborContinuousHeight=max([0;others]);
                r.dataset=string(dataset);r.frame=s.frames(k);r.pillar=owner;r.hypothesis=a;
                rows{end+1,1}=r; %#ok<AGROW>
            end
        end
        if mod(k,100)==0 || k==numel(s.frames)
            T=struct2table(vertcat(rows{:}));writetable(T,fullfile(folder,[label '_whole_features.csv']));
            fprintf('%s whole %d/%d %.1f seconds\n',label,k,numel(s.frames),toc(started));
        end
    end
end

function total=countSupport(z)
    if numel(z)<3,total=0;return;end
    [p,~,g]=unique([z-.25;z+.25]);n=numel(z);count=cumsum(accumarray(g,[ones(n,1);-ones(n,1)]));
    total=sum(diff(p).*(count(1:end-1)>=3));
end
