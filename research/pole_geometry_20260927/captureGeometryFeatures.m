function captureGeometryFeatures(dataset,label)
% captureGeometryFeatures: Capture pre-veto shaft distributions and exact labels.
% References are joined only after raw point features have been measured.
    root=setupVehicleLocalization();folder=fullfile(root,'output','pole_geometry_20260927');
    addpath(fullfile(root,'research','pole_precision_20260927'));
    addpath(fullfile(root,'research','pole_context_20260927'));
    s=load(fullfile(root,'output','pole_precision_20260927',[label '.mat']), ...
        'records','references','frames','cfg');cfg=s.cfg;
    file='MissisipiPointClouds.mat';if strcmpi(dataset,'Downtown'),file='downTownPointClouds.mat';end
    source=matfile(fullfile(root,'data','raw',file));rows={};started=tic;lastBlock=-1;
    for k=1:numel(s.frames)
        blockId=floor((s.frames(k)-1)/50);
        if blockId~=lastBlock
            first=blockId*50+1;last=min(first+49,max(s.frames));block=source.pointClouds(1,first:last);lastBlock=blockId;
        end
        g=pillarizePointCloud(block(s.frames(k)-first+1),cfg.voxel);
        ground=segmentGround(g,cfg.groundSegmentation);keep=~ismember(g.pointIndices,ground);
        q=double(g.points(keep,:));owners=double(g.pointPillarLinIdx(keep));geometry=g.pillarGeometry;
        intensity=g.pointAttributes.intensity(keep);mask=~(isfinite(intensity) & intensity>cfg.offGroundFeatures.trafficSignIntensityThreshold);
        count=accumarray(owners,1,[prod(geometry.mapSize) 1]);evidence=reshape(count,geometry.mapSize);evidence(evidence<3)=0;
        [pointScore,lineScore]=buildPillarShapeScores(single(evidence),reshape(count>0,geometry.mapSize), ...
            cfg.offGroundFeatures,geometry.cellSize(1),geometry.cellSize(2));
        h=s.records{k}.hypotheses;
        for a=1:numel(h),h(a).geometryAccepted=true;end
        settings=cfg.offGroundFeatures.pole.validation;settings.minimumOwnerPoints=3;settings.minimumOwnerHeight=.15;
        [base,ids,hids]=measurePrecisionFeatures(q,owners,geometry,h,mask,pointScore,lineScore,settings);
        heightMap=nan(geometry.mapSize);
        for a=unique(hids).'
            shaft=h(a);near=all(abs(q(:,1:2)-shaft.axisXY)<=2,2);
            local=measureVerticalContext(q(near,:),mask(near),shaft);
            whole=measureVerticalContext(q(near,:),mask(near),shaft,false);
            for j=find(hids==a).'
                r=base{j};
                for key=fieldnames(local).',r.(key{1})=local.(key{1});end
                for key=fieldnames(whole).',r.(['whole_' key{1}])=whole.(key{1});end
                owner=ids(j);[y,x]=ind2sub(geometry.mapSize,owner);lower=geometry.origin+([x y]-1).*geometry.cellSize;
                own=q(owners==owner,:);xy=(own(:,1:2)-lower)./geometry.cellSize;
                mu=mean(xy,1);c=cov(xy);axis=(shaft.axisXY-lower)./geometry.cellSize;
                r.axisOwnerX=axis(1);r.axisOwnerY=axis(2);r.slopeX=shaft.slopeXY(1);r.slopeY=shaft.slopeXY(2);
                r.ownerMeanX=mu(1);r.ownerMeanY=mu(2);r.ownerVarianceX=c(1,1);r.ownerVarianceY=c(2,2);r.ownerCovarianceXY=c(1,2);
                r.ownerSkewX=mean((xy(:,1)-mu(1)).^3)/max(c(1,1)^1.5,eps);
                r.ownerSkewY=mean((xy(:,2)-mu(2)).^3)/max(c(2,2)^1.5,eps);
                [cc,rr]=meshgrid(max(1,x-1):min(geometry.mapSize(2),x+1),max(1,y-1):min(geometry.mapSize(1),y+1));
                neighbors=sub2ind(geometry.mapSize,rr(:),cc(:));
                for b=neighbors.'
                    if isnan(heightMap(b)),heightMap(b)=countSupport(q(owners==b & mask,3));end
                end
                others=heightMap(neighbors(neighbors~=owner));r.ownerContinuousHeight=heightMap(owner);
                r.ownerContextHeightFraction=heightMap(owner)/max(sum(heightMap(neighbors)),eps);
                r.neighborContinuousHeight=sum(others);r.maximumNeighborContinuousHeight=max([0;others]);
                r.dataset=string(dataset);r.frame=s.frames(k);r.pillar=owner;r.hypothesis=a;
                r.finePointCount=nnz(s.references{k}.pointPillarIds==owner);
                rows{end+1,1}=r; %#ok<AGROW>
            end
        end
        if mod(k,100)==0 || k==numel(s.frames)
            T=struct2table(vertcat(rows{:}));writetable(T,fullfile(folder,[label '_features.csv']));
            fprintf('%s geometry %d/%d: %d rows, %.1f seconds\n',label,k,numel(s.frames),height(T),toc(started));
        end
    end
end

function total=countSupport(z)
    if numel(z)<3,total=0;return;end
    [p,~,g]=unique([z-.25;z+.25]);n=numel(z);count=cumsum(accumarray(g,[ones(n,1);-ones(n,1)]));
    total=sum(diff(p).*(count(1:end-1)>=3));
end
