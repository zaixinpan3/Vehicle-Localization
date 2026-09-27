function [selected,diagnostic]=detectPrecisionPillars(frame,cfg,model)
% detectPrecisionPillars: Research classifier using only shared coarse inputs.
% This rejected/experimental profile is not installed in perceiveFrame.
% Frozen fine labels, frame identity and a fine XY index are never read here.
    assert(all(cfg.voxel.voxelSize==.6),'This study requires the shared 0.6 m grid.');
    cfg.voxel.useNativeKernels=true;cfg.groundSegmentation.useNativeKernels=true;
    cfg.offGroundFeatures.useNativeKernels=true;
    cfg.offGroundFeatures.pole.validation.sparseSupportPointThreshold=0;
    cfg.offGroundFeatures.pole.validation.sparseOwnerPointThreshold=0;
    g=pillarizePointCloud(frame,cfg.voxel);ground=segmentGround(g,cfg.groundSegmentation);
    keep=~ismember(g.pointIndices,ground);
    p=struct('points',g.points(keep,:),'pointPillarLinIdx',g.pointPillarLinIdx(keep), ...
        'pillarGeometry',g.pillarGeometry,'pointAttributes',struct());
    for name=fieldnames(g.pointAttributes).',p.pointAttributes.(name{1})=g.pointAttributes.(name{1})(keep,:);end
    cloud=coarseSemanticProbabilityCloudConfig(cfg.voxel);cloud.semanticNames=["pole","facade"];
    off=analyzeStructuralPillars(p,cfg.offGroundFeatures,cloud);m=off.columnMaps;
    mask=~(isfinite(p.pointAttributes.intensity) & p.pointAttributes.intensity>cfg.offGroundFeatures.trafficSignIntensityThreshold);
    [~,hypotheses]=validatePillarPoleSupport(p.points,p.pointPillarLinIdx,p.pillarGeometry,m.poleValidationProposals, ...
        cfg.offGroundFeatures.pole.validation,mask,double(m.pointScore(double(m.statistics.pillarIndices))));
    [base,owners,ids]=measurePrecisionFeatures(p.points,p.pointPillarLinIdx,p.pillarGeometry,hypotheses,mask, ...
        m.pointScore,m.lineScore,cfg.offGroundFeatures.pole.validation);
    features=zeros(numel(base),numel(model.featureNames));q=double(p.points);pointOwners=double(p.pointPillarLinIdx);
    geometry=p.pillarGeometry;heightMap=nan(geometry.mapSize);
    for a=unique(ids).'
        h=hypotheses(a);near=all(abs(q(:,1:2)-h.axisXY)<=2,2);
        local=measureVerticalContext(q(near,:),mask(near),h);
        whole=measureVerticalContext(q(near,:),mask(near),h,false);
        for j=find(ids==a).'
            r=base{j};
            for key=fieldnames(local).',r.(key{1})=local.(key{1});end
            for key=fieldnames(whole).',r.(['whole_' key{1}])=whole.(key{1});end
            owner=owners(j);[y,x]=ind2sub(geometry.mapSize,owner);lower=geometry.origin+([x y]-1).*geometry.cellSize;
            own=q(pointOwners==owner,:);xy=(own(:,1:2)-lower)./geometry.cellSize;
            mu=mean(xy,1);c=cov(xy);axis=(h.axisXY-lower)./geometry.cellSize;
            r.axisOwnerX=axis(1);r.axisOwnerY=axis(2);r.slopeX=h.slopeXY(1);r.slopeY=h.slopeXY(2);
            r.ownerMeanX=mu(1);r.ownerMeanY=mu(2);r.ownerVarianceX=c(1,1);r.ownerVarianceY=c(2,2);r.ownerCovarianceXY=c(1,2);
            r.ownerSkewX=mean((xy(:,1)-mu(1)).^3)/max(c(1,1)^1.5,eps);
            r.ownerSkewY=mean((xy(:,2)-mu(2)).^3)/max(c(2,2)^1.5,eps);
            [cc,rr]=meshgrid(max(1,x-1):min(geometry.mapSize(2),x+1),max(1,y-1):min(geometry.mapSize(1),y+1));
            neighbors=sub2ind(geometry.mapSize,rr(:),cc(:));
            r.ownerFacade=double(off.facadeCellMask(owner));r.neighborFacadeCount=nnz(off.facadeCellMask(neighbors));
            for b=neighbors.'
                if isnan(heightMap(b)),heightMap(b)=countSupport(q(pointOwners==b & mask,3));end
            end
            total=sum(heightMap(neighbors));ownHeight=heightMap(owner);others=heightMap(neighbors(neighbors~=owner));
            r.ownerContinuousHeight=ownHeight;r.ownerContextHeightFraction=ownHeight/max(total,eps);
            r.neighborContinuousHeight=sum(others);r.maximumNeighborContinuousHeight=max([0;others]);
            for k=1:numel(model.featureNames),features(j,k)=r.(model.featureNames{k});end
        end
    end
    score=scorePrecisionModel(features,model);selected=unique(owners(score>=model.threshold));
    diagnostic=struct('owners',owners,'hypotheses',ids,'features',features,'score',score,'geometry',geometry);
end

function total=countSupport(z)
    if numel(z)<3,total=0;return;end
    [p,~,g]=unique([z-.25;z+.25]);n=numel(z);count=cumsum(accumarray(g,[ones(n,1);-ones(n,1)]));
    total=sum(diff(p).*(count(1:end-1)>=3));
end
