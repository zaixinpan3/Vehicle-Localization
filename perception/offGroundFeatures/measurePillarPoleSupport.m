function hypotheses=measurePillarPoleSupport(points,pillarIds,geometry,modes,settings,structuralMask)
% measurePillarPoleSupport: Measure complete continuous-height shaft support.
% Existing shaft axes are proposals. Moving counts define supported heights
% without an XY subgrid or fixed Z phase; all supported transverse returns
% contribute before trimming. No reference label enters this calculation.
    if nargin<5,settings=pillarPoleValidationConfig();end
    if nargin<6,structuralMask=true(size(points,1),1);end
    ids=double(modes.pillarIndices(:));p=double(points);
    [occupied,~,group]=unique(double(pillarIds(:)));assert(isequal(ids,occupied));
    counts=accumarray(group,1);[~,order]=sort(group);first=cumsum([1;counts(1:end-1)]);
    lookup=zeros(geometry.mapSize);lookup(ids)=1:numel(ids);
    selected=find(modes.found);hypotheses=struct([]);
    % Identical assigned axes are measured once; point ownership is recomputed.
    keys=[modes.axisXY(selected,:),modes.axisZ(selected),modes.slopeXY(selected,:)];
    [~,keep]=unique(keys,'rows','stable');selected=selected(keep);
    records=cell(numel(selected)*numel(settings.radii),1);recordCount=0;
    for j=selected.'
        center=modes.axisXY(j,:);slope=modes.slopeXY(j,:);z0=modes.axisZ(j);
        lower=floor((center-settings.searchHalfWidth-geometry.origin)./geometry.cellSize)+1;
        upper=floor((center+settings.searchHalfWidth-geometry.origin)./geometry.cellSize)+1;
        rr=max(1,lower(2)):min(geometry.mapSize(1),upper(2));cc=max(1,lower(1)):min(geometry.mapSize(2),upper(1));
        neighbors=lookup(rr,cc);neighbors=neighbors(neighbors>0);
        pieces=arrayfun(@(b)order(first(b):first(b)+counts(b)-1),neighbors(:),'UniformOutput',false);
        index=vertcat(pieces{:});q=p(index,:);owners=group(index);
        distance=vecnorm(q(:,1:2)-center-(q(:,3)-z0).*slope,2,2);
        nearby=distance<=settings.contextRadius;q=q(nearby,:);owners=owners(nearby);distance=distance(nearby);
        structural=structuralMask(index(nearby));
        densityStructural=structural;
        if size(q,1)<6,continue;end
        for radius=settings.radii
            core=distance<=radius;
            if nnz(core)<6,continue;end
            [intervals,meanRatio,massRatio]=qualifiedIntervals(q(:,3),core,settings,densityStructural);
            if isempty(intervals),continue;end
            supportHeight=sum(intervals(:,2)-intervals(:,1));
            if supportHeight<settings.minimumSupportHeight-1e-10 || ...
                    meanRatio<settings.minimumMeanRatio || massRatio<settings.minimumMassRatio,continue;end
            supported=false(size(core));
            for k=1:size(intervals,1)
                supported=supported | q(:,3)>=intervals(k,1) & q(:,3)<=intervals(k,2);
            end
            fit=core & supported;
            longest=max(intervals(:,2)-intervals(:,1));
            % Prefer dense interiors to avoid borrowing clutter at window
            % edges. Sparse paired returns may qualify only BETWEEN samples;
            % when no interior fit exists, use actual qualified-window support.
            if nnz(fit)<6 || max(q(fit,3))-min(q(fit,3))<.5
                supportIntervals=mergeSupportWindows(intervals,settings.halfWindow);
                for k=1:size(supportIntervals,1)
                    supported=supported | q(:,3)>=supportIntervals(k,1) & q(:,3)<=supportIntervals(k,2);
                end
                fit=core & supported;longest=max(supportIntervals(:,2)-supportIntervals(:,1));
            end
            if nnz(fit)<6 || max(q(fit,3))-min(q(fit,3))<.5,continue;end
            axisZ=median(q(fit,3));design=[ones(size(q,1),1),q(:,3)-axisZ];
            beta=design(fit,:)\q(fit,1:2);residual=q(:,1:2)-design*beta;
            radial=vecnorm(residual,2,2);radialRms=sqrt(mean(radial(fit).^2));
            transverse=cov(residual(fit,:));eigenvalues=max(0,sort(eig(transverse)));
            coreCount=nnz(supported & radial<=settings.isolationCoreRadius);neighborhoodCount=nnz(supported & radial<=settings.contextRadius);
            mid=median(radial(fit));sigma=max(1.4826*median(abs(radial(fit)-mid)),settings.minimumRadialScale);
            limit=min(settings.maximumTrimRadius,mid+settings.trimMadMultiplier*sigma);if meanRatio<settings.strongMeanRatio,limit=min(limit,settings.weakTrimRadius);end
            accepted=fit & radial<=limit;
            if any(accepted)
                accepted(accepted)=retainContinuousPoleSupport(q(accepted,3),settings);
            end
            ownerIds=unique(owners(accepted));ownerCount=zeros(size(ownerIds));ownerHeight=ownerCount;
            for k=1:numel(ownerIds)
                own=accepted & owners==ownerIds(k);ownerCount(k)=nnz(own);zz=q(own,3);ownerHeight(k)=max(zz)-min(zz);
            end
            h=struct('proposalPillar',ids(j),'radius',radius,'supportHeight',supportHeight, ...
                'longestSupportedHeight',longest,'meanRatio',meanRatio,'massRatio',massRatio, ...
                'supportedCount',nnz(fit),'tilt',atand(norm(beta(2,:))),'radialRms',radialRms, ...
                'maximumStd',sqrt(eigenvalues(2)),'aspect',sqrt(eigenvalues(2)/max(eigenvalues(1),eps)), ...
                'isolation',coreCount/max(neighborhoodCount,1),'coreCount',coreCount, ...
                'densityContrast',coreCount/max(neighborhoodCount-coreCount,1)*(settings.contextRadius^2/settings.isolationCoreRadius^2-1), ...
                'axisXY',beta(1,:),'axisZ',axisZ,'slopeXY',beta(2,:), ...
                'acceptedCount',nnz(accepted),'ownerIds',ids(ownerIds),'ownerCount',ownerCount,'ownerHeight',ownerHeight);
            recordCount=recordCount+1;records{recordCount}=h;
        end
    end
    if recordCount>0,hypotheses=vertcat(records{1:recordCount});end
end

function intervals=mergeSupportWindows(centers,halfWidth)
    expanded=centers+[-halfWidth halfWidth];intervals=zeros(size(expanded));count=1;
    intervals(1,:)=expanded(1,:);
    for k=2:size(expanded,1)
        if expanded(k,1)<=intervals(count,2)
            intervals(count,2)=max(intervals(count,2),expanded(k,2));
        else
            count=count+1;intervals(count,:)=expanded(k,:);
        end
    end
    intervals=intervals(1:count,:);
end

function [intervals,meanRatio,massRatio]=qualifiedIntervals(z,core,cfg,structural)
    [positions,~,events]=unique([z-cfg.halfWindow;z+cfg.halfWindow]);
    delta=[double(core & structural),double(structural)];delta=[delta;-delta];counts=zeros(numel(positions),2);
    for k=1:2,counts(:,k)=cumsum(accumarray(events,delta(:,k),[numel(positions) 1]));end
    widths=diff(positions);counts=counts(1:end-1,:);ratio=counts(:,1)./max(counts(:,2),1);
    selected=counts(:,1)>=cfg.minimumWindowPoints & ratio>cfg.minimumWindowFraction;
    changes=diff([false;selected;false]);first=find(changes==1);last=find(changes==-1);
    intervals=[positions(first),positions(last)];total=sum(widths(selected));meanRatio=0;massRatio=0;
    if total>0
        meanRatio=sum(widths(selected).*ratio(selected))/total;
        massRatio=sum(widths(selected).*counts(selected,1))/sum(widths(selected).*counts(selected,2));
    end
end
