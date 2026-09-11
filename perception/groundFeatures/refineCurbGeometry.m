function [accepted, detail] = refineCurbGeometry(xyz, pointIndices, groundMask, cfg, referenceNormal, traceOnly)
% refineCurbGeometry: Locate narrow curb boundaries using unorganized XYZ.
% Metric XY neighborhoods test height relief and departure from a local plane.
% Mid-height returns seed spatial boundary consensus; metric arc-length
% sampling retains original returns without using ring, row, or scan order.
    if nargin<5,referenceNormal=[];end
    if nargin<6,traceOnly=false;end
    accepted=false(numel(pointIndices),1);
    detail=struct('seedPointIndices',zeros(0,1),'boundaryPointIndices',{{}},'status',"noSupportedBoundary");
    if isempty(pointIndices),return;end
    % Canonical coordinate order makes neighborhood ties independent of input order.
    groundIds=find(groundMask & all(isfinite(xyz),2));
    [~,order]=sortrows(xyz(groundIds,:),[1 2 3]);groundIds=groundIds(order);
    xyCloud=pointCloud([xyz(groundIds,1:2),zeros(numel(groundIds),1)]);
    [~,order]=sortrows(xyz(pointIndices,:),[1 2 3]);indices=pointIndices(order);
    score=zeros(numel(indices),1);gradients=zeros(numel(indices),2);seed=false(size(indices));
    rawGradients=zeros(numel(indices),2);
    planeResidual=zeros(size(indices));
    for k=1:numel(indices)
        origin=xyz(indices(k),:);
        if isempty(referenceNormal)
            rows=findNeighborsInRadius(xyCloud,[origin(1:2),0],cfg.curbNeighborhoodRadiusMeters);
        else
            % Gather sparse returns along an established curb without widening
            % the neighborhood across the face or changing geometric gates.
            alongRadius=cfg.curbContinuationAlongRadiusMeters;
            acrossRadius=cfg.curbNeighborhoodRadiusMeters;
            rows=findNeighborsInRadius(xyCloud,[origin(1:2),0],hypot(alongRadius,acrossRadius));
            delta=xyz(groundIds(rows),1:2)-origin(1:2);
            along=delta*[-referenceNormal(2);referenceNormal(1)];
            across=delta*referenceNormal.';
            rows=rows((along/alongRadius).^2+(across/acrossRadius).^2<=1);
        end
        if numel(rows)<cfg.curbMinimumNeighbors,continue;end
        points=xyz(groundIds(rows),:)-origin;
        heights=quantile(points(:,3),[0.1 0.9]);relief=diff(heights);
        % Raw height variation includes terrain grade; cap only detrended relief.
        if relief<cfg.curbMinimumReliefMeters,continue;end
        design=[ones(size(points,1),1),points(:,1:2)];moments=design.'*design;
        if rcond(moments)<1e-10,continue;end
        coefficients=moments\(design.'*points(:,3));
        residual=points(:,3)-design*coefficients;
        spread=sqrt(mean(residual.^2));
        if spread<cfg.curbMinimumPlaneResidualMeters,continue;end
        planeResidual(k)=spread;
        gradient=coefficients(2:3).';rawGradients(k,:)=gradient;
        if norm(gradient)<cfg.curbMinimumSlope,continue;end
        normal=gradient/norm(gradient);
        if ~isempty(referenceNormal),normal=referenceNormal;end
        strip=abs(points(:,1:2)*normal.')<=cfg.curbLocalStripHalfWidthMeters;
        if nnz(strip)<cfg.curbMinimumStripNeighbors,continue;end
        % Estimate terrain grade on a supported side surface, away from the face.
        slope=[0;0];bestSurfaceResidual=cfg.curbMaximumSurfaceResidualMeters;
        for angle=[0 pi/4 pi/2 3*pi/4]
            rotation=[cos(angle),-sin(angle);sin(angle),cos(angle)];
            across=points(:,1:2)*rotation*normal.';
            for side=[-1 1]
                surface=points(side*across>cfg.curbLocalStripHalfWidthMeters,:);
                if size(surface,1)<cfg.curbMinimumSurfaceNeighbors,continue;end
                surfaceDesign=[ones(size(surface,1),1),surface(:,1:2)];
                if rcond(surfaceDesign.'*surfaceDesign)<1e-10,continue;end
                fit=surfaceDesign\surface(:,3);
                surfaceResidual=sqrt(mean((surface(:,3)-surfaceDesign*fit).^2));
                if surfaceResidual<bestSurfaceResidual && norm(fit(2:3))<=cfg.curbMaximumSurfaceSlope
                    bestSurfaceResidual=surfaceResidual;slope=fit(2:3);
                end
            end
        end
        terrainCorrected=points(:,3)-points(:,1:2)*slope;
        neighborhoodRelief=diff(quantile(terrainCorrected,[0.1 0.9]));
        gradient=gradient-slope.';
        if norm(gradient)<cfg.curbMinimumSlope,continue;end
        normal=gradient/norm(gradient);
        if ~isempty(referenceNormal),normal=referenceNormal;end
        strip=abs(points(:,1:2)*normal.')<=cfg.curbLocalStripHalfWidthMeters;
        if nnz(strip)<cfg.curbMinimumStripNeighbors,continue;end
        local=points(strip,:);
        detrended=local(:,3)-local(:,1:2)*slope;
        heights=quantile(detrended,[0.1 0.9]);relief=diff(heights);
        if relief<cfg.curbMinimumReliefMeters || relief>cfg.curbMaximumReliefMeters,continue;end
        midpoint=mean(heights);
        if abs(midpoint)>cfg.curbMidHeightBandMeters,continue;end
        score(k)=min(std(detrended,1)/0.04,1)*exp(-0.5*(midpoint/cfg.curbMidHeightBandMeters)^2);
        % Anchored continuation may follow a supported local strip beside
        % taller terrain; its strip relief still passed the same height cap.
        seed(k)=abs(midpoint)<=cfg.curbSeedMidHeightBandMeters && ...
            (traceOnly || neighborhoodRelief<=cfg.curbMaximumReliefMeters);
        gradients(k,:)=gradient;
    end
    valid=score>=cfg.curbMinimumSeedScore;
    indices=indices(valid);score=score(valid);gradients=gradients(valid,:);seed=seed(valid);
    planeResidual=planeResidual(valid);
    % Terrain correction must not manufacture a reversed local transition.
    directionConsistent=sum(rawGradients(valid,:).*gradients,2)>0;
    detail.seedPointIndices=indices(seed);
    if ~any(seed),return;end
    points=xyz(indices,1:2);
    if traceOnly
        [selected,boundaries]=traceCurbRidges(xyz,indices,score,gradients,seed,planeResidual,cfg);
    else
        [selected,boundaries]=selectBoundaries(points,indices,score,gradients,seed,planeResidual,cfg);
    end
    dominated=rejectWeakerRaisedCurbEdges(xyz,indices,score,gradients,cfg);
    conflict=selected & dominated;
    if hasSupportedCurbConflict(points,conflict,cfg)
        % Strong evidence can reconstruct an initially ambiguous set of models.
        keep=~dominated;
        [chosen,boundaries]=traceCurbRidges(xyz,indices(keep),score(keep), ...
            gradients(keep,:),seed(keep),planeResidual(keep),cfg);
        selected=false(size(indices));selected(keep)=chosen;
    else
        % A modest strength advantage is useful only beside an already valid
        % boundary. Restrict this correction to the competing local models.
        [trusted,~]=retainSupportedCurbBoundaries(points,indices,gradients, ...
            directionConsistent,selected,boundaries,cfg);
        alternativeCfg=cfg;alternativeCfg.curbCompetingEdgeGradientRatio=cfg.curbAlternativeEdgeGradientRatio;
        dominated=rejectWeakerRaisedCurbEdges(xyz,indices,score,gradients,alternativeCfg);
        conflict=trusted & dominated;
        if hasSupportedCurbConflict(points,conflict,cfg)
            affected=cellfun(@(ids) any(ismember(ids,indices(conflict))),boundaries);
            affectedIds=vertcat(boundaries{affected});targets=xyz(affectedIds,1:2);
            nearby=min(hypot(points(:,1)-targets(:,1).',points(:,2)-targets(:,2).'),[],2) ...
                <=cfg.curbDuplicateBandMeters;
            keep=nearby & ~dominated;
            [chosen,replacements]=traceCurbRidges(xyz,indices(keep),score(keep), ...
                gradients(keep,:),seed(keep),planeResidual(keep),cfg);
            selected(ismember(indices,affectedIds))=false;
            selected(keep)=selected(keep) | chosen;
            boundaries=[boundaries(~affected),replacements];
        end
    end
    [selected,boundaries]=retainSupportedCurbBoundaries(points,indices,gradients, ...
        directionConsistent,selected,boundaries,cfg);
    accepted=ismember(pointIndices,indices(selected));detail.boundaryPointIndices=boundaries;
    if any(accepted),detail.status="supportedMetricBoundary";end
end

function [selected,boundaries]=retainSupportedCurbBoundaries(points,indices,gradients,directionConsistent,selected,boundaries,cfg)
% Validate models before acceptance or locally anchored competitor tracing.
    % A supported boundary can tolerate individual gradient ambiguity on a
    % sloping road; reject only a spatially supported reversal consensus.
    rejected=false(size(boundaries));
    for j=1:numel(boundaries)
        member=ismember(indices,boundaries{j});
        if ~hasConsistentCurbNormals(points(member,:),gradients(member,:),cfg)
            selected(member)=false;rejected(j)=true;
            continue;
        end
        reversed=member & ~directionConsistent;
        cells=unique(floor(points(reversed,:)/cfg.curbProposalCellSizeMeters),'rows');
        if nnz(reversed)>=cfg.curbMinimumSupportCells && ...
                size(cells,1)>=cfg.curbMinimumOutputSupportCells && ...
                nnz(reversed)/nnz(member)>=cfg.curbGradientReversalFraction
            selected(member)=false;rejected(j)=true;
        end
    end
    boundaries(rejected)=[];
end

function supported=hasSupportedCurbConflict(points,conflict,cfg)
% A competitor needs spatial extent and independently occupied XY cells.
    cells=unique(floor((points(conflict,:)-min(points,[],1))/cfg.curbProposalCellSizeMeters),'rows');
    supported=any(conflict) && size(cells,1)>=cfg.curbMinimumOutputSupportCells && ...
        norm(max(points(conflict,:),[],1)-min(points(conflict,:),[],1))>=cfg.curbMinimumBoundaryLengthMeters;
end

function [selected,boundaries]=selectBoundaries(points,indices,score,gradients,seed,planeResidual,cfg)
% Fit and sample the initial supported boundary proposals.
    bins=floor((points-min(points,[],1))/cfg.curbProposalCellSizeMeters);
    [~,~,groups]=unique(bins,'rows');
    proposal=false(size(indices));
    for group=unique(groups(seed)).'
        members=find(groups==group & seed);[~,best]=max(score(members));proposal(members(best))=true;
    end
    active=seed;available=true(size(seed));selected=false(size(active));boundaries={};
    while numel(unique(groups(active)))>=cfg.curbMinimumSupportCells
        previousActiveCount=nnz(active);
        rows=find(active);anchors=find(active & proposal);
        if numel(anchors)>cfg.curbMaximumProposalAnchors
            anchors=anchors(unique(round(linspace(1,numel(anchors),cfg.curbMaximumProposalAnchors))));
        end
        [normal,offset,extent]=bestBoundaryProposal(points,groups,score,rows,anchors,cfg);
        if isempty(normal),break;end
        [distance,along,extent]=refineBoundaryCurve(points,groups,score,active,normal,offset,extent,cfg);
        members=find(available & distance<=cfg.curbOutputBandMeters & along>=extent(1) & along<=extent(2));
        if isempty(members)
            % Always retire the original proposal's support, even if its curve moved.
            active(abs(points*normal.'-offset)<=cfg.curbConsensusBandMeters ...
                & along>=extent(1) & along<=extent(2))=false;
            continue;
        end
        arcBin=floor((along(members)-min(along(members)))/cfg.curbOutputSpacingMeters);
        chosen=zeros(0,1);
        for bin=unique(arcBin).'
            rows=members(arcBin==bin);[~,best]=max(score(rows)-distance(rows));chosen(end+1,1)=rows(best); %#ok<AGROW>
        end
        % Weak geometric support may stabilize a boundary without becoming an
        % output feature. Gate after sampling to avoid moving/replacing points.
        outputChosen=chosen(planeResidual(chosen)>=cfg.curbMinimumOutputPlaneResidualMeters);
        % A boundary must retain spatial support after point confidence gates;
        % sparse survivors of a weak hypothesis do not establish a curb.
        if numel(unique(groups(outputChosen)))>=cfg.curbMinimumOutputSupportCells && ~isempty(outputChosen)
            selected(outputChosen)=true;boundaries{end+1}=indices(outputChosen); %#ok<AGROW>
        end
        sense=sign(gradients*normal.');boundarySense=sign(sum(sense(chosen)));
        sameFacing=sense==0 | sense==boundarySense | boundarySense==0;
        duplicate=distance<=cfg.curbDuplicateBandMeters & along>=min(along(chosen))-cfg.curbDuplicateEndpointMarginMeters ...
            & along<=max(along(chosen))+cfg.curbDuplicateEndpointMarginMeters & sameFacing;
        duplicate(chosen)=true;active(duplicate)=false;available(duplicate)=false;
        if nnz(active)==previousActiveCount
            % Weak members can face away from the seeds; still guarantee progress.
            active(abs(points*normal.'-offset)<=cfg.curbConsensusBandMeters ...
                & along>=extent(1) & along<=extent(2))=false;
        end
    end
end

function [distance,along,extent]=refineBoundaryCurve(points,groups,score,active,normal,offset,extent,cfg)
% Fit a local curve only where a narrow spatial consensus already exists.
    along=points*[normal(2);-normal(1)];across=points*normal.';
    inlier=active & abs(across-offset)<=cfg.curbConsensusBandMeters & along>=extent(1) & along<=extent(2);
    rows=find(inlier);center=mean(extent);scale=max(diff(extent)/2,eps);
    u=(along-center)/scale;design=[ones(size(u)),u,u.^2];
    counts=accumarray(groups(rows),1,[max(groups),1]);weight=score(rows)./counts(groups(rows));
    local=design(rows,:);weighted=local.*sqrt(weight);
    if rcond(weighted.'*weighted)<1e-10
        distance=abs(across-offset);return;
    end
    quadratic=weighted\(across(rows).*sqrt(weight));
    linear=weighted(:,1:2)\(across(rows).*sqrt(weight));
    curveError=sum(weight.*(across(rows)-local*quadratic).^2);
    lineError=sum(weight.*(across(rows)-local(:,1:2)*linear).^2);
    if abs(quadratic(3)/scale^2)<=cfg.curbMaximumCurvature && ...
            curveError<cfg.curbCurveResidualRatio*lineError
        target=design*quadratic;
        extent=extent+[-1 1]*cfg.curbCurveExtensionMeters;
    else
        target=design(:,1:2)*linear;
    end
    distance=abs(across-target);
end

function [normal,offset,extent] = bestBoundaryProposal(points,groups,score,rows,anchors,cfg)
% Exhaustive bounded proposals, scored once per supporting group, avoid RNG state.
    normal = []; offset = []; extent = [];
    [first,last] = find(triu(true(numel(anchors),numel(anchors)),1));
    delta = points(anchors(last),:)-points(anchors(first),:);
    separationMeters = vecnorm(delta,2,2);
    valid = separationMeters>=cfg.curbMinimumBoundaryLengthMeters & separationMeters<=cfg.curbMaximumProposalLengthMeters;
    if ~any(valid), return; end
    delta = delta(valid,:); separationMeters = separationMeters(valid); first = first(valid);
    normals = [-delta(:,2),delta(:,1)]./separationMeters;
    offsets = sum(points(anchors(first),:).*normals,2);
    bestScore = 0;
    for start = 1:cfg.curbProposalBatchSize:size(normals,1)
        batch = start:min(start+cfg.curbProposalBatchSize-1,size(normals,1));
        distance = abs(points(rows,:)*normals(batch,:).'-offsets(batch).');
        supported = distance<=cfg.curbConsensusBandMeters;
        objective = zeros(1,numel(batch)); count = zeros(size(objective));
        for group = unique(groups(rows)).'
            member = groups(rows)==group;
            contribution = max((score(rows(member))+cfg.curbSupportWeight).*supported(member,:),[],1);
            objective = objective+contribution;
            count = count+(contribution>0);
        end
        objective(count<cfg.curbMinimumSupportCells) = 0;
        [values,order] = sort(objective,'descend');
        for k = 1:numel(order)
            if values(k)<=bestScore, break; end
            candidate = order(k);
            n = normals(batch(candidate),:);
            members = rows(supported(:,candidate));
            along = points(members,:)*[n(2);-n(1)];
            [along,sequence] = sort(along); members = members(sequence);
            starts = [1;find(diff(along)>cfg.curbMaximumSupportGapMeters)+1];
            ends = [starts(2:end)-1;numel(members)];
            for run = 1:numel(starts)
                component = members(starts(run):ends(run));
                groupIds = unique(groups(component));
                if numel(groupIds)<cfg.curbMinimumSupportCells || ...
                        along(ends(run))-along(starts(run))<cfg.curbMinimumBoundaryLengthMeters
                    continue;
                end
                value = 0;
                for group = groupIds.'
                    value = value+max(score(component(groups(component)==group)))+cfg.curbSupportWeight;
                end
                if value>bestScore
                    bestScore = value;
                    normal = n; offset = offsets(batch(candidate));
                    extent = [along(starts(run))-eps(max(1,abs(along(starts(run))))), ...
                        along(ends(run))+eps(max(1,abs(along(ends(run)))))];
                end
            end
        end
    end
end
