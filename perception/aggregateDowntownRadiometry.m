function result=aggregateDowntownRadiometry(pillars,threshold,groundContext)
% aggregateDowntownRadiometry: Joint radiometric and geometric pillar moments.
% Conditional moments describe the intensity distribution within each input
% XY pillar. They are descriptors, not semantic point decisions. No point IDs,
% spatial subdivisions, fitted geometry or point-neighborhood queries leave
% this aggregation. Ground context merges statistics on the same XY lattice.
    if nargin<3,groundContext=struct();end
    xyz=double(pillars.points);[ids,~,group]=unique(double(pillars.pointPillarLinIdx(:)));
    count=accumarray(group,1,[numel(ids),1]);
    intensity=nan(size(group));
    if isfield(pillars.pointAttributes,'intensity')
        intensity=double(pillars.pointAttributes.intensity(:));
    end
    bright=isfinite(intensity) & intensity>threshold;
    result=struct('pillarIndices',int32(ids),'intensityThreshold',double(threshold), ...
        'finiteIntensityCount',accumarray(group(isfinite(intensity)),1,[numel(ids),1]));
    result.reflective=conditionalMoments(xyz,group,bright,count);
    result.nonreflective=conditionalMoments(xyz,group,~bright,count);
    result.ground=groundStatistics(pillars.pillarGeometry,ids,groundContext);
end

function value=conditionalMoments(xyz,group,selected,totalCount)
    number=numel(totalCount);count=accumarray(group(selected),1,[number,1]);
    value=struct('count',count,'fraction',count./max(totalCount,1), ...
        'meanXYZ',zeros(number,3),'covarianceXYZ',zeros(number,6), ...
        'minimumXYZ',nan(number,3),'maximumXYZ',nan(number,3));
    if ~any(selected),return;end
    points=xyz(selected,:);groups=group(selected);present=count>0;
    for axis=1:3
        value.meanXYZ(:,axis)=accumarray(groups,points(:,axis),[number,1])./max(count,1);
        low=accumarray(groups,points(:,axis),[number,1],@min,NaN);
        high=accumarray(groups,points(:,axis),[number,1],@max,NaN);
        value.minimumXYZ(present,axis)=low(present);value.maximumXYZ(present,axis)=high(present);
    end
    residual=points-value.meanXYZ(groups,:);pairs=[1 1;1 2;2 2;1 3;2 3;3 3];
    for k=1:6
        value.covarianceXYZ(:,k)=accumarray(groups,residual(:,pairs(k,1)).* ...
            residual(:,pairs(k,2)),[number,1])./max(count,1);
    end
end

function output=groundStatistics(geometry,ids,context)
% Map cell moments rather than re-querying original ground neighborhoods.
    radii=[0 1 2 4];columns=["count","cells","meanZ","minimumZ","maximumZ", ...
        "varianceZ","nearestDistance","nearestCount","nearestMeanZ","nearestMinimumZ","nearestVarianceZ"];
    output=struct();
    for radius=radii
        for name=columns,output.("r"+radius+"_"+name)=zeros(numel(ids),1);end
    end
    if ~isfield(context,'groundXYView') || isempty(context.groundPoints),return;end
    view=context.groundXYView;spacing=double(view.cellSize(:).');
    assert(all(abs(spacing-double(geometry.cellSize))<1e-10), ...
        'perception:ContextLatticeMismatch','Ground and structural cell spacing must agree.');
    offset=(double(geometry.origin)-double(view.origin))./spacing;
    assert(all(abs(offset-round(offset))<1e-8), ...
        'perception:ContextLatticeMismatch','Ground and structural lattice phases must agree.');
    dims=double(view.gridSize);number=prod(dims);cellIds=double(context.groundCellLinIdx(:));
    z=double(context.groundPoints(:,3));
    count=reshape(accumarray(cellIds,1,[number,1]),dims).';
    sumZ=reshape(accumarray(cellIds,z,[number,1]),dims).';
    sumZZ=reshape(accumarray(cellIds,z.^2,[number,1]),dims).';
    meanZ=sumZ./max(count,1);variance=max(0,sumZZ./max(count,1)-meanZ.^2);
    low=reshape(accumarray(cellIds,z,[number,1],@min,inf),dims).';
    high=reshape(accumarray(cellIds,z,[number,1],@max,-inf),dims).';
    low(count==0)=inf;high(count==0)=-inf;
    % Padding retains neighbors when the off-ground raster extends beyond
    % the compact ground raster. All support remains within four cells.
    padding=max(radii);padded=size(count)+2*padding;
    row=padding+(1:size(count,1));col=padding+(1:size(count,2));
    fields={count,sumZ,sumZZ,meanZ,variance,low,high};
    for k=1:numel(fields)
        fill=0;if k==6,fill=inf;elseif k==7,fill=-inf;end
        expanded=repmat(fill,padded);expanded(row,col)=fields{k};fields{k}=expanded;
    end
    count=fields{1};sumZ=fields{2};sumZZ=fields{3};meanZ=fields{4};
    variance=fields{5};low=fields{6};high=fields{7};occupied=count>0;
    [distance,nearest]=bwdist(occupied);
    [queryRow,queryCol]=ind2sub(double(geometry.mapSize),ids);
    queryRow=queryRow+round(offset(2))+padding;queryCol=queryCol+round(offset(1))+padding;
    valid=queryRow>=1 & queryRow<=padded(1) & queryCol>=1 & queryCol<=padded(2);
    query=sub2ind(padded,queryRow(valid),queryCol(valid));
    for radius=radii
        kernel=ones(2*radius+1);n=conv2(count,kernel,'same');
        cells=conv2(double(occupied),kernel,'same');mu=conv2(sumZ,kernel,'same')./max(n,1);
        varZ=max(0,conv2(sumZZ,kernel,'same')./max(n,1)-mu.^2);
        minimum=imerode(low,kernel);maximum=imdilate(high,kernel);
        values=zeros(numel(ids),numel(columns));
        values(valid,1:6)=[n(query),cells(query),mu(query),minimum(query),maximum(query),varZ(query)];
        closest=nearest(query);hasNearest=isfinite(distance(query)) & distance(query)<=radius;
        local=zeros(numel(query),5);use=closest(hasNearest);
        local(hasNearest,:)=[distance(query(hasNearest))*mean(spacing), ...
            count(use),meanZ(use),low(use),variance(use)];
        values(valid,7:11)=local;
        values(~isfinite(values))=0;
        for k=1:numel(columns),output.("r"+radius+"_"+columns(k))=values(:,k);end
    end
end
