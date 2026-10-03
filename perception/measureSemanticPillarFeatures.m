function [x,names,ids]=measureSemanticPillarFeatures(ground,offGround,semantic)
% measureSemanticPillarFeatures: Current-frame 0.6 m cell distribution evidence.
% No labels, frame index, absolute XY position or fine lattice are inputs.
% IDs address the branch raster, before publication on the shared lattice.
    semantic=string(semantic);
    if semantic=="curb"
        mask=ground.curbCellMask;ids=find(mask);dims=size(mask);
        fields=struct();groups={ground.stats,ground.energyMaps};prefixes={'ground_','energy_'};
        for g=1:numel(groups)
            source=groups{g};
            for key=sort(fieldnames(source)).'
                value=source.(key{1});
                if (isnumeric(value)||islogical(value)) && isequal(size(value),dims)
                    fields.([prefixes{g} key{1}])=double(value);
                end
            end
        end
        fields.road=double(ground.roadCellMask);
        origin=ground.cellOrigin;spacing=ground.cellSize;
        occupied=ground.stats.countMap>0;
        neighbor={'ground_countMap','ground_roughnessMap','ground_heightMap', ...
            'energy_totalBase','energy_total','energy_heightStepMeters','energy_linearity','road'};
    else
        maps=offGround.columnMaps;mask=offGround.(semantic+"CellMask");ids=find(mask);dims=size(mask);
        stats=maps.statistics;at=double(stats.pillarIndices);cv=stats.covarianceXYZ;
        [rr,cc]=ind2sub(dims,at);origin=maps.origin;spacing=[maps.dx maps.dy];
        lower=origin+([cc(:) rr(:)]-1).*spacing;
        fields=struct();fields.count=zeros(dims);fields.count(at)=stats.count;
        data=[stats.meanXYZ(:,1:2)-lower,stats.minimumXYZ(:,1:2)-lower, ...
            stats.maximumXYZ(:,1:2)-lower,stats.meanXYZ(:,3),stats.minimumXYZ(:,3),stats.maximumXYZ(:,3),cv];
        columns=["localMeanX","localMeanY","localMinX","localMinY","localMaxX","localMaxY", ...
            "meanZ","minZ","maxZ","varXX","covXY","varYY","covXZ","covYZ","varZZ"];
        for k=1:numel(columns),v=zeros(dims);v(at)=data(:,k);fields.(columns(k))=v;end
        fields.height=fields.maxZ-fields.minZ;
        for attribute=["intensity","reflectivity"]
            v=zeros(dims);if isfield(stats,attribute),v(at)=stats.(attribute).maximum;end
            fields.(attribute+"Max")=v;
        end
        fields.xyTrace=fields.varXX+fields.varYY;
        fields.xyAnisotropy=sqrt((fields.varXX-fields.varYY).^2+4*fields.covXY.^2)./max(fields.xyTrace,eps);
        fields.radialVariance=max(0,fields.xyTrace-(fields.covXZ.^2+fields.covYZ.^2)./max(fields.varZZ,eps));
        fields.tilt=hypot(fields.covXZ,fields.covYZ)./max(fields.varZZ,eps);
        occupied=fields.count>0;
        neighbor={'count','height','varZZ','xyTrace','radialVariance','intensityMax'};
        if semantic=="facade"
            fields.lineScore=double(maps.facadeLineScore);
            fields.planeDistance=zeros(dims);fields.normalVariance=zeros(dims);
            fields.lineLength=zeros(dims);fields.lineSupportCount=zeros(dims);fields.lineHeight=zeros(dims);
            for k=1:size(offGround.facade.detectedLines,1)
                line=offGround.facade.detectedLines(k,:);t=line(3:4)-line(1:2);length=norm(t);
                normal=[-t(2),t(1)]/max(length,eps);members=offGround.facade.lineMap(at)==k;
                chosen=at(members);
                fields.planeDistance(chosen)=abs((stats.meanXYZ(members,1:2)-line(1:2))*normal.');
                fields.normalVariance(chosen)=cv(members,1)*normal(1)^2+2*cv(members,2)*prod(normal)+cv(members,3)*normal(2)^2;
                fields.lineLength(chosen)=length;fields.lineSupportCount(chosen)=sum(stats.count(members));
                if any(members),fields.lineHeight(chosen)=max(stats.maximumXYZ(members,3))-min(stats.minimumXYZ(members,3));end
            end
            neighbor=[neighbor,{'lineScore','planeDistance','normalVariance'}];
        end
    end
    ids=ids(:);
    [rows,cols]=ind2sub(dims,ids);center=origin+([cols(:) rows(:)]-.5).*spacing;
    x=hypot(center(:,1),center(:,2));names="range";
    for key=sort(fieldnames(fields)).'
        value=fields.(key{1});x(:,end+1)=value(ids);names(end+1)=string(key{1}); %#ok<AGROW>
    end
    for k=1:numel(neighbor)
        v=fields.(neighbor{k});valid=occupied & isfinite(v);v(~valid)=0;
        for radius=[1 2]
            kernel=ones(2*radius+1);count=conv2(double(valid),kernel,'same');
            mean=conv2(v,kernel,'same')./max(count,1);
            variance=max(0,conv2(v.^2,kernel,'same')./max(count,1)-mean.^2);
            values=reshape([mean(ids),sqrt(variance(ids)),v(ids)-mean(ids),count(ids)],numel(ids),4);
            x=[x,values]; %#ok<AGROW>
            names=[names,string(neighbor{k})+"_r"+radius+["_mean","_std","_contrast","_count"]]; %#ok<AGROW>
        end
    end
    x(~isfinite(x))=0;x=floor(x*1e8+.5)/1e8;
end
