function candidates = buildPerceptionCandidates(pillars, ground, offGround, semanticNames)
% buildPerceptionCandidates: Publish semantic XY pillar IDs and geometry.
% Height bins contribute statistics; only XY pillars receive semantic labels.
    if nargin < 4
        semanticNames = ["curb", "roadMarking", "pole", "facade", "trafficSign"];
    end
    semanticNames = string(semanticNames(:));
    geometry = pillars.pillarGeometry;
    ids = cell(numel(semanticNames),1);
    for k = 1:numel(semanticNames)
        name = semanticNames(k);
        if ismember(name,["curb","roadMarking"])
            offset = [0 0];
            if isfield(ground,"pillarOffset"), offset = ground.pillarOffset; end
            [row,col] = find(ground.(name+"CellMask"));
            bins = [col(:)+offset(1),row(:)+offset(2)];
        else
            [row,col] = find(offGround.(name+"CellMask"));
            maps = offGround.columnMaps;
            centers = maps.origin + ([col(:) row(:)]-0.5).*[maps.dx maps.dy];
            bins = floor((centers-geometry.origin)./geometry.cellSize)+1;
        end
        valid = bins(:,1)>=1 & bins(:,1)<=geometry.mapSize(2) & ...
            bins(:,2)>=1 & bins(:,2)<=geometry.mapSize(1);
        ids{k} = int32(unique(sub2ind(geometry.mapSize,bins(valid,2),bins(valid,1))));
    end
    candidates = struct("productType", "sparseSemanticPillarCandidates", ...
        "geometry", geometry, "semanticNames", semanticNames, ...
        "pillarIndices", {ids}, "framePointCount", pillars.numInputPoints, ...
        "groundReflectivityThreshold", ground.roadMarkingReflectivityThreshold);
end
