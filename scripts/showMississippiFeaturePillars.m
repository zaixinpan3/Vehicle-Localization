function result=showMississippiFeaturePillars(frameIndex,exportFolder)
% showMississippiFeaturePillars: Frozen fine points above current coarse cells.
% Colors: orange pole, magenta traffic sign, cyan curb. Original source points
% are colored once at uniform size. White marks overlapping reference labels.
% A shared coarse cell uses color strips solely to show multiple memberships;
% these strips are not subcells and do not change the 0.6 m detector lattice.
    arguments
        frameIndex (1,1) double {mustBeInteger,mustBePositive}=500
        exportFolder (1,1) string=""
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    names=["pole","trafficSign","curb"];
    palette=[1 .43 .10;1 .20 .85;.05 .90 1];
    [references,referenceSource]=loadReference(root,frameIndex,names);
    result=showMississippiPolePillars(frameIndex);
    fig=result.figure;ax=result.axes;cloud=result.sourceScatter;
    xyz=double([result.frame.x(:),result.frame.y(:),result.frame.z(:)]);
    finite=all(isfinite(xyz),2);sourceIds=find(finite);masks=false(size(xyz,1),numel(names));
    assert(isequal(references{1},double(result.reference.pointIndices(:))), ...
        'Original partition pole indices differ from the previously displayed reference.');
    geometry=result.perception.candidates.geometry;
    dims=double(geometry.mapSize);origin=double(geometry.origin);spacing=double(geometry.cellSize);
    membership=false(prod(dims),numel(names));features=struct();
    colors=repmat([.48 .53 .59],nnz(finite),1);
    for k=1:numel(names)
        ids=references{k};assert(all(finite(ids)),'Reference points must be finite.');
        masks(ids,k)=true;
        colors(masks(sourceIds,k),:)=repmat(palette(k,:),numel(ids),1);
        channel=find(result.perception.candidates.semanticNames==names(k));
        assert(isscalar(channel),'The current coarse invocation must include every requested class.');
        selected=double(result.perception.candidates.pillarIndices{channel}(:));
        membership(selected,k)=true;
        [alignment,detail]=measureFinePoleAlignment(result.frame,ids,selected,geometry);
        features.(names(k))=struct('referencePointIndices',ids,'referencePointCount',numel(ids), ...
            'selectedPillarIds',selected,'selectedPillarCount',numel(selected), ...
            'referencePillarIds',detail.finePillarIds,'alignment',alignment,'colorRGB',palette(k,:));
    end
    overlap=sum(masks,2)>1;colors(overlap(sourceIds),:)=1;
    % Preserve the native full-cloud RGB cache used by rotate and zoom.
    cloud.CData=colors;cloud.ColorData=colors;
    legend(ax,'off');delete(result.referenceLegend);delete(result.pillarPatches);
    delete(findobj(ax,'Type','patch','DisplayName','Detected coarse pole pillars'));
    hold(ax,'on');selected=find(any(membership,2));patches=gobjects(0);patchClasses=zeros(0,1);patchIds=zeros(0,1);
    floorZ=result.metrics.displayGridZ;
    for i=1:numel(selected)
        id=selected(i);[row,col]=ind2sub(dims,id);lower=origin+([col row]-1).*spacing;
        classes=find(membership(id,:));n=numel(classes);
        for j=1:n
            k=classes(j);left=lower(1)+(j-1)*spacing(1)/n;right=lower(1)+j*spacing(1)/n;
            patches(end+1,1)=patch(ax,[left right right left], ...
                lower(2)+[0 0 spacing(2) spacing(2)],(floorZ+.015)*ones(1,4),palette(k,:), ...
                'EdgeColor',palette(k,:),'LineWidth',1,'HandleVisibility','off'); %#ok<AGROW>
            patchClasses(end+1,1)=k;patchIds(end+1,1)=id; %#ok<AGROW>
        end
    end
    proxies=gobjects(numel(names),1);
    for k=1:numel(names)
        f=features.(names(k));
        proxies(k)=plot3(ax,NaN,NaN,NaN,'.','Color',palette(k,:), ...
            'DisplayName',sprintf('%s: %d fine points | %d coarse cells',names(k),f.referencePointCount,f.selectedPillarCount));
        assert(isequal(sort(patchIds(patchClasses==k)),sort(f.selectedPillarIds)));
    end
    if any(overlap)
        proxies(end+1)=plot3(ax,NaN,NaN,NaN,'.','Color','white','DisplayName','Overlapping fine labels');
    end
    legend(ax,[cloud;proxies],'Location','northeast','TextColor',[.87 .90 .94], ...
        'Color',[.055 .065 .085],'AutoUpdate','off');
    fig.Name=sprintf('Mississippi %d | fine reference and coarse feature pillars',frameIndex);
    title(ax,{sprintf('Mississippi | frame %d | full cloud: %d points',frameIndex,nnz(finite)), ...
        'Points: original 0.3 m fine reference | Grid: current 0.6 m coarse selections'}, ...
        'Color',[.87 .90 .94],'FontSize',14);
    notes=findall(fig,'Type','textboxshape');
    for k=1:numel(notes),delete(notes(k));end
    annotation(fig,'textbox',[.04 .055 .92 .045],'String', ...
        sprintf('Orange: pole | Magenta: traffic sign | Cyan: curb. Shared cells use color strips.\nGrid at Z = %.2f m for display only. Fine reference = stored detector output.',floorZ), ...
        'Color',[.87 .90 .94],'EdgeColor','none','FontSize',11);
    state=getappdata(fig,'PolePillarViewControls');state.focusPoints=xyz(any(masks,2),:);
    [row,col]=ind2sub(dims,selected);
    state.focusPoints=[state.focusPoints;origin+([col row]-.5).*spacing,repmat(floorZ,numel(selected),1)];
    if isempty(state.focusPoints),state.focusPoints=xyz(finite,:);end
    setappdata(fig,'PolePillarViewControls',state);
    button=findall(fig,'Tag','PoleRegion');button.String='Feature region';
    assert(isscalar(findobj(ax,'Type','scatter')) && cloud.SizeData==4);
    assert(isequal([cloud.XData(:),cloud.YData(:),cloud.ZData(:)],xyz(finite,:)));
    assert(isequal(cloud.CData,colors) && isequal(cloud.ColorData,colors));
    for k=1:numel(names)
        assert(nnz(all(cloud.CData==palette(k,:),2))==nnz(masks(:,k) & ~overlap));
    end
    metrics=struct('frame',frameIndex,'sourcePoints',size(xyz,1),'displayedPoints',nnz(finite), ...
        'features',features,'referenceSource',referenceSource,'referenceInferenceRerun',false, ...
        'overlappingReferencePoints',nnz(overlap),'sharedCoarseCells',nnz(sum(membership,2)>1), ...
        'gridGeometry',geometry,'displayGridZ',floorZ,'markerSize',4,'featureOverlay',false);
    result.metrics=metrics;result.referenceMasks=masks;result.featureNames=names;
    result.referenceLegend=proxies;result.pillarPatches=patches;result.patchClasses=patchClasses;result.patchPillarIds=patchIds;
    setappdata(fig,'PolePillarComparison',metrics);drawnow;
    if strlength(exportFolder)>0
        if ~isfolder(exportFolder),mkdir(exportFolder);end
        stem=sprintf('features_%04d',frameIndex);
        exportgraphics(fig,fullfile(exportFolder,[stem '.png']),'Resolution',120);
        file=fopen(fullfile(exportFolder,[stem '.json']),'w');cleanup=onCleanup(@()fclose(file));
        fprintf(file,'%s\n',jsonencode(metrics,PrettyPrint=true));
    end
    for k=1:numel(names)
        f=features.(names(k));fprintf('%s: %d fine points, %d coarse cells, %d/%d reference points covered, %d empty-reference cells.\n', ...
            names(k),f.referencePointCount,f.selectedPillarCount,f.alignment.coveredFinePointCount, ...
            f.alignment.finePointCountInRoi,f.alignment.extraPillarCount);
    end
    fprintf('Feature viewer ready: frame %d.\n',frameIndex);
end

function [references,source]=loadReference(root,frameIndex,names)
    references=cell(1,numel(names));source="";
    for worker=1:4
        relative=fullfile('output','fine_matching_20260919',sprintf('inputs_%d.mat',worker));
        header=load(fullfile(root,relative),'frames');at=find(header.frames==frameIndex);
        if isempty(at),continue;end
        assert(isscalar(at),'Reference frame must occur once per partition.');
        part=load(fullfile(root,relative),'selectedIndices','counts','cfg','processed');
        assert(part.processed==numel(header.frames),'Reference partition is incomplete.');
        assert(all(abs(part.cfg.voxel.voxelSize(1:2)-.3)<1e-12),'Expected original 0.3 m references.');
        for k=1:numel(names)
            channel=find(string(part.cfg.featureNames)==names(k));assert(isscalar(channel));
            ids=double(part.selectedIndices{at,channel}(:));
            assert(numel(ids)==part.counts(at,channel) && numel(unique(ids))==numel(ids));
            assert(all(isfinite(ids) & ids>=1 & ids==floor(ids)));
            references{k}=ids;
        end
        source=string(relative);break;
    end
    assert(strlength(source)>0,'No frozen original fine reference for this frame.');
end
