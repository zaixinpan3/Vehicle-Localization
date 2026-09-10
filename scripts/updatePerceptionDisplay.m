function updatePerceptionDisplay(fig, frame, masks, featureNames, frameIndex, totalFrames)
% updatePerceptionDisplay: Color each original point once at a uniform size.
% The figure contains one pcshow scatter with its native pcviewer tag. Later
% requested classes supply the display color when masks overlap; the masks
% themselves remain unchanged. Legend proxies contain no finite points.
    cloud=findobj(fig,'Type','scatter','Tag','pcviewer');
    assert(isscalar(cloud),'Expected one tagged pcshow point cloud.');
    ax=ancestor(cloud,'axes');
    properties={'XLim','YLim','ZLim','DataAspectRatio','PlotBoxAspectRatio', ...
        'CameraPosition','CameraTarget','CameraUpVector','CameraViewAngle','Projection'};
    viewState=struct();
    for k=1:numel(properties),viewState.(properties{k})=get(ax,properties{k});end
    set(ax,'XLimMode','manual','YLimMode','manual','ZLimMode','manual', ...
        'DataAspectRatioMode','manual','PlotBoxAspectRatioMode','manual', ...
        'CameraPositionMode','manual','CameraTargetMode','manual', ...
        'CameraUpVectorMode','manual','CameraViewAngleMode','manual');
    names=validatePerceptionFeatureNames(featureNames);
    xyz=double([frame.x(:),frame.y(:),frame.z(:)]);
    indices=find(all(isfinite(xyz),2));
    colors=repmat([.42 .42 .46],numel(indices),1);
    labels=repmat("source",numel(indices),1);
    paletteNames=["curb","pole","facade","trafficSign"];
    palette=[1 .25 .08;0 .85 1;.3 1 .4;1 .2 .9];
    [~,rows]=ismember(names,paletteNames);
    counts=zeros(numel(names),1);
    for k=1:numel(names)
        mask=masks.(names(k));
        assert(islogical(mask) && numel(mask)==size(xyz,1),'Masks must address original points.');
        selected=mask(indices);
        colors(selected,:)=repmat(palette(rows(k),:),nnz(selected),1);
        labels(selected)=names(k);counts(k)=nnz(selected);
    end
    manager=datacursormode(fig);removeAllDataCursors(manager);
    % Native drag/zoom restores the full cloud from these transient caches.
    % Keep them aligned with the latest frame and exact semantic RGB values.
    assert(isprop(cloud,'PointCloud') && isprop(cloud,'ColorData'), ...
        'perception:RestoreViewer','Call restorePerceptionFigure after openfig.');
    cloud.PointCloud=pointCloud(xyz(indices,:));
    cloud.ColorData=colors;
    interaction=ax.PCUserData;
    interaction.colorMapData="userspecified";
    ax.PCUserData=interaction;
    set(cloud,'XData',xyz(indices,1),'YData',xyz(indices,2),'ZData',xyz(indices,3), ...
        'CData',colors,'MarkerEdgeColor','flat','MarkerFaceColor','flat','PickableParts','visible','HitTest','on');
    setappdata(cloud,'OriginalFramePointIndices',indices);
    setappdata(cloud,'PointDisplayLabels',labels);
    dataset=getappdata(fig,'PerceptionDataset');if isempty(dataset),dataset="Mississippi";end
    source=struct('dataset',dataset,'xyz',xyz,'featureMasks',masks,'featureNames',names, ...
        'frameSize',size(frame.x),'frameIndex',frameIndex);
    setappdata(fig,'PointTipSource',source);
    setappdata(fig,'PickedPointIndices',zeros(0,1));
    manager.UpdateFcn=@perceptionPointTip;
    proxies=findobj(ax,'Tag','PerceptionLegendProxy');
    previous=getappdata(fig,'PerceptionLegendNames');
    if ~isequal(previous,names) || numel(proxies)~=numel(names)
        delete(proxies);legend(ax,'off');
        proxies=gobjects(numel(names),1);held=ishold(ax);hold(ax,'on');
        for k=1:numel(names)
            proxies(k)=plot3(ax,NaN,NaN,NaN,'.','Color',palette(rows(k),:), ...
                'MarkerSize',8,'Tag','PerceptionLegendProxy','HitTest','off','PickableParts','none');
        end
        if ~held,hold(ax,'off');end
        if ~isempty(names)
            legend(ax,proxies,compose('%s: %d points',names(:),counts), ...
                'TextColor','white','Color',[.1 .1 .12],'Location','northeast','AutoUpdate','off');
        end
        setappdata(fig,'PerceptionLegendNames',names);
    else
        key=findobj(fig,'Type','legend');
        if ~isempty(key),key.String=compose('%s: %d points',names(:),counts);end
    end
    counter=findall(fig,'Tag','PerceptionFrameCounter');
    if isempty(counter)
        counter=annotation(fig,'textbox',[.02 .94 .38 .045], ...
            'Color','white','BackgroundColor','black','LineStyle','none', ...
            'FontSize',14,'FitBoxToText','on','Interpreter','none','Tag','PerceptionFrameCounter');
    end
    counter.String=sprintf('Frame %d / %d',frameIndex,totalFrames);
    for k=1:numel(properties),set(ax,properties{k},viewState.(properties{k}));end
end
