function result=showMississippiPolePillars(frameIndex,exportFolder)
% showMississippiPolePillars: Full source cloud above a coarse pole grid.
% Orange points are frozen original 0.3 m fine pole references. Orange
% floor cells are current 0.6 m coarse pole selections, independently drawn.
% All finite original XYZ points are displayed without ROI cropping or
% downsampling. The grid is moved below the cloud for display only; its XY
% origin, spacing, extent and selected cells retain their actual geometry.
% Native camera zoom targets source points instead of cropping data limits.
% Drag to rotate and use the scroll wheel to zoom. The optional exportFolder
% receives a PNG and JSON with exact original-point and pillar counts.
    arguments
        frameIndex (1,1) double {mustBeInteger,mustBePositive}=900
        exportFolder (1,1) string=""
    end
    root=fileparts(fileparts(mfilename('fullpath')));addpath(root);setupVehicleLocalization;
    addpath(fullfile(root,'research','pillar_fine_alignment_20260926'));
    source=fullfile(root,'data','raw','MissisipiPointClouds.mat');
    referenceFile=fullfile(root,'output','pole_precision_20260927','mississippi.mat');
    assert(isfile(referenceFile),'The frozen original fine-reference cache is required.');
    [frame,totalFrames]=loadPointCloudFrame(source,frameIndex);
    cache=load(referenceFile,'frames','references');at=find(cache.frames==frameIndex,1);
    assert(~isempty(at),'The selected frame has no frozen fine reference.');
    ref=cache.references{at};cfg=perceptionConfig('Mississippi');p=perceiveFrame(frame,cfg);
    geometry=p.candidates.geometry;
    selected=double(p.candidates.pillarIndices{p.candidates.semanticNames=="pole"});
    xyz=double([frame.x(:),frame.y(:),frame.z(:)]);finite=all(isfinite(xyz),2);
    fine=double(ref.pointIndices(:));assert(all(finite(fine)),'Reference points must be finite.');
    assert(isequal(xyz(fine,:),double(ref.points)),'Frozen reference XYZ no longer matches the raw frame.');
    [alignment,detail]=measureFinePoleAlignment(frame,fine,selected,geometry);
    assert(any(finite),'No finite source points to display.');

    orange=[1 .43 .10];background=[.055 .065 .085];foreground=[.87 .90 .94];
    points=xyz(finite,:);low=min(points,[],1);high=max(points,[],1);
    floorZ=low(3)-5;
    origin=double(geometry.origin);spacing=double(geometry.cellSize);dims=double(geometry.mapSize);
    xEdges=origin(1)+(0:dims(2))*spacing(1);yEdges=origin(2)+(0:dims(1))*spacing(2);
    fig=figure('Name',sprintf('Mississippi %d | fine pole points and coarse pillars',frameIndex), ...
        'NumberTitle','off','Color',background,'Position',[80 80 1500 1000],'WindowStyle','normal');
    ax=axes(fig,'Position',[.06 .10 .90 .79],'Color',background, ...
        'XColor',foreground,'YColor',foreground,'ZColor',foreground,'FontSize',11);
    colors=repmat([.48 .53 .59],nnz(finite),1);
    colors(ismember(find(finite),fine),:)=repmat(orange,numel(fine),1);
    pcshow(points,colors,'Parent',ax,'MarkerSize',4,'Projection','orthographic', ...
        'BackgroundColor',background,'AxesVisibility','on');
    cloud=findobj(ax,'Type','scatter','Tag','pcviewer');
    cloud.DisplayName='Full source cloud';
    hold(ax,'on');
    patch(ax,[xEdges(1) xEdges(end) xEdges(end) xEdges(1)], ...
        [yEdges(1) yEdges(1) yEdges(end) yEdges(end)],floorZ*ones(1,4),[.09 .12 .16], ...
        'EdgeColor',[.5 .58 .67],'LineWidth',1,'HandleVisibility','off');
    xx=[xEdges;xEdges;nan(size(xEdges))];yy=[repmat(yEdges(1),size(xEdges));repmat(yEdges(end),size(xEdges));nan(size(xEdges))];
    line(ax,xx(:),yy(:),floorZ*ones(numel(xx),1),'Color',[.23 .28 .35],'LineWidth',.4,'HandleVisibility','off');
    yy=[yEdges;yEdges;nan(size(yEdges))];xx=[repmat(xEdges(1),size(yEdges));repmat(xEdges(end),size(yEdges));nan(size(yEdges))];
    line(ax,xx(:),yy(:),floorZ*ones(numel(xx),1),'Color',[.23 .28 .35],'LineWidth',.4,'HandleVisibility','off');
    [row,col]=ind2sub(dims,selected);lower=origin+([col row]-1).*spacing;
    cellHandles=gobjects(numel(selected),1);
    for k=1:numel(selected)
        x=lower(k,1)+[0 spacing(1) spacing(1) 0];y=lower(k,2)+[0 0 spacing(2) spacing(2)];
        cellHandles(k)=patch(ax,x,y,(floorZ+.015)*ones(1,4),orange, ...
            'EdgeColor',orange,'LineWidth',1.8,'FaceAlpha',1,'HandleVisibility','off');
    end
    % Legend proxy has no finite coordinates; source returns are colored once.
    truth=plot3(ax,NaN,NaN,NaN,'.','Color',orange, ...
        'DisplayName','Stored original fine pole points');
    cellKey=patch(ax,NaN,NaN,NaN,orange,'EdgeColor',orange,'DisplayName','Detected coarse pole pillars');
    axis(ax,'equal');axis(ax,'tight');grid(ax,'off');view(ax,-38,24);axis(ax,'vis3d');
    xlabel(ax,'X (m)');ylabel(ax,'Y (m)');zlabel(ax,'Z (m)');
    legend(ax,[cloud truth cellKey],'Location','northeast','TextColor',foreground, ...
        'Color',background,'EdgeColor',[.25 .3 .36],'AutoUpdate','off');
    title(ax,{sprintf('Mississippi | frame %d / %d',frameIndex,totalFrames), ...
        sprintf('Full cloud: %s points   |   Fine pole reference: %d points   |   Coarse selection: %d pillars', ...
        char(string(nnz(finite))),numel(fine),numel(selected))},'Color',foreground,'FontSize',14);
    annotation(fig,'textbox',[.04 .055 .92 .045],'String', ...
        sprintf('Orange points: fine pole reference. Orange cells: coarse detections. Coverage: %d/%d; false cells: %d/%d.\nGrid lowered to Z = %.2f m for display. Drag to rotate; point and scroll to zoom.', ...
        alignment.coveredFinePointCount,alignment.finePointCountInRoi,alignment.extraPillarCount, ...
        alignment.candidatePillarCount,floorZ),'Color',foreground,'EdgeColor','none','FontSize',11);
    rotate3d(fig,'on');drawnow;
    properties={'XLim','YLim','ZLim','DataAspectRatio','PlotBoxAspectRatio', ...
        'CameraPosition','CameraTarget','CameraUpVector','CameraViewAngle','Projection'};
    home=struct();for k=1:numel(properties),home.(properties{k})=get(ax,properties{k});end
    focusPoints=xyz(fine,:);
    if ~isempty(selected),focusPoints=[focusPoints;lower+spacing/2,repmat(floorZ,numel(selected),1)];end
    if isempty(focusPoints),focusPoints=points;end
    controls=struct('axes',ax,'cloud',cloud,'home',home,'focusPoints',focusPoints);
    setappdata(fig,'PolePillarViewControls',controls);
    z=zoom(fig);z.ActionPreCallback=@(~,~)focusPolePillarView(fig,'pointer');
    mode=getuimode(fig,'Exploration.Rotate3d');nativeWheel=mode.WindowScrollWheelFcn;
    mode.WindowScrollWheelFcn=@(src,event)wheelZoom(src,event,fig,nativeWheel);
    uicontrol(fig,'Style','pushbutton','String','Full scene','Units','normalized', ...
        'Position',[.04 .012 .10 .031],'Callback',@(~,~)focusPolePillarView(fig,'full'),'Tag','PoleFullScene');
    uicontrol(fig,'Style','pushbutton','String','Pole region','Units','normalized', ...
        'Position',[.15 .012 .10 .031],'Callback',@(~,~)focusPolePillarView(fig,'poles'),'Tag','PoleRegion');
    resetplotview(ax,'SaveCurrentView');
    assert(numel(cloud.XData)==nnz(finite));
    assert(numel(cellHandles)==numel(selected));
    assert(isscalar(cloud.SizeData) && cloud.SizeData==4);
    assert(isscalar(findobj(ax,'Type','scatter')));
    assert(nnz(all(cloud.CData==orange,2))==numel(fine));
    metrics=struct('dataset','Mississippi','frame',frameIndex,'totalFrames',totalFrames, ...
        'sourcePoints',size(xyz,1),'finiteSourcePoints',nnz(finite),'displayedSourcePoints',numel(cloud.XData), ...
        'finePolePoints',numel(fine),'selectedPillarIds',selected,'referencePillarIds',detail.finePillarIds, ...
        'alignment',alignment,'gridGeometry',geometry,'displayGridZ',floorZ, ...
        'displayXYBounds',[min([low(1:2);origin],[],1);max([high(1:2);[xEdges(end) yEdges(end)]],[],1)], ...
        'highlightRGB',orange,'markerSize',4,'featureOverlay',false,'cameraZoomStyle',z.getAxes3DPanAndZoomStyle(ax), ...
        'scoreThreshold',cfg.offGroundFeatures.pole.distributionValidation.minimumScore, ...
        'referenceSource','Frozen original fine point indices from output/pole_precision_20260927/mississippi.mat');
    result=struct('figure',fig,'axes',ax,'frame',frame,'perception',p, ...
        'reference',ref,'metrics',metrics,'sourceScatter',cloud,'referenceLegend',truth,'pillarPatches',cellHandles);
    setappdata(fig,'PolePillarComparison',metrics);
    if strlength(exportFolder)>0
        if ~isfolder(exportFolder),mkdir(exportFolder);end
        stem=sprintf('mississippi_%04d',frameIndex);
        exportgraphics(fig,fullfile(exportFolder,[stem '.png']),'Resolution',150,'BackgroundColor',background);
        file=fopen(fullfile(exportFolder,[stem '.json']),'w');
        cleanup=onCleanup(@()fclose(file));fprintf(file,'%s\n',jsonencode(metrics,PrettyPrint=true));
    end
    disp(metrics);
end

function wheelZoom(src,event,fig,nativeWheel)
    if event.VerticalScrollCount<0,focusPolePillarView(fig,'pointer');end
    if iscell(nativeWheel),feval(nativeWheel{1},src,event,nativeWheel{2:end});
    else,feval(nativeWheel,src,event);end
end
