function result=showMississippiPoleReference(frameIndex,exportFolder)
% showMississippiPoleReference: Inspect stored fine pole output on a full frame.
% Orange marks the exact cached original fine-perception point indices.
% Gray includes every other finite source point, without ROI cropping or
% downsampling. This view performs no coarse or fine perception inference.
    arguments
        frameIndex (1,1) double {mustBeInteger,mustBePositive}=500
        exportFolder (1,1) string=""
    end
    root=fileparts(fileparts(mfilename('fullpath')));addpath(root);setupVehicleLocalization;
    [frame,total]=loadPointCloudFrame(fullfile(root,'data','raw','MissisipiPointClouds.mat'),frameIndex);
    cache=load(fullfile(root,'output','pole_precision_20260927','mississippi.mat'),'frames','references');
    at=find(cache.frames==frameIndex,1);assert(~isempty(at),'No stored fine reference for this frame.');
    ref=cache.references{at};xyz=double([frame.x(:),frame.y(:),frame.z(:)]);
    indices=find(all(isfinite(xyz),2));fine=double(ref.pointIndices(:));
    assert(isequal(xyz(fine,:),double(ref.points)),'Cached reference coordinates do not match the source frame.');
    mask=ismember(indices,fine);assert(nnz(mask)==numel(fine));
    orange=[1 .43 .10];background=[.055 .065 .085];foreground=[.87 .90 .94];
    colors=repmat([.48 .53 .59],numel(indices),1);colors(mask,:)=repmat(orange,nnz(mask),1);
    fig=figure('Name',sprintf('Mississippi %d | original fine pole reference',frameIndex), ...
        'NumberTitle','off','Color',background,'Position',[80 80 1500 1000],'WindowStyle','normal');
    ax=axes(fig,'Position',[.06 .13 .90 .76]);
    pcshow(xyz(indices,:),colors,'Parent',ax,'MarkerSize',4,'Projection','orthographic', ...
        'BackgroundColor',background,'AxesVisibility','on');
    cloud=findobj(ax,'Type','scatter','Tag','pcviewer');cloud.DisplayName='Full source cloud';hold(ax,'on');
    % Legend proxy has no finite coordinates; source returns are colored once.
    truth=plot3(ax,NaN,NaN,NaN,'.','Color',orange, ...
        'DisplayName','Stored original fine pole points');
    axis(ax,'equal');axis(ax,'tight');view(ax,-38,28);axis(ax,'vis3d');
    xlabel(ax,'X (m)');ylabel(ax,'Y (m)');zlabel(ax,'Z (m)');
    title(ax,{sprintf('Mississippi | frame %d / %d | original fine pole reference',frameIndex,total), ...
        sprintf('Full source cloud: %d points | Highlighted fine pole points: %d',numel(indices),numel(fine))}, ...
        'Color',foreground,'FontSize',14);
    legend(ax,[cloud truth],'Location','northeast','TextColor',foreground,'Color',background,'AutoUpdate','off');
    annotation(fig,'textbox',[.04 .06 .92 .04],'String', ...
        'Orange: stored 0.3 m fine-perception pole points. Gray: other source points. Drag to rotate; point and scroll to zoom.', ...
        'Color',foreground,'EdgeColor','none','FontSize',11);
    drawnow;
    properties={'XLim','YLim','ZLim','DataAspectRatio','PlotBoxAspectRatio', ...
        'CameraPosition','CameraTarget','CameraUpVector','CameraViewAngle','Projection'};
    home=struct();for k=1:numel(properties),home.(properties{k})=get(ax,properties{k});end
    focus=xyz(fine,:);if isempty(focus),focus=xyz(indices,:);end
    setappdata(fig,'PolePillarViewControls',struct('axes',ax,'cloud',cloud,'home',home,'focusPoints',focus));
    z=zoom(fig);z.ActionPreCallback=@(~,~)focusPolePillarView(fig,'pointer');
    mode=getuimode(fig,'Exploration.Rotate3d');wheel=mode.WindowScrollWheelFcn;
    mode.WindowScrollWheelFcn=@(src,event)wheelZoom(src,event,fig,wheel);
    uicontrol(fig,'Style','pushbutton','String','Full scene','Units','normalized', ...
        'Position',[.04 .015 .10 .032],'Callback',@(~,~)focusPolePillarView(fig,'full'));
    uicontrol(fig,'Style','pushbutton','String','Pole region','Units','normalized', ...
        'Position',[.15 .015 .10 .032],'Callback',@(~,~)focusPolePillarView(fig,'poles'));
    resetplotview(ax,'SaveCurrentView');
    assert(numel(cloud.XData)==numel(indices));
    assert(isequal(cloud.PointCloud.Location,xyz(indices,:)) && isequal(cloud.ColorData,colors));
    assert(isscalar(cloud.SizeData) && cloud.SizeData==4);
    assert(isscalar(findobj(ax,'Type','scatter')));
    assert(nnz(all(cloud.CData==orange,2))==numel(fine));
    metrics=struct('frame',frameIndex,'sourcePoints',size(xyz,1),'displayedFinitePoints',numel(indices), ...
        'finePolePoints',numel(fine),'originalPointIndices',fine,'highlightRGB',orange,'markerSize',4,'featureOverlay',false, ...
        'cameraZoomStyle',z.getAxes3DPanAndZoomStyle(ax),'perceptionRerun',false, ...
        'source','Stored original 0.3 m fine pole reference; not manual annotations');
    result=struct('figure',fig,'axes',ax,'sourceScatter',cloud,'referenceLegend',truth,'metrics',metrics);
    if strlength(exportFolder)>0
        if ~isfolder(exportFolder),mkdir(exportFolder);end
        stem=sprintf('fine_reference_%04d',frameIndex);
        exportgraphics(fig,fullfile(exportFolder,[stem '.png']),'Resolution',120,'BackgroundColor',background);
        file=fopen(fullfile(exportFolder,[stem '.json']),'w');cleanup=onCleanup(@()fclose(file));
        fprintf(file,'%s\n',jsonencode(metrics,PrettyPrint=true));
    end
    fprintf('Frame %d: all %d finite points; %d stored fine pole points; no perception rerun.\n',frameIndex,numel(indices),numel(fine));
end

function wheelZoom(src,event,fig,nativeWheel)
    if event.VerticalScrollCount<0,focusPolePillarView(fig,'pointer');end
    if iscell(nativeWheel),feval(nativeWheel{1},src,event,nativeWheel{2:end});
    else,feval(nativeWheel,src,event);end
end
