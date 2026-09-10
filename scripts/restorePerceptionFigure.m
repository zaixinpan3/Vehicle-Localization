function restorePerceptionFigure(fig)
% restorePerceptionFigure: Rebuild live pcshow interactions after openfig.
%   fig=openfig(filename); restorePerceptionFigure(fig) restores point-centered
%   rotation and datatips from the saved original points and semantic masks.
%   The camera, axis limits, marker size, labels and frame counter are retained.
%   pcshow's transient interaction state is not serialized by savefig.
    arguments
        fig (1,1) matlab.ui.Figure
    end
    source=getappdata(fig,'PointTipSource');
    assert(isstruct(source) && isfield(source,'xyz'), ...
        'perception:MissingDisplaySource','Use a saved perception figure.');
    cloud=findobj(fig,'Type','scatter');
    assert(isscalar(cloud),'Expected one original-point scatter.');
    ax=ancestor(cloud,'axes');
    properties={'XLim','YLim','ZLim','DataAspectRatio','PlotBoxAspectRatio', ...
        'CameraPosition','CameraTarget','CameraUpVector','CameraViewAngle','Projection'};
    state=struct();
    for k=1:numel(properties),state.(properties{k})=get(ax,properties{k});end
    markerSize=cloud.SizeData;marker=cloud.Marker;
    labels={ax.Title.String,ax.XLabel.String,ax.YLabel.String,ax.ZLabel.String};
    counter=findall(fig,'Tag','PerceptionFrameCounter');
    counterText='';if ~isempty(counter),counterText=counter.String;end
    datacursormode(fig,'off');rotate3d(fig,'off');
    legend(ax,'off');
    % Re-enter the public renderer to install fresh point-cloud callbacks.
    hold(ax,'off');
    valid=all(isfinite(source.xyz),2);
    pcshow(source.xyz(valid,:),[.42 .42 .46],'Parent',ax,'MarkerSize',markerSize, ...
        'Projection',state.Projection);
    cloud=findobj(ax,'Type','scatter','Tag','pcviewer');cloud.Marker=marker;
    for k=1:numel(properties),set(ax,properties{k},state.(properties{k}));end
    frame=struct('x',reshape(source.xyz(:,1),source.frameSize), ...
        'y',reshape(source.xyz(:,2),source.frameSize), ...
        'z',reshape(source.xyz(:,3),source.frameSize));
    setappdata(fig,'PerceptionDataset',source.dataset);
    updatePerceptionDisplay(fig,frame,source.featureMasks,source.featureNames,source.frameIndex,source.frameIndex);
    if ~isempty(counterText),counter.String=counterText;end
    title(ax,labels{1},'Color','white');xlabel(ax,labels{2});ylabel(ax,labels{3});zlabel(ax,labels{4});
    rotate3d(fig,'on');
    drawnow;
end
