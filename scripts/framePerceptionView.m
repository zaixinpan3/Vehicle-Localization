function framePerceptionView(fig, bounds)
% framePerceptionView: Fit a stable orbit around a selected world-space box.
% Bounds are [xmin xmax; ymin ymax; zmin zmax]. With no bounds, fit every
% displayed source return. All points and original indices remain present;
% a local view changes only the camera. Axis limits always contain every
% source return so native renderer clipping cannot discard the outer cloud.
    arguments
        fig (1,1) matlab.ui.Figure
        bounds double = []
    end
    cloud=findobj(fig,'Type','scatter','Tag','pcviewer');
    assert(isscalar(cloud),'perception:ViewerCloud','Expected one live perception cloud.');
    ax=ancestor(cloud,'axes');
    xyz=reshape(double(cloud.PointCloud.Location),[],3);
    sourceBounds=[min(xyz,[],1);max(xyz,[],1)].';
    wholeScene=isempty(bounds);
    if wholeScene,bounds=sourceBounds;end
    assert(isequal(size(bounds),[3 2]) && all(isfinite(bounds),'all') && ...
        all(bounds(:,2)>=bounds(:,1)), ...
        'perception:ViewBounds','Bounds must be a finite ordered 3-by-2 box.');
    center=mean(bounds,2).';
    span=max(bounds(:,2)-bounds(:,1),.1);
    % Equal metric scale and a sphere fit remain valid at every orbit angle.
    % A manual plot-box ratio inherited from a previous view distorts zoom.
    set(ax,'DataAspectRatio',[1 1 1],'PlotBoxAspectRatioMode','auto', ...
        'XLim',sourceBounds(1,:)+[-.01 .01],'YLim',sourceBounds(2,:)+[-.01 .01], ...
        'ZLim',sourceBounds(3,:)+[-.01 .01], ...
        'Projection','orthographic','Clipping','on');
    cloud.Clipping='on';
    direction=[1 -1 .75];direction=direction/norm(direction);
    angle=30;radius=norm(span)/2;
    units=ax.Units;ax.Units='pixels';position=ax.Position;ax.Units=units;
    aspect=max(position(3),1)/max(position(4),1);
    distance=1.15*radius/(tand(angle/2)*min(1,aspect));
    camtarget(ax,center);campos(ax,center+direction*distance);
    camup(ax,[0 0 1]);camva(ax,angle);
    % Native center rotation suits the whole scan. Regional views retain
    % pcshow's rotation around the clicked point, with full data bounds.
    interaction=ax.PCUserData;
    interaction.rotateFromCenter=wholeScene;
    interaction.dataLimits=[ax.XLim ax.YLim ax.ZLim];
    ax.PCUserData=interaction;
    resetplotview(ax,'SaveCurrentView');
    drawnow;
end
