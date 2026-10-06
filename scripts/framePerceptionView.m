function framePerceptionView(fig, bounds)
% framePerceptionView: Fit a stable orbit around a selected world-space box.
% Bounds are [xmin xmax; ymin ymax; zmin zmax]. With no bounds, fit every
% displayed source return. All points and original indices remain present;
% a local view changes the camera rather than removing or clipping points.
    arguments
        fig (1,1) matlab.ui.Figure
        bounds double = []
    end
    cloud=findobj(fig,'Type','scatter','Tag','pcviewer');
    assert(isscalar(cloud),'perception:ViewerCloud','Expected one live perception cloud.');
    ax=ancestor(cloud,'axes');
    if isempty(bounds)
        xyz=reshape(double(cloud.PointCloud.Location),[],3);
        bounds=[min(xyz,[],1);max(xyz,[],1)].';
    end
    assert(isequal(size(bounds),[3 2]) && all(isfinite(bounds),'all') && ...
        all(bounds(:,2)>=bounds(:,1)), ...
        'perception:ViewBounds','Bounds must be a finite ordered 3-by-2 box.');
    center=mean(bounds,2).';
    span=max(bounds(:,2)-bounds(:,1),.1);
    bounds=[center(:)-span/2,center(:)+span/2];
    % Equal metric scale and a sphere fit remain valid at every orbit angle.
    % A manual plot-box ratio inherited from a previous view distorts zoom.
    set(ax,'DataAspectRatio',[1 1 1],'PlotBoxAspectRatioMode','auto', ...
        'XLim',bounds(1,:),'YLim',bounds(2,:),'ZLim',bounds(3,:), ...
        'Projection','orthographic','Clipping','off');
    cloud.Clipping='off';
    direction=[1 -1 .75];direction=direction/norm(direction);
    angle=30;radius=norm(span)/2;
    units=ax.Units;ax.Units='pixels';position=ax.Position;ax.Units=units;
    aspect=max(position(3),1)/max(position(4),1);
    distance=1.15*radius/(tand(angle/2)*min(1,aspect));
    camtarget(ax,center);campos(ax,center+direction*distance);
    camup(ax,[0 0 1]);camva(ax,angle);
    % pcshow's native center orbit uses axis limits. Keep its center and the
    % fitted camera target identical, including after a regional view change.
    interaction=ax.PCUserData;
    interaction.rotateFromCenter=true;
    interaction.dataLimits=reshape(bounds.',1,[]);
    ax.PCUserData=interaction;
    resetplotview(ax,'SaveCurrentView');
    drawnow;
end
