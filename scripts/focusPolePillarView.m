function focusPolePillarView(fig,mode,ray)
% focusPolePillarView: Reframe the diagnostic without removing source points.
% Pointer focus uses the nearest original return to the current viewing ray.
% The optional explicit ray supports deterministic interaction regression.
    arguments
        fig (1,1) matlab.ui.Figure
        mode (1,1) string {mustBeMember(mode,["full","poles","pointer"])}
        ray double=[]
    end
    state=getappdata(fig,'PolePillarViewControls');ax=state.axes;
    if mode=="full"
        for key=fieldnames(state.home).',set(ax,key{1},state.home.(key{1}));end
        drawnow;return;
    end
    if mode=="poles"
        lower=min(state.focusPoints,[],1)-2;upper=max(state.focusPoints,[],1)+2;
        target=(lower+upper)/2;angle=state.home.CameraViewAngle;
        direction=state.home.CameraPosition-state.home.CameraTarget;direction=direction/norm(direction);
        radius=norm(upper-lower)/2;distance=1.15*radius/tand(angle/2);
        ax.CameraTarget=target;ax.CameraPosition=target+direction*distance;ax.CameraViewAngle=angle;
        drawnow;return;
    end
    if isempty(ray),ray=ax.CurrentPoint;end
    if ~isequal(size(ray),[2 3]) || any(~isfinite(ray),'all'),return;end
    direction=ray(2,:)-ray(1,:);if norm(direction)<eps,return;end
    direction=direction/norm(direction);
    p=state.cloud.PointCloud.Location;offset=double(p)-ray(1,:);depth=offset*direction.';
    distanceSquared=max(0,sum(offset.^2,2)-depth.^2);
    if strcmp(ax.Projection,'perspective'),distanceSquared=distanceSquared./max(depth.^2,eps);end
    distanceSquared(depth<0)=Inf;
    [distance,index]=min(distanceSquared);if ~isfinite(distance),return;end
    target=double(p(index,:));delta=target-ax.CameraTarget;
    ax.CameraPosition=ax.CameraPosition+delta;ax.CameraTarget=target;
end
