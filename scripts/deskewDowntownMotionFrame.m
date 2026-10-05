function frame=deskewDowntownMotionFrame(frame,inputs)
% deskewDowntownMotionFrame Sensor-only planar motion compensation for queries.
% Bracketed recorded wheel/gyro/lateral motion; no reference/map pose input.
    t=inputs.highRate.time+inputs.nativeOriginSeconds;query=frame.timestamp+double(frame.pointTimeSeconds(:));
    assert(min(query)>=t(1)-1e-7 && max(query)<=t(end)+1e-7, ...
        'VehicleLocalization:QueryDeskewCoverage','Measured motion must cover every query point.');
    [uq,~,group]=unique(query);p=interp1(t,inputs.motion,uq,'linear');p0=interp1(t,inputs.motion,frame.timestamp,'linear');
    x=double(frame.x(:));y=double(frame.y(:));z=double(frame.z(:));angle=p(group,3)-p0(3);
    noReturn=(x==0 & y==0 & z==0) | ~isfinite(x) | ~isfinite(y) | ~isfinite(z);
    R0=[cos(p0(3)),-sin(p0(3));sin(p0(3)),cos(p0(3))];translation=(p(group,1:2)-p0(1:2))*R0;
    xx=cos(angle).*x-sin(angle).*y+translation(:,1);yy=sin(angle).*x+cos(angle).*y+translation(:,2);
    xx(noReturn)=NaN;yy(noReturn)=NaN;
    frame.x=reshape(single(xx),size(frame.x));frame.y=reshape(single(yy),size(frame.y));
end
