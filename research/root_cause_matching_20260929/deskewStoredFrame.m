function frame=deskewStoredFrame(frame,pointTimes,twist,phase,calibration,tilt)
% deskewStoredFrame Compensate a recorded scan under constant planar body twist.
% Point times are verified sensor-relative seconds. phase is the explicitly
% assumed sensor-relative epoch of the cloud header, not an inferred GPS offset.
% Calibration and known tilt locate the sensor lever arm before compensation.
    assert(isequal(size(frame.x),size(pointTimes)) && all(isfinite(pointTimes),'all'), ...
        'VehicleLocalization:PointTimeShape','Supply a finite time for every point in its original index order.');
    assert(numel(twist)==3 && all(isfinite(twist)) && isscalar(phase) && isfinite(phase), ...
        'VehicleLocalization:DeskewMotion','Require finite body twist and an explicit finite phase.');
    if all(twist==0),return;end
    xyz=double([frame.x(:),frame.y(:),frame.z(:)]);dt=double(pointTimes(:))-phase;
    R=tilt*calibration.rotation;t=calibration.translation*tilt.';p=xyz*R.'+t;
    angle=twist(3)*dt;ca=cos(angle);sa=sin(angle);
    if abs(twist(3))<1e-10,a=dt;b=zeros(size(dt));else,a=sa/twist(3);b=2*sin(angle/2).^2/twist(3);end
    xy=[ca.*p(:,1)-sa.*p(:,2)+a*twist(1)-b*twist(2), ...
        sa.*p(:,1)+ca.*p(:,2)+b*twist(1)+a*twist(2)];
    corrected=([xy,p(:,3)]-t)*R;
    valid=isfinite(xyz(:,1))&isfinite(xyz(:,2))&isfinite(xyz(:,3))&any(xyz~=0,2);
    xyz(valid,:)=corrected(valid,:);frame.x=reshape(cast(xyz(:,1),'like',frame.x),size(frame.x));
    frame.y=reshape(cast(xyz(:,2),'like',frame.y),size(frame.y));frame.z=reshape(cast(xyz(:,3),'like',frame.z),size(frame.z));
end
