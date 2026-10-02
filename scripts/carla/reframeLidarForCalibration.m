function frame=reframeLidarForCalibration(frame,source,target)
% reframeLidarForCalibration Express a scan in a virtual calibrated frame.
% Both transforms map their stored scan coordinates to the SAME body output
% point. Changing the stored coordinates preserves every physical body point:
% R_target*p_virtual+t_target = R_source*p_raw+t_source.
% This permits a new sensor mount to use a map with an explicit stored-frame
% contract without changing that map's geometry or bypassing its validator.
    source=validateLidarFrameCalibration(source);target=validateLidarFrameCalibration(target);
    xyz=double([frame.x(:),frame.y(:),frame.z(:)]);
    body=xyz*source.rotation.'+source.translation;
    virtual=(body-target.translation)*target.rotation;
    frame.x=cast(virtual(:,1),'like',frame.x);frame.y=cast(virtual(:,2),'like',frame.y);frame.z=cast(virtual(:,3),'like',frame.z);
end
