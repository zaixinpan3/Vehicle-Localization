function frame=deskewReferenceMappingFrame(frame,poses)
% deskewReferenceMappingFrame Offline map-only full-point SE3 interpolation.
% Reference poses are forbidden in the localization counterpart. No extrapolation.
    query=frame.timestamp+double(frame.pointTimeSeconds(:));t=poses.lidar_stamp_sec;
    assert(min(query)>=t(1)-1e-7 && max(query)<=t(end)+1e-7, ...
        'VehicleLocalization:MappingDeskewCoverage','Mapping scan needs covered reference motion.');
    xyz=[double(frame.x(:)),double(frame.y(:)),double(frame.z(:))];
    noReturn=all(xyz==0,2) | ~all(isfinite(xyz),2);
    [uq,~,group]=unique(query);[~,~,bin]=histcounts(uq,t);bin=min(max(bin,1),numel(t)-1);
    alpha=(uq-t(bin))./(t(bin+1)-t(bin));
    q=[poses.pose_qw,poses.pose_qx,poses.pose_qy,poses.pose_qz];q=q./vecnorm(q,2,2);
    qa=q(bin,:);qb=q(bin+1,:);flip=sum(qa.*qb,2)<0;qb(flip,:)=-qb(flip,:);
    dotp=min(1,max(-1,sum(qa.*qb,2)));angle=acos(dotp);small=angle<1e-8;
    a=sin((1-alpha).*angle)./sin(angle);b=sin(alpha.*angle)./sin(angle);
    a(small)=1-alpha(small);b(small)=alpha(small);qq=a.*qa+b.*qb;qq=qq./vecnorm(qq,2,2);
    p=[poses.pose_x_m,poses.pose_y_m,poses.pose_z_m];position=interp1(t,p,uq,'linear');
    qq=qq(group,:);position=position(group,:);
    qw=qq(:,1);v=qq(:,2:4);rotated=xyz+2*cross(v,cross(v,xyz,2)+qw.*xyz,2);
    [R,p0]=poseRowToRigidTransform(poses(find(abs(t-frame.timestamp)<1e-7,1),:));
    local=(rotated+position-p0)*R;
    local(noReturn,:)=NaN;
    frame.x=reshape(single(local(:,1)),size(frame.x));frame.y=reshape(single(local(:,2)),size(frame.y));frame.z=reshape(single(local(:,3)),size(frame.z));
end
