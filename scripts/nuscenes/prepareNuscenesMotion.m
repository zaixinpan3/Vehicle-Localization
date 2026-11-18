function [measurements,auxiliary]=prepareNuscenesMotion(imu,vehicleInfo,steering,trial)
% prepareNuscenesMotion Sensor-only, causal preparation on the native IMU clock.
% Accept only ms_imu, zoe_veh_info and steeranglefeedback messages. No pose,
% navigation velocity or reference trajectory is read. Speed/steer packets use
% last received samples; all streams must have coverage with bounded age.
% IMU quaternion supplies tilt only; its arbitrary yaw is discarded.
    t=double([imu.utime].');assert(all(diff(t)>0),'VehicleLocalization:InvalidNuscenesClock','IMU clock must increase.');
    vt=double([vehicleInfo.utime].');st=double([steering.utime].');
    assert(all(diff(vt)>0) && all(diff(st)>0),'VehicleLocalization:InvalidNuscenesClock','CAN clocks must increase.');
    keep=t>=max(vt(1),st(1)) & t<=min(vt(end),st(end));imu=imu(keep);t=t(keep);
    assert(numel(t)>=2,'VehicleLocalization:MissingNuscenesCoverage','No common sensor interval.');
    vi=interp1(vt,(1:numel(vt)).',t,'previous');si=interp1(st,(1:numel(st)).',t,'previous');
    age=[t-vt(vi),t-st(si)]/1e6;
    assert(all(age<=trial.maximumCanAgeS,'all'),'VehicleLocalization:StaleNuscenesCan','CAN packet age exceeds the declared limit.');
    % The Zoe CAN odometer reports vehicle speed directly in km/h.
    speedKmh=[vehicleInfo(vi).odom_speed].';
    assert(iscolumn(speedKmh) && numel(speedKmh)==numel(t) && all(isfinite(speedKmh)), ...
        'VehicleLocalization:InvalidNuscenesSpeed','Require one finite CAN vehicle speed per aligned sample.');
    vx=double(speedKmh)/3.6;
    q=reshape([imu.q],4,[]).';q=q./vecnorm(q,2,2);
    phi=atan2(2*(q(:,1).*q(:,2)+q(:,3).*q(:,4)),1-2*(q(:,2).^2+q(:,3).^2));
    theta=asin(max(-1,min(1,2*(q(:,1).*q(:,3)-q(:,4).*q(:,2)))));
    force=reshape([imu.linear_accel],3,[]).';gyro=reshape([imu.rotation_rate],3,[]).';
    % Horizontal specific force equals horizontal acceleration after leveling.
    ax=cos(theta).*force(:,1)+sin(theta).*(sin(phi).*force(:,2)+cos(phi).*force(:,3));
    ay=cos(phi).*force(:,2)-sin(phi).*force(:,3);
    r=(sin(phi).*gyro(:,2)+cos(phi).*gyro(:,3))./cos(theta);
    time=(t-t(1))/1e6;
    measurements=struct('time',time,'longitudinalSpeed',vx, ...
        'steeringAngle',[steering(si).value].'/trial.steeringRatio, ...
        'longitudinalAcceleration',ax,'lateralAcceleration',ay,'yawRate',r);
    auxiliary=struct('utime',t,'tilt',[phi,theta],'packetAgeS',age);
end
