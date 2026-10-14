function [measurements,auxiliary]=prepareNuscenesMotion(imu,wheels,steering,trial)
% prepareNuscenesMotion Sensor-only, causal preparation on the native IMU clock.
% Accept only ms_imu, zoe_veh_info and steeranglefeedback messages. No pose,
% navigation velocity or reference trajectory is read. Wheel/steer packets use
% last received samples; all streams must have coverage with bounded age.
% IMU quaternion supplies tilt only; its arbitrary yaw is discarded.
    t=double([imu.utime].');assert(all(diff(t)>0),'VehicleLocalization:InvalidNuscenesClock','IMU clock must increase.');
    wt=double([wheels.utime].');st=double([steering.utime].');
    assert(all(diff(wt)>0) && all(diff(st)>0),'VehicleLocalization:InvalidNuscenesClock','CAN clocks must increase.');
    keep=t>=max(wt(1),st(1)) & t<=min(wt(end),st(end));imu=imu(keep);t=t(keep);
    assert(numel(t)>=2,'VehicleLocalization:MissingNuscenesCoverage','No common sensor interval.');
    wi=interp1(wt,(1:numel(wt)).',t,'previous');si=interp1(st,(1:numel(st)).',t,'previous');
    age=[t-wt(wi),t-st(si)]/1e6;
    assert(all(age<=trial.maximumCanAgeS,'all'),'VehicleLocalization:StaleNuscenesCan','CAN packet age exceeds the declared limit.');
    rpm=[[wheels(wi).FL_wheel_speed].',[wheels(wi).FR_wheel_speed].', ...
        [wheels(wi).RL_wheel_speed].',[wheels(wi).RR_wheel_speed].'];
    wheelMps=rpm*(2*pi*trial.wheelRadiusM/60);vx=mean(wheelMps(:,3:4),2);
    q=reshape([imu.q],4,[]).';q=q./vecnorm(q,2,2);
    phi=atan2(2*(q(:,1).*q(:,2)+q(:,3).*q(:,4)),1-2*(q(:,2).^2+q(:,3).^2));
    theta=asin(max(-1,min(1,2*(q(:,1).*q(:,3)-q(:,4).*q(:,2)))));
    force=reshape([imu.linear_accel],3,[]).';gyro=reshape([imu.rotation_rate],3,[]).';
    % Horizontal specific force equals horizontal acceleration after leveling.
    ax=cos(theta).*force(:,1)+sin(theta).*(sin(phi).*force(:,2)+cos(phi).*force(:,3));
    ay=cos(phi).*force(:,2)-sin(phi).*force(:,3);
    r=(sin(phi).*gyro(:,2)+cos(phi).*gyro(:,3))./cos(theta);
    time=(t-t(1))/1e6;rdot=zeros(size(r));
    for k=2:numel(t)
        dt=time(k)-time(k-1);a=-expm1(-dt/trial.yawDerivativeTimeConstantS);
        rdot(k)=rdot(k-1)+a*((r(k)-r(k-1))/dt-rdot(k-1));
    end
    measurements=struct('time',time,'longitudinalSpeed',vx, ...
        'steeringAngle',[steering(si).value].'/trial.steeringRatio, ...
        'longitudinalAcceleration',ax,'lateralAcceleration',ay,'yawRate',r);
    % Rigid-body acceleration transport from assumed CG to rear-axle origin.
    offset=trial.vehicle.lr;
    auxiliary=struct('utime',t,'tilt',[phi,theta],'wheelSpeedsMps',wheelMps, ...
        'packetAgeS',age,'rearAxleAcceleration',[ax+offset*r.^2,ay-offset*rdot], ...
        'yawAcceleration',rdot);
end
