function initial=initializeMncavVdbFromGnss(s,inputs,startTime)
% Use the current position and a measured yaw, or a zero-radian initial guess.
    k=find(abs(s.time-startTime)<1e-7 & s.gnssValid==1,1);
    assert(~isempty(k),'VehicleLocalization:InitializationUnavailable','A GNSS position is required at initialization.');
    position=[s.gnssX(k),s.gnssY(k)];
    assert(all(isfinite(position)),'VehicleLocalization:InitializationUnavailable','The initial GNSS position must be finite.');
    heading=0;source="Measured GNSS position with a zero-radian heading prior";
    if ismember('gnssHeadingRad',s.Properties.VariableNames) && isfinite(s.gnssHeadingRad(k))
        heading=s.gnssHeadingRad(k);source="Measured GNSS position and independent heading";
    end
    h=inputs.highRate;[gap,i]=min(abs(h.time-startTime));assert(gap<1e-7);
    R=[cos(heading),-sin(heading);sin(heading),cos(heading)];
    v=R*[h.longitudinalSpeed(i);inputs.lateral.lateralVelocity(i)];
    initial=struct('pose',[position,heading],'state',[position(1);v(1);0;position(2);v(2);0;heading], ...
        'source',source,'firstGnssTime',s.time(k), ...
        'lastGnssTime',s.time(k),'gnssSamples',1);
end
