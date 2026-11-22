function initial=initializeMncavVdbFromGnss(s,inputs,startTime)
% Initialize from the native GNSS pose at the requested acquisition time.
    assert(ismember('gnssHeadingRad',s.Properties.VariableNames), ...
        'VehicleLocalization:InitializationUnavailable','Regenerate the measurement export with GNSS heading.');
    k=find(abs(s.time-startTime)<1e-7 & s.gnssValid==1,1);
    assert(~isempty(k),'VehicleLocalization:InitializationUnavailable','A GNSS pose is required at initialization.');
    position=[s.gnssX(k),s.gnssY(k)];heading=s.gnssHeadingRad(k);
    assert(all(isfinite([position,heading])),'VehicleLocalization:InitializationUnavailable', ...
        'The initial GNSS pose must be finite.');
    h=inputs.highRate;[gap,i]=min(abs(h.time-startTime));assert(gap<1e-7);
    R=[cos(heading),-sin(heading);sin(heading),cos(heading)];
    v=R*[h.longitudinalSpeed(i);inputs.lateral.lateralVelocity(i)];
    initial=struct('pose',[position,heading],'state',[position(1);v(1);0;position(2);v(2);0;heading], ...
        'source',"Measured GNSS position and independent heading",'firstGnssTime',s.time(k), ...
        'lastGnssTime',s.time(k),'gnssSamples',1);
end
