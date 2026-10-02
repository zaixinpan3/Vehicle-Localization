function initial=initializeMncavVdbFromGnss(s,inputs,startTime)
% initializeMncavVdbFromGnss Bootstrap from past GNSS displacement and IMU.
% Heading uses a straight-motion GNSS fit, never a reference quaternion.
% All scenarios share this measured bootstrap; "LiDAR only" begins afterward.
    selected=s.time>=startTime-3 & s.time<=startTime & s.gnssValid==1;
    t=s.time(selected);xy=[s.gnssX(selected),s.gnssY(selected)];
    assert(numel(t)>=15 && all(isfinite(xy),'all'),'VehicleLocalization:InitializationUnavailable', ...
        'Need at least 15 valid native GNSS observations in the preceding three seconds.');
    D=[ones(size(t)),t-startTime];fit=D\xy;velocity=fit(2,:);
    baseline=norm(xy(end,:)-xy(1,:));
    h=inputs.highRate;span=h.time>=t(1) & h.time<=startTime;
    assert(baseline>=5 && norm(velocity)>=2 && max(abs(h.yawRate(span)))<.05, ...
        'VehicleLocalization:InitializationUnavailable','Need five metres of approximately straight measured motion.');
    [gap,k]=min(abs(h.time-startTime));assert(gap<1e-7);
    beta=atan2(inputs.lateral.lateralVelocity(k),h.longitudinalSpeed(k));
    heading=atan2(velocity(2),velocity(1))-beta;
    position=xy(end,:);R=[cos(heading),-sin(heading);sin(heading),cos(heading)];
    v=R*[h.longitudinalSpeed(k);inputs.lateral.lateralVelocity(k)];
    a=R*[h.longitudinalAcceleration(k);h.lateralAcceleration(k)];
    initial=struct('pose',[position,heading],'state',[position(1);v(1);a(1);position(2);v(2);a(2);heading], ...
        'source',"Past native GNSS positions plus measured-motion sideslip; no truth", ...
        'firstGnssTime',t(1),'lastGnssTime',t(end),'gnssSamples',numel(t),'displacementM',baseline);
end
