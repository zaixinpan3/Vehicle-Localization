function packet=globalObserverMotionMeasurements(lateral,signedSpeed,gyroRate)
% Prepare measured motion at the localization output point on the native clock.
% The suspended LPV state remains untouched. At a valid low-speed sample use
% the wheel/vehicle speed, a zero lateral-speed approximation, and the gyro.
    t=lateral.time(:);u=signedSpeed(:);r=gyroRate(:);
    assert(numel(u)==numel(t) && numel(r)==numel(t),'VehicleLocalization:InvalidMotionPacket', ...
        'Wheel/vehicle speed, gyro and lateral estimates must share a clock.');
    eligible=lateral.measurements.dynamicValid(:) & isfinite(u);
    low=eligible & abs(u)<lateral.minimumDesignSpeedMps;
    packet=struct('time',t,'longitudinalSpeed',lateral.longitudinalSpeed(:), ...
        'lateralVelocity',lateral.lateralVelocity(:),'yawRate',lateral.yawRate(:), ...
        'velocityValid',lateral.velocityValid(:),'yawRateValid',lateral.yawRateValid(:), ...
        'lowSpeedMeasurement',low);
    packet.longitudinalSpeed(low)=u(low);
    packet.lateralVelocity(low)=0;
    packet.velocityValid(low)=true;
    packet.yawRate(low)=r(low);
    gyroValid=isfinite(r);
    if isfield(lateral.measurements,'yawRateValid')
        mask=lateral.measurements.yawRateValid(:);
        assert(numel(mask)==numel(t) && all(ismember(mask,[0,1])), ...
            'VehicleLocalization:InvalidMotionValidity','Invalid native gyro validity mask.');
        gyroValid=gyroValid & logical(mask);
    end
    packet.yawRateValid(low)=gyroValid(low);
end
