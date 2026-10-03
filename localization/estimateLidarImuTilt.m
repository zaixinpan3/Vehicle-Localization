function tilt=estimateLidarImuTilt(imu,wheel,queryTime,cfg)
% estimateLidarImuTilt Replay the streaming filter using past packets only.
% imu arrivalTime/deviceTime, specificForce, angularVelocity; wheel
% arrivalTime/speed. Query times are packet-arrival clock times, in seconds.
    if nargin<4,cfg=lidarImuTiltConfig();end
    queryTime=queryTime(:);n=numel(queryTime);
    assert(all(diff(queryTime)>0),'VehicleLocalization:InvalidTiltClock','Queries must increase.');
    assert(all(diff(imu.arrivalTime)>0) && all(diff(imu.deviceTime)>0) && ...
        all(diff(wheel.arrivalTime)>0),'VehicleLocalization:InvalidTiltClock','Sensor clocks must increase.');
    rotation=repmat(eye(3),1,1,n);angles=zeros(n,2);valid=false(n,1);aligned=valid;
    age=nan(n,1);sourceIndex=zeros(n,1);state=[];i=0;j=0;latest=eye(3);last=struct();
    for k=1:n
        while i<numel(imu.arrivalTime) && imu.arrivalTime(i+1)<=queryTime(k)
            i=i+1;
            while j<numel(wheel.arrivalTime) && wheel.arrivalTime(j+1)<=imu.arrivalTime(i),j=j+1;end
            if j==0,continue;end
            assert(imu.arrivalTime(i)-wheel.arrivalTime(j)<=cfg.maximumWheelAgeSeconds, ...
                'VehicleLocalization:TiltWheelGap','Tilt compensation requires a current wheel sample.');
            sample=struct('time',imu.deviceTime(i),'specificForce',imu.specificForce(i,:), ...
                'angularVelocity',imu.angularVelocity(i,:));
            [latest,state,last]=updateLidarImuTilt(sample,wheel.speed(j),state,cfg);
        end
        if ~isempty(state)
            age(k)=queryTime(k)-imu.arrivalTime(i);
            assert(age(k)<=cfg.maximumImuAgeSeconds,'VehicleLocalization:TiltImuGap','Latest IMU sample is stale.');
            rotation(:,:,k)=latest;angles(k,:)=[last.roll,last.pitch];valid(k)=true;aligned(k)=last.aligned;sourceIndex(k)=i;
        end
    end
    tilt=struct('rotation',rotation,'angles',angles,'valid',valid,'aligned',aligned, ...
        'ageSeconds',age,'sourceIndex',sourceIndex,'time',queryTime,'configuration',cfg, ...
        'schemaVersion',1,'source',"front-ouster-raw-imu",'referencePoseUsed',false,'causal',true);
end
