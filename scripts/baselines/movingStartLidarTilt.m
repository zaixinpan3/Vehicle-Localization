function rotation=movingStartLidarTilt(folder,queryTime,initialPose)
% Moving-start replay: one initial reference tilt, then only IMU/wheel inputs.
% The initial gyro bias is zero; no later receiver attitude is read here.
    s=readtable(fullfile(folder,'front_imu.csv'));w=readtable(fullfile(folder,'sensors','wheel_speed_report.csv'));wc=wheelSpeedObserverConfig();cfg=lidarImuTiltConfig();
    speed=mean([w.rear_left,w.rear_right].*wc.effectiveRadius(3:4),2);[R,~]=poseSupport.poseRowToRigidTransform(initialPose);
    state=[];rotation=repmat(eye(3),1,1,numel(queryTime));latest=eye(3);i=0;
    for k=1:numel(queryTime)
        while i<height(s) && s.arrivalTime(i+1)<=queryTime(k)
            i=i+1;j=find(w.bag_time_sec<=s.arrivalTime(i),1,'last');if isempty(j),continue;end
            if isempty(state)
                state=struct('time',s.deviceTime(i)-.001,'up',R(3,:).','gyroBias',zeros(3,1),'biasCount',0,'stationarySeconds',1,'speed',speed(j),'initialized',true);
            end
            sample=struct('time',s.deviceTime(i),'specificForce',[s.specificX(i),s.specificY(i),s.specificZ(i)],'angularVelocity',[s.gyroX(i),s.gyroY(i),s.gyroZ(i)]);
            [latest,state]=lidarCalibrationSupport.updateLidarImuTilt(sample,speed(j),state,cfg);
        end
        rotation(:,:,k)=latest;
    end
end
