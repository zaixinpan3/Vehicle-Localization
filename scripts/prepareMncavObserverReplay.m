function [sensorData,reference,metadata] = prepareMncavObserverReplay(sensorFolder,parameterFile,calls,fixedLidarDelay)
% prepareMncavObserverReplay Align recorded inputs and timestamped pose events.
% CAN inputs use previous-sample hold and independent-drive fixed corrections.
% GNSS positions and interpolated GNSS/INS yaw are reference data; only GPS XY
% events and one initial heading enter the global observer. Lidar events carry
% the matching algorithm's full physical [X,Y,psi] information matrix.
    arguments
        sensorFolder (1,1) string
        parameterFile (1,1) string
        calls table = table()
        fixedLidarDelay (1,1) double {mustBeNonnegative} = .15
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    folder=fullfile(root,'data','raw','Missisipi','gnss');
    stem="raw_data_2024-06-07-12-09-31_0";
    ins=readtable(fullfile(folder,stem+"_inspva.csv"));
    odom=readtable(fullfile(folder,stem+"_odom.csv"));
    poses=readFramePoseTable(fullfile(folder,stem+"_front_lidar_pose_match_1_1170.csv"),1:1170);
    twist=readtable(fullfile(sensorFolder,'twist.csv'));
    imu=readtable(fullfile(sensorFolder,'imu.csv'));
    steering=readtable(fullfile(sensorFolder,'steering.csv'));
    parameters=jsondecode(fileread(parameterFile));
    rosOrigin=ins.stamp_sec(1); receiver=ins.gps_seconds-ins.gps_seconds(1);
    bridge=@(stamp) interp1(ins.stamp_sec-rosOrigin,receiver,stamp-rosOrigin,'linear','extrap');
    start=bridge(poses.lidar_stamp_sec(1));
    motionTime=bridge(twist.stamp_sec)-start;
    imuTime=bridge(imu.stamp_sec)-start;
    steeringTime=bridge(steering.stamp_sec)-start;
    last=min([bridge(poses.lidar_stamp_sec(end))-start,motionTime(end),imuTime(end),steeringTime(end)]);
    time=(0:.01:last).';
    assert(max([motionTime(1),imuTime(1),steeringTime(1)])<=0,'Inputs do not cover replay start.');
    high=struct('time',time);
    high.longitudinalSpeed=interp1(motionTime,twist.linear_x_mps,time,'previous');
    high.steeringAngle=interp1(steeringTime,steering.steering_wheel_angle_rad,time,'previous')/parameters.steeringRatio;
    rawFields=["acceleration_x_mps2","acceleration_y_mps2","angular_z_radps"];
    names=["longitudinalAcceleration","lateralAcceleration","yawRate"];
    for k=1:3
        correction=parameters.input_correction.(names(k));
        high.(names(k))=correction.sign*interp1(imuTime,imu.(rawFields(k)),time,'previous')+correction.offset;
    end
    sensorData=struct('highRate',high);
    odomTime=bridge(odom.stamp_sec)-start;
    yaw=unwrap(atan2(2*(odom.qw.*odom.qz+odom.qx.*odom.qy),1-2*(odom.qy.^2+odom.qz.^2)));
    edgeExtrapolation=max([0,odomTime(1)-time(1),time(end)-odomTime(end)]);
    assert(edgeExtrapolation<=.04,'Reference edge gap exceeds 40 ms.');
    reference=array2table([time,interp1(odomTime,[odom.x_m,odom.y_m,yaw],time,'linear','extrap')], ...
        'VariableNames',{'time','x','y','psi'});
    assert(all(isfinite(reference{:,:}),'all'),'Reference does not cover replay.');
    selected=(1:5:height(odom)).'; selected=selected(odomTime(selected)>=0 & odomTime(selected)<=time(end));
    sensorData.gps=struct('timestamp',odomTime(selected), ...
        'arrivalTime',max(odomTime(selected),bridge(odom.bag_time_sec(selected))-start), ...
        'pose',[odom.x_m(selected),odom.y_m(selected)]);
    sensorData.lidar=struct();
    if ~isempty(calls)
        assert(all(ismember(calls.accepted,[0 1])),'Invalid recorded acceptance flag.');
        accepted=calls.accepted==1; n=nnz(accepted); c=calls(accepted,:);
        information=zeros(3,3,n);
        for k=1:n
            information(:,:,k)=[c.informationXX(k),c.informationXY(k),c.informationXPsi(k); ...
                c.informationXY(k),c.informationYY(k),c.informationYPsi(k); ...
                c.informationXPsi(k),c.informationYPsi(k),c.informationPsiPsi(k)];
            [~,failure]=chol(information(:,:,k));
            assert(failure==0,'Accepted scan lacks positive definite information.');
        end
        sensorData.lidar=struct('timestamp',c.timeSeconds, ...
            'arrivalTime',c.timeSeconds+fixedLidarDelay,'pose',[c.x,c.y,c.psi],'information',information);
    end
    metadata=struct('fixedLidarDelaySeconds',fixedLidarDelay, ...
        'clock',"INSPVA receiver time bridged from ROS headers; no pose used to fit the clock", ...
        'inputInterpolation',"causal previous-sample hold", ...
        'reference',"NovAtel odom body-origin XY and quaternion yaw; also mapping reference", ...
        'gpsInput',"every fifth recorded odom position, approximately 10 Hz; no continuous GNSS heading input", ...
        'lidarInput',"accepted fresh coarse-perception D2D poses and full information; fixed delivery delay", ...
        'parameters',parameters,'sampleTimeSeconds',.01,'samples',numel(time), ...
        'maximumReferenceEdgeExtrapolationSeconds',edgeExtrapolation);
end
