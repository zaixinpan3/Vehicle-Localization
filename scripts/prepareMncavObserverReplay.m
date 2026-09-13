function [sensorData,reference,metadata] = prepareMncavObserverReplay(sensorFolder,parameterFile,calls,fixedLidarDelay)
% prepareMncavObserverReplay Export recorded inputs and physical pose samples.
% This is a data exporter. Its physical timestamp metadata is not an observer
% timing model. reconstructContinuousObserverSignals supplies the separately
% declared offline input reconstruction for the current continuous runner.
% Inputs use linear reconstruction and independent-drive fixed corrections.
% GNSS/INS yaw is reference data; only GNSS XY enters the measurement channel.
% LiDAR samples carry the full physical [X,Y,psi] information matrix.
% No delivery timestamps or artificial GNSS downsampling are introduced.
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
    high.longitudinalSpeed=interp1(motionTime,twist.linear_x_mps,time,'linear');
    high.steeringAngle=interp1(steeringTime,steering.steering_wheel_angle_rad,time,'linear')/parameters.steeringRatio;
    rawFields=["acceleration_x_mps2","acceleration_y_mps2","angular_z_radps"];
    names=["longitudinalAcceleration","lateralAcceleration","yawRate"];
    for k=1:3
        correction=parameters.input_correction.(names(k));
        high.(names(k))=correction.sign*interp1(imuTime,imu.(rawFields(k)),time,'linear')+correction.offset;
    end
    sensorData=struct('highRate',high);
    odomTime=bridge(odom.stamp_sec)-start;
    yaw=unwrap(atan2(2*(odom.qw.*odom.qz+odom.qx.*odom.qy),1-2*(odom.qy.^2+odom.qz.^2)));
    edgeExtrapolation=max([0,odomTime(1)-time(1),time(end)-odomTime(end)]);
    assert(edgeExtrapolation<=.04,'Reference edge gap exceeds 40 ms.');
    reference=array2table([time,interp1(odomTime,[odom.x_m,odom.y_m,yaw],time,'linear','extrap')], ...
        'VariableNames',{'time','x','y','psi'});
    assert(all(isfinite(reference{:,:}),'all'),'Reference does not cover replay.');
    selected=find(odomTime>=0 & odomTime<=time(end));
    sensorData.gps=struct('timestamp',odomTime(selected), ...
        'pose',[odom.x_m(selected),odom.y_m(selected)]);
    sensorData.lidar=struct();
    if ~isempty(calls)
        assert(all(ismember(calls.accepted,[0 1])),'Invalid recorded acceptance flag.');
        directional=false(height(calls),1);
        if ismember('directionalAccepted',calls.Properties.VariableNames)
            assert(all(ismember(calls.directionalAccepted,[0 1])),'Invalid directional acceptance flag.');
            directional=calls.directionalAccepted==1;
            assert(~any(directional & calls.accepted==1),'Full and directional acceptance must be distinct.');
        end
        accepted=calls.accepted==1 | directional; n=nnz(accepted); c=calls(accepted,:);
        information=zeros(3,3,n);
        for k=1:n
            information(:,:,k)=[c.informationXX(k),c.informationXY(k),c.informationXPsi(k); ...
                c.informationXY(k),c.informationYY(k),c.informationYPsi(k); ...
                c.informationXPsi(k),c.informationYPsi(k),c.informationPsiPsi(k)];
            if c.accepted(k)
                [~,failure]=chol(information(:,:,k));
                assert(failure==0,'Accepted full scan lacks positive definite information.');
            else
                values=eig(information(:,:,k));tol=1e-10*max(1,max(abs(values)));
                assert(min(values)>=-tol && max(values)>tol && ismember(c.rank(k),[1 2]), ...
                    'Accepted directional scan lacks nonzero PSD information.');
            end
        end
        sensorData.lidar=struct('timestamp',c.timeSeconds, ...
            'pose',[c.x,c.y,c.psi],'information',information);
    end
    metadata=struct('fixedLidarDelaySeconds',fixedLidarDelay, ...
        'clock',"INSPVA receiver time bridged from ROS headers; no pose used to fit the clock", ...
        'inputInterpolation',"offline linear reconstruction of recorded input samples", ...
        'reference',"NovAtel odom body-origin XY and quaternion yaw; also mapping reference", ...
        'gpsInput',"all covered recorded odom positions; no GNSS heading measurement", ...
        'lidarInput',"accepted full/directional geometric samples; the continuous adapter rejects insufficient information", ...
        'parameters',parameters,'sampleTimeSeconds',.01,'samples',numel(time), ...
        'maximumReferenceEdgeExtrapolationSeconds',edgeExtrapolation);
end
