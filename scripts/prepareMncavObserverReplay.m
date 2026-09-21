function [sensorData,reference,metadata] = prepareMncavObserverReplay(sensorFolder,parameterFile,calls,fixedLidarDelay,options)
% prepareMncavObserverReplay Export recorded inputs and physical pose samples.
% This is a data exporter. Its physical timestamp metadata is not an observer
% timing model. reconstructContinuousObserverSignals supplies the separately
% declared offline input reconstruction for the current continuous runner.
% Vx comes exclusively from four-wheel fusion. IMU/steering use offline
% linear reconstruction and independent-drive fixed corrections.
% GNSS/INS yaw is reference data; only GNSS XY enters the measurement channel.
% LiDAR samples carry the full physical [X,Y,psi] information matrix.
% No delivery timestamps or artificial GNSS downsampling are introduced.
    arguments
        sensorFolder (1,1) string
        parameterFile (1,1) string
        calls table = table()
        fixedLidarDelay (1,1) double {mustBeNonnegative} = .15
        options.IncludeOdom (1,1) logical = true
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    folder=fullfile(root,'data','raw','Missisipi','gnss');
    stem="raw_data_2024-06-07-12-09-31_0";
    clock=loadReceiverClock(fullfile(folder,stem+"_inspva.csv"));
    frames=readtable(fullfile(folder,stem+"_front_lidar_points.csv"));
    parameters=jsondecode(fileread(parameterFile));
    bridge=@(stamp) receiverClockTime(clock,stamp);
    start=bridge(frames.stamp_sec(1));
    % This recording starts stationary; initial prediction is declared and
    % marked separately until the first actual wheel packet arrives.
    [high,wheel,wheelMetadata]=prepareWheelMotionInputs(sensorFolder,parameters,clock, ...
        start,bridge(frames.stamp_sec(end)),InitialSpeed=0);
    time=high.time;sensorData=struct('highRate',high,'wheelVelocity',wheel,'clockModelId',string(clock.modelId));
    reference=table();sensorData.gps=struct();edgeExtrapolation=0;
    if options.IncludeOdom
    odom=readtable(fullfile(folder,stem+"_odom.csv"));
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
    end
    sensorData.lidar=struct();
    if ~isempty(calls)
        assert(ismember('clockModelId',calls.Properties.VariableNames) && ...
            all(string(calls.clockModelId)==string(clock.modelId)), ...
            'VehicleLocalization:ClockMismatch','LiDAR calls must declare the shared receiver clock.');
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
        'clock',clock, ...
        'inputInterpolation',"native wheel fusion; offline linear IMU/steering reconstruction", ...
        'longitudinalVelocity',wheelMetadata, ...
        'reference',"NovAtel odom body-origin XY and quaternion yaw; also mapping reference", ...
        'gpsInput',"all covered recorded odom positions; no GNSS heading measurement", ...
        'lidarInput',"accepted full/directional geometric samples; the continuous adapter rejects insufficient information", ...
        'parameters',parameters,'sampleTimeSeconds',.01,'samples',numel(time), ...
        'maximumReferenceEdgeExtrapolationSeconds',edgeExtrapolation);
    if ~options.IncludeOdom
        metadata.reference="No reference returned; ODOM not read";
        metadata.gpsInput="No GNSS channel exported; caller supplies BESTPOS";
    end
end
