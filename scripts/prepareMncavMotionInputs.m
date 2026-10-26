function [sensorData,metadata] = prepareMncavMotionInputs(sensorFolder,parameterFile)
% prepareMncavMotionInputs Export wheel, IMU and steering on the receiver clock.
% Vx comes exclusively from four-wheel fusion. IMU/steering use offline linear
% reconstruction and independent-drive fixed corrections. GNSS measurements
% and overlap-gradient source clouds are prepared by their own input stages.
    arguments
        sensorFolder (1,1) string
        parameterFile (1,1) string = ""
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    folder=fullfile(root,'data','raw','Missisipi','gnss');
    stem="raw_data_2024-06-07-12-09-31_0";
    clock=loadReceiverClock(fullfile(folder,stem+"_inspva.csv"));
    frames=readtable(fullfile(folder,stem+"_front_lidar_points.csv"));
    parameters=mncavReplayConfig(parameterFile);
    start=receiverClockTime(clock,frames.stamp_sec(1));
    finish=receiverClockTime(clock,frames.stamp_sec(end));
    % The recording starts stationary, before the first actual wheel packet.
    [high,wheel,wheelMetadata]=prepareWheelMotionInputs(sensorFolder,parameters,clock, ...
        start,finish,InitialSpeed=0);
    sensorData=struct('highRate',high,'wheelVelocity',wheel,'clockModelId',string(clock.modelId));
    metadata=struct('clock',clock, ...
        'inputInterpolation',"native wheel fusion; offline linear IMU/steering reconstruction", ...
        'longitudinalVelocity',wheelMetadata, ...
        'parameters',parameters,'sampleTimeSeconds',.01,'samples',numel(high.time));
end
