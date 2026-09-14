function [data,lateralInput,metadata]=reconstructFrameAlignedLidarSignals(high,lateral, ...
        frameTime,poseTime,pose,information,cfg,options)
% reconstructFrameAlignedLidarSignals Align precomputed poses with zero delay.
% Every frame timestamp becomes an integration knot. Only accepted poses
% supply measurement knots; rejected frame times remain evaluation times.
% Between accepted poses, the existing continuous observer uses an explicitly
% offline linear reconstruction. Precomputation removes processing latency,
% not the distinction between a measurement and an interpolated value.
    arguments
        high (1,1) struct
        lateral (1,1) struct
        frameTime (:,1) double {mustBeFinite}
        poseTime (:,1) double {mustBeFinite}
        pose double {mustBeFinite}
        information double
        cfg (1,1) struct
        options.MaximumOfflineGap (1,1) double {mustBePositive,mustBeFinite}=1
        options.MaximumMotionEdgeHold (1,1) double {mustBeNonnegative,mustBeFinite}=.02
    end
    assert(cfg.mode=="lidar" && cfg.measurement.fixedLidarDelay==0, ...
        'VehicleLocalization:ZeroDelayRequired','Frame-aligned precomputed replay requires zero LiDAR delay.');
    assert(numel(frameTime)>=2 && all(diff(frameTime)>0) && numel(high.time)>=2 ...
        && all(diff(high.time)>0),'VehicleLocalization:InvalidFrameTimes','Input and frame clocks must increase.');
    assert(isequal(lateral.time(:),high.time(:)), ...
        'VehicleLocalization:InvalidLateralInputs','Lateral outputs must cover the original motion grid.');
    assert(all(ismember(poseTime,frameTime)), ...
        'VehicleLocalization:InvalidFrameTimes','Every accepted pose must identify a processed frame.');
    before=max(0,high.time(1)-frameTime(1));after=max(0,frameTime(end)-high.time(end));
    assert(max(before,after)<=options.MaximumMotionEdgeHold+1e-12, ...
        'VehicleLocalization:MotionCoverage','The frame clock exceeds the declared motion edge-hold limit.');
    originalTime=high.time(:);
    time=unique([originalTime(originalTime>=frameTime(1) & originalTime<=frameTime(end));frameTime]);
    query=min(max(time,originalTime(1)),originalTime(end));
    for name=string(fieldnames(high)).'
        if name=="time",continue;end
        assert(isvector(high.(name)) && numel(high.(name))==numel(originalTime) ...
            && all(isfinite(high.(name))),'VehicleLocalization:InvalidInputs','Motion fields must be finite and aligned.');
        high.(name)=interp1(originalTime,high.(name)(:),query,'linear');
    end
    high.time=time;lateralInput=struct('time',time);
    for name=["lateralVelocity","sideSlipAngle","sideSlipAngleRate"]
        assert(isfield(lateral,name) && numel(lateral.(name))==numel(originalTime) ...
            && all(isfinite(lateral.(name))),'VehicleLocalization:InvalidLateralInputs','Lateral fields must be finite and aligned.');
        lateralInput.(name)=interp1(originalTime,lateral.(name)(:),query,'linear');
    end
    [data,metadata]=reconstructContinuousObserverSignals(high,poseTime,pose,information,cfg, ...
        MaximumGap=options.MaximumOfflineGap);
    assert(isequal(data.highRate.time,time),'VehicleLocalization:FramePoseCoverage', ...
        'Accepted measurements must bracket the requested replay; no endpoint pose is fabricated.');
    metadata.frameCount=numel(frameTime);metadata.acceptedPoseCount=numel(poseTime);
    metadata.motionStartHoldSeconds=before;metadata.motionEndHoldSeconds=after;
    metadata.frameTimestampMaximumMismatchSeconds=0;
    metadata.processingLatencyAppliedSeconds=0;
    metadata.frameIndicesInIntegrationGrid=arrayfun(@(u) find(time==u,1),frameTime);
    metadata.rejectedFramePolicy="Retain frame evaluation; no new measurement knot. Reconstruct from adjacent accepted poses offline.";
    metadata.measurementPolicy="Exact accepted pose/information at its capture time; linear reconstruction between accepted frames.";
end
