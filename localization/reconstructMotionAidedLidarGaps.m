function [data,metadata]=reconstructMotionAidedLidarGaps(data,lateral,poseTime,options)
% reconstructMotionAidedLidarGaps Reconstruct long offline gaps using motion.
% Integrate measured yaw rate, then distribute the two LiDAR endpoint yaw
% discrepancy over the interval. Integrate measured longitudinal speed and
% estimated lateral velocity in that orientation, and distribute the endpoint
% position discrepancy likewise. Original accepted poses remain exact.
% These generated knots are motion-informed interpolation, not new LiDAR
% measurements. The right endpoint is required: this is explicitly offline.
% Geometric information is unchanged; it is not a new calibrated covariance.
    arguments
        data (1,1) struct
        lateral (1,1) struct
        poseTime (:,1) double {mustBeFinite}
        options.MinimumGap (1,1) double {mustBePositive,mustBeFinite}=.25
    end
    h=data.highRate;s=data.lidar;t=h.time(:);n=numel(t);
    assert(n>=2 && all(isfinite(t)) && all(diff(t)>0) && isequal(s.delay,0) ...
        && string(s.representation)=="piecewiseLinear" && string(s.headingConvention)=="unwrapped" ...
        && isequal(s.time(:),t) && isequal(lateral.time(:),t), ...
        'VehicleLocalization:InvalidMotionGapInput','Require aligned, lifted, zero-delay continuous inputs.');
    assert(isequal(size(s.pose),[n,3]) && all(isfinite(s.pose),'all'), ...
        'VehicleLocalization:InvalidMotionGapInput','Require a finite aligned pose reconstruction.');
    for name=["longitudinalSpeed","yawRate"]
        assert(isvector(h.(name)) && numel(h.(name))==n && all(isfinite(h.(name))), ...
            'VehicleLocalization:InvalidMotionGapInput','Motion samples must be finite and aligned.');
    end
    assert(isvector(lateral.lateralVelocity) && numel(lateral.lateralVelocity)==n ...
        && all(isfinite(lateral.lateralVelocity)), ...
        'VehicleLocalization:InvalidMotionGapInput','Lateral speed must be finite and aligned.');
    [present,anchors]=ismember(poseTime,t);
    assert(numel(anchors)>=2 && all(present) && all(diff(anchors)>0) ...
        && anchors(1)==1 && anchors(end)==n, ...
        'VehicleLocalization:InvalidMotionGapAnchors','Accepted timestamps must bracket and belong to the integration grid.');
    original=s.pose;selected=find(diff(poseTime)>options.MinimumGap);corrected=0;
    speed=h.longitudinalSpeed(:);gyro=h.yawRate(:);lateralSpeed=lateral.lateralVelocity(:);
    for j=selected(:).'
        ix=(anchors(j):anchors(j+1)).';tau=t(ix)-t(ix(1));fraction=tau/tau(end);
        yaw=original(ix(1),3)+cumtrapz(tau,gyro(ix));
        yaw=yaw+fraction*(original(ix(end),3)-yaw(end));
        vx=speed(ix);vy=lateralSpeed(ix);
        velocity=[cos(yaw).*vx-sin(yaw).*vy,sin(yaw).*vx+cos(yaw).*vy];
        displacement=cumtrapz(tau,velocity);
        endpointResidual=original(ix(end),1:2)-original(ix(1),1:2)-displacement(end,:);
        position=original(ix(1),1:2)+displacement+fraction*endpointResidual;
        interior=ix(2:end-1);
        data.lidar.pose(interior,:)=[position(2:end-1,:),yaw(2:end-1)];
        corrected=corrected+numel(interior);
    end
    mismatch=max(abs(data.lidar.pose(anchors,:)-original(anchors,:)),[],'all');
    assert(mismatch==0,'VehicleLocalization:MotionGapAnchorMismatch','Accepted LiDAR poses must remain exact.');
    metadata=struct('method',"Endpoint-constrained integral of measured motion in long LiDAR gaps", ...
        'minimumGapSeconds',options.MinimumGap,'correctedIntervals',numel(selected), ...
        'correctedInteriorKnots',corrected,'maximumAcceptedPoseMismatch',mismatch, ...
        'futureEndpointUsed',true,'onlineCausalityClaimed',false,'processingDelaySeconds',0, ...
        'referenceUsed',false,'informationUnchanged',isequaln(data.lidar.information,s.information), ...
        'motionInputs',"Measured longitudinal speed and gyro; estimated lateral velocity", ...
        'interpretation',"Offline interpolation, not independent or additional LiDAR measurements; no covariance calibration claimed");
    data.lidar.offlineMotionGapReconstruction=metadata;
end
