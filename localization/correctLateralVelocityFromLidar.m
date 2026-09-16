function [corrected,audit]=correctLateralVelocityFromLidar(data,lateral,poseTime,options)
% correctLateralVelocityFromLidar Estimate a slow mapped-frame velocity bias.
% At each accepted LiDAR endpoint, compare its past-window displacement with
% integrated measured speed and the uncorrected lateral observer output.
% Solve the two-dimensional constant body-bias equation, but apply only its
% lateral component through a bounded first-order filter. No reference, new
% physical extrinsic calibration, or corrected-state feedback is used.
% Endpoint updates use only past accepted measurements. The supplied motion
% and LiDAR interpolation may themselves have been prepared offline.
% This adapter does not extend the lateral observer's physical CG certificate.
    arguments
        data (1,1) struct
        lateral (1,1) struct
        poseTime (:,1) double {mustBeFinite}
        options.WindowSeconds (1,1) double {mustBePositive,mustBeFinite}=2
        options.TimeConstant (1,1) double {mustBePositive,mustBeFinite}=4
        options.MaximumGap (1,1) double {mustBePositive,mustBeFinite}=.25
        options.MaximumBias (1,1) double {mustBePositive,mustBeFinite}=.8
        options.MinimumSpeed (1,1) double {mustBePositive,mustBeFinite}=5
    end
    assert(options.MinimumSpeed>1,'VehicleLocalization:InvalidVelocityBiasInput', ...
        'The full-participation speed must exceed the 1 m/s fade threshold.');
    h=data.highRate;t=h.time(:);n=numel(t);p=data.lidar.pose;
    assert(n>=2 && all(isfinite(t)) && all(diff(t)>0) && isequal(lateral.time(:),t) ...
        && isequal(size(p),[n,3]) && all(isfinite(p),'all'), ...
        'VehicleLocalization:InvalidVelocityBiasInput','Require aligned finite motion and poses.');
    vx=h.longitudinalSpeed(:);vy=lateral.lateralVelocity(:);
    assert(numel(vx)==n && numel(vy)==n && all(isfinite([vx;vy])) ...
        && numel(lateral.sideSlipAngle)==n && numel(lateral.sideSlipAngleRate)==n ...
        && all(isfinite([lateral.sideSlipAngle(:);lateral.sideSlipAngleRate(:)])), ...
        'VehicleLocalization:InvalidVelocityBiasInput','Require aligned finite lateral interface signals.');
    [present,anchors]=ismember(poseTime,t);
    assert(numel(anchors)>=2 && all(present) && all(diff(anchors)>0), ...
        'VehicleLocalization:InvalidVelocityBiasAnchors','Accepted timestamps must belong to the integration grid.');
    assert(isequal(data.lidar.delay,0) && string(data.lidar.headingConvention)=="unwrapped", ...
        'VehicleLocalization:InvalidVelocityBiasInput','Require declared zero delay and lifted heading.');
    c=cos(p(:,3));s=sin(p(:,3));
    integral=cumtrapz(t,[c.*vx-s.*vy,s.*vx+c.*vy,c,s]);
    bias=zeros(n,1);target=zeros(n,1);candidate=nan(numel(anchors),2);
    used=false(numel(anchors),1);updateIndex=2;left=1;lastTarget=0;
    for k=2:n
        dt=t(k)-t(k-1);
        bias(k)=lastTarget+(bias(k-1)-lastTarget)*exp(-dt/options.TimeConstant);
        if updateIndex<=numel(anchors) && k==anchors(updateIndex)
            while left+1<updateIndex && poseTime(left+1)<=t(k)-options.WindowSeconds
                left=left+1;
            end
            a=anchors(left);span=t(k)-t(a);
            covered=span>=options.WindowSeconds && span<=options.WindowSeconds+options.MaximumGap ...
                && all(diff(poseTime(left:updateIndex))<=options.MaximumGap) ...
                && min(vx(a:k))>=options.MinimumSpeed;
            if covered
                q=integral(k,:)-integral(a,:);B=[q(3),-q(4);q(4),q(3)];
                if rcond(B)>.5 && norm(B,'fro')>span
                    candidate(updateIndex,:)=(B\(p(k,1:2)-p(a,1:2)-q(1:2)).').';
                    % Reject inconsistent windows; never clip an outlier into
                    % an accepted pseudo-measurement at the configured bound.
                    if all(abs(candidate(updateIndex,:))<=options.MaximumBias)
                        lastTarget=candidate(updateIndex,2);used(updateIndex)=true;
                    end
                end
            end
            updateIndex=updateIndex+1;
        end
        target(k)=lastTarget;
    end
    x=min(1,max(0,(abs(vx)-1)/max(options.MinimumSpeed-1,eps)));
    participation=x.^3.*(10-15*x+6*x.^2);
    applied=bias.*participation;
    corrected=lateral;corrected.lateralVelocity=vy+applied;
    extra=atan2(vy+applied,max(vx,1))-atan2(vy,max(vx,1));
    corrected.sideSlipAngle=lateral.sideSlipAngle(:)+extra;
    corrected.sideSlipAngleRate=lateral.sideSlipAngleRate(:)+[0;diff(extra)./diff(t)];
    corrected.lidarVelocityBias=applied;
    audit=struct('options',options,'acceptedWindows',nnz(used),'candidateWindows',numel(anchors), ...
        'referenceUsed',false,'physicalCgCalibrationClaimed',false,'maximumAppliedBias',max(abs(applied)), ...
        'updateClock',"Current and past accepted endpoints; supplied interpolation may be offline", ...
        'stateMeaning',"Effective mapped-reference lateral velocity; original physical lateral observer states are retained", ...
        'time',t,'bias',bias,'target',target,'windowTime',poseTime,'candidateBodyBias',candidate,'used',used);
end
