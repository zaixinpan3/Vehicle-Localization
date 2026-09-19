function [cloud,history,details]=updateLocalizationSourceWindow(current,timestamp,motionPose,history,cfg)
% updateLocalizationSourceWindow Transport recent coarse XY distributions.
% motionPose is a cumulative planar wheel/gyro pose in any fixed odometry
% frame. Only relative motion is used. Store Gaussian statistics, not points.
% Cloud weights are normalized after concatenation. Registration balances
% source evidence within each class, so repeated scans cannot multiply the
% exported information as if they were independent observations.
% The pooled product is XY only: vertical ego motion is not supplied here.
    if nargin<4 || isempty(history),history=struct('clouds',{{}},'time',zeros(0,1),'motion',zeros(0,3));end
    if nargin<5,cfg=localizationSourceWindowConfig();end
    assert(isscalar(timestamp)&&isfinite(timestamp)&&isreal(timestamp), ...
        'VehicleLocalization:InvalidWindowTime','Require a finite acquisition timestamp.');
    assert(numel(motionPose)==3&&isreal(motionPose)&&all(isfinite(motionPose)), ...
        'VehicleLocalization:InvalidWindowMotion','Require cumulative planar odometry.');
    assert(cfg.maximumFrames>=1 && cfg.maximumFrames==fix(cfg.maximumFrames) && ...
        isfinite(cfg.maximumFrames) && isfinite(cfg.maximumAgeSeconds) && cfg.maximumAgeSeconds>=0, ...
        'VehicleLocalization:InvalidWindowConfiguration','Invalid source-window bounds.');
    assert(isempty(history.time)||timestamp>history.time(end), ...
        'VehicleLocalization:NonmonotonicWindowTime','Reset the source window before rewinding time.');
    mappingSupport.validateSemanticProbabilityCloud(current);
    keep=history.time>=timestamp-cfg.maximumAgeSeconds;
    history.clouds=history.clouds(keep);history.time=history.time(keep);history.motion=history.motion(keep,:);
    history.clouds{end+1,1}=registrationSupport.projectSemanticProbabilityCloud(current,2);
    history.time(end+1,1)=timestamp;history.motion(end+1,:)=double(motionPose(:).');
    first=max(1,numel(history.time)-cfg.maximumFrames+1);
    history.clouds=history.clouds(first:end);history.time=history.time(first:end);history.motion=history.motion(first:end,:);
    cloud=history.clouds{end};sets=cell(numel(history.time),1);origin=history.motion(end,:);
    rotation=[cos(origin(3)) -sin(origin(3));sin(origin(3)) cos(origin(3))];
    for k=1:numel(sets)
        registrationSupport.validateRegistrationCalibration(cloud,history.clouds{k});
        c=history.clouds{k}.components;delta=history.motion(k,:)-origin;
        r=[cos(delta(3)) -sin(delta(3));sin(delta(3)) cos(delta(3))];
        c.mean=c.mean*r.'+delta(1:2)*rotation;
        c.covariance=pagemtimes(pagemtimes(r,c.covariance),r.');
        c.semanticProbability=fieldOrUnit(c,'semanticProbability');
        c.occupancyProbability=fieldOrUnit(c,'occupancyProbability');
        sets{k}=struct('mean',c.mean,'covariance',c.covariance,'semanticName',c.semanticName, ...
            'mixtureWeight',c.mixtureWeight,'semanticProbability',c.semanticProbability, ...
            'occupancyProbability',c.occupancyProbability);
    end
    sets=vertcat(sets{:});
    c=struct('mean',vertcat(sets.mean),'covariance',cat(3,sets.covariance), ...
        'semanticName',vertcat(sets.semanticName),'mixtureWeight',vertcat(sets.mixtureWeight), ...
        'semanticProbability',vertcat(sets.semanticProbability), ...
        'occupancyProbability',vertcat(sets.occupancyProbability));
    c.numComponents=size(c.mean,1);
    c.mixtureWeight=c.mixtureWeight/max(sum(c.mixtureWeight),realmin);
    cloud.components=c;
    details=struct('frameCount',numel(history.time),'oldestTimestamp',history.time(1), ...
        'newestTimestamp',timestamp,'spanSeconds',timestamp-history.time(1), ...
        'componentCount',c.numComponents,'currentComponentCount',current.components.numComponents, ...
        'motionSource',"separately supplied cumulative wheel/gyro odometry", ...
        'informationCalibrated',false,'dimension',2);
end

function value=fieldOrUnit(c,name)
    value=ones(c.numComponents,1);
    if isfield(c,name),value=c.(name)(:);end
end
