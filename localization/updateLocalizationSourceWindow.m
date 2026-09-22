function [cloud,history,details]=updateLocalizationSourceWindow(current,timestamp,motionPose,history,cfg)
% updateLocalizationSourceWindow Confirm and merge recent coarse XY distributions.
% motionPose is a cumulative planar wheel/gyro pose in any fixed odometry
% frame. Only relative motion is used. Store Gaussian statistics, not points.
% Same-class distributions are associated one-to-one per scan, then merged
% by equal-scan mixture moments. Only repeated tracks enter matching. A track
% receives one vote per acquisition, never per point or neighboring pillar.
% Stability is detection count / configured horizon, with no startup exemption.
% Covariance is spatial scatter, not covariance of an independent sample mean.
% The pooled product is XY only: vertical ego motion is not supplied here.
    if nargin<4 || isempty(history),history=struct('clouds',{{}},'time',zeros(0,1),'motion',zeros(0,3));end
    if nargin<5,cfg=localizationSourceWindowConfig();end
    assert(isscalar(timestamp)&&isfinite(timestamp)&&isreal(timestamp), ...
        'VehicleLocalization:InvalidWindowTime','Require a finite acquisition timestamp.');
    assert(numel(motionPose)==3&&isreal(motionPose)&&all(isfinite(motionPose)), ...
        'VehicleLocalization:InvalidWindowMotion','Require cumulative planar odometry.');
    assert(cfg.minimumDetectionFrames>=2 && cfg.minimumDetectionFrames==fix(cfg.minimumDetectionFrames) && ...
        isfinite(cfg.minimumDetectionFrames) && cfg.maximumFrames>=cfg.minimumDetectionFrames && ...
        cfg.maximumFrames==fix(cfg.maximumFrames) && isfinite(cfg.maximumFrames) && ...
        isfinite(cfg.maximumAgeSeconds) && cfg.maximumAgeSeconds>=0 && ...
        isfinite(cfg.maximumAssociationDistance) && cfg.maximumAssociationDistance>0 && ...
        isfinite(cfg.maximumStandardizedDistance) && cfg.maximumStandardizedDistance>0 && ...
        isfinite(cfg.associationNoiseStandardDeviation) && cfg.associationNoiseStandardDeviation>0, ...
        'VehicleLocalization:InvalidWindowConfiguration','Invalid source-window bounds.');
    assert(isempty(history.time)||timestamp>history.time(end), ...
        'VehicleLocalization:NonmonotonicWindowTime','Reset the source window before rewinding time.');
    mappingSupport.validateSemanticProbabilityCloud(current);
    assert(~isfield(current.components,'temporalStability'), ...
        'VehicleLocalization:AlreadyStackedSource','Supply a fresh single-scan cloud, not a previous horizon product.');
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
    [c,statistics]=stableTracks(sets,cfg);
    cloud.components=c;
    details=struct('frameCount',numel(history.time),'oldestTimestamp',history.time(1), ...
        'newestTimestamp',timestamp,'spanSeconds',timestamp-history.time(1), ...
        'componentCount',c.numComponents,'currentComponentCount',current.components.numComponents, ...
        'unfilteredComponentCount',statistics.inputComponents, ...
        'trackCount',statistics.tracks,'rejectedSingletons',statistics.singletons, ...
        'rejectedTracks',statistics.rejected,'minimumDetectionFrames',cfg.minimumDetectionFrames, ...
        'minimumSupport',statistics.minimumSupport,'meanStability',statistics.meanStability, ...
        'motionSource',"separately supplied cumulative wheel/gyro odometry", ...
        'informationCalibrated',false,'dimension',2);
end

function value=fieldOrUnit(c,name)
    value=ones(c.numComponents,1);
    if isfield(c,name),value=c.(name)(:);end
end

function [c,statistics]=stableTracks(sets,cfg)
% Associate temporally, with no map, ring, reference pose or raw-point access.
    total=sum(cellfun(@(s)size(s.mean,1),sets));horizon=numel(sets);
    meanXY=zeros(total,2);scatter=zeros(2,2,total);names=strings(total,1);
    count=zeros(total,1);semantic=zeros(total,1);occupancy=zeros(total,1);
    support=false(total,horizon);tracks=0;
    for frame=1:horizon
        observation=sets{frame};n=size(observation.mean,1);assignment=zeros(n,1);
        % Compare one semantic class at a time. A track and an observation
        % can each occur in only one selected pair for this acquisition.
        for name=unique(observation.semanticName).'
            a=find(names(1:tracks)==name);b=find(observation.semanticName==name & observation.mixtureWeight>0);
            if isempty(a)||isempty(b),continue;end
            dx=meanXY(a,1)-observation.mean(b,1).';dy=meanXY(a,2)-observation.mean(b,2).';
            ca=scatter(:,:,a);cb=observation.covariance(:,:,b);
            xx=reshape(ca(1,1,:),[],1)+reshape(cb(1,1,:),1,[])+cfg.associationNoiseStandardDeviation^2;
            xy=reshape(ca(1,2,:),[],1)+reshape(cb(1,2,:),1,[]);
            yy=reshape(ca(2,2,:),[],1)+reshape(cb(2,2,:),1,[])+cfg.associationNoiseStandardDeviation^2;
            distance=(yy.*dx.^2-2*xy.*dx.*dy+xx.*dy.^2)./(xx.*yy-xy.^2);
            eligible=find(dx.^2+dy.^2<=cfg.maximumAssociationDistance^2 & distance<=cfg.maximumStandardizedDistance^2);
            [~,order]=sort(distance(eligible));eligible=eligible(order);
            used=false(numel(a),1);
            for candidate=1:numel(eligible)
                index=eligible(candidate);
                [i,j]=ind2sub(size(distance),index);
                if ~used(i)&&assignment(b(j))==0
                    assignment(b(j))=a(i);used(i)=true;
                end
            end
        end
        for j=find(observation.mixtureWeight>0).'
            id=assignment(j);
            if id==0,tracks=tracks+1;id=tracks;names(id)=observation.semanticName(j);end
            previous=count(id);count(id)=previous+1;fraction=1/count(id);
            delta=observation.mean(j,:)-meanXY(id,:);
            % Within-scan scatter plus between-scan disagreement; never divide
            % the scatter by frame count as though scans were independent.
            scatter(:,:,id)=(1-fraction)*scatter(:,:,id)+fraction*observation.covariance(:,:,j) ...
                +fraction*(1-fraction)*(delta.'*delta);
            meanXY(id,:)=meanXY(id,:)+fraction*delta;
            semantic(id)=semantic(id)+fraction*(observation.semanticProbability(j)-semantic(id));
            occupancy(id)=occupancy(id)+fraction*(observation.occupancyProbability(j)-occupancy(id));
            support(id,frame)=true;
        end
    end
    keep=count>=cfg.minimumDetectionFrames;stability=count(keep)/cfg.maximumFrames;
    mass=semantic(keep).*occupancy(keep).*stability;
    c=struct('mean',meanXY(keep,:),'covariance',scatter(:,:,keep), ...
        'semanticName',names(keep),'mixtureWeight',mass/max(sum(mass),realmin), ...
        'semanticProbability',semantic(keep),'occupancyProbability',occupancy(keep), ...
        'detectionFrameCount',count(keep),'detectionFrameMask',support(keep,:), ...
        'temporalStability',stability,'numComponents',nnz(keep));
    minimumSupport=0;meanStability=0;
    if any(keep),minimumSupport=min(count(keep));meanStability=mean(stability);end
    statistics=struct('inputComponents',total,'tracks',tracks,'singletons',nnz(count==1), ...
        'rejected',tracks-nnz(keep),'minimumSupport',minimumSupport,'meanStability',meanStability);
end
