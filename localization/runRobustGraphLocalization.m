function estimate=runRobustGraphLocalization(data,lateral,clouds,map,motionPose,initialPose,cfg,observerCfg)
% runRobustGraphLocalization Run the graph as the pose-fusion backend.
% Wheel/gyro motion is supplied independently of map corrections. Stacked
% clouds initialize candidates only; confirmed current observations supply
% likelihood factors. Graph output is never fed back as independent LiDAR.
    t=data.highRate.time(:);n=numel(t);state=[];pose=zeros(n,3);results=cell(n,1);
    candidates=cell(n,1);conditional=cell(n,1);seconds=zeros(n,1);
    assert(numel(clouds.sources)>=n && numel(clouds.currentSources)>=n && size(motionPose,1)>=n);
    assert(isequal(lateral.time(:),t),'Require aligned lateral input provenance.');
    for k=1:n
        timer=tic;delta=zeros(1,3);seed=initialPose;
        if k>1
            R=rotation(motionPose(k-1,3));
            delta=[(motionPose(k,1:2)-motionPose(k-1,1:2))*R,wrap(motionPose(k,3)-motionPose(k-1,3))];
            seed=pose(k-1,:)+[delta(1:2)*rotation(pose(k-1,3)).',delta(3)];
        end
        aid=struct('valid',false,'timestamp',t(k));
        if isfield(data,'gnss') && data.gnss.valid(k)
            assert(abs(data.gnss.time(k)-t(k))<1e-7,'Position and scan clocks must agree.');
            [p,I]=correctGnssOutputPoint(data.gnss.position(k,:),data.gnss.information(:,:,k),seed(3),observerCfg.gnss.outputPoint);
            aid.position=p;aid.covariance=I\eye(2);aid.valid=true;
        end
        r=matchLocalProbabilityCloud(map,clouds.sources{k},seed,cfg.registration,aid);
        candidates{k}=r;
        if r.accepted||r.directionalAccepted,seed=r.poseXYTheta;end
        packet=struct('time',t(k),'seed',seed,'relativeMotion',delta, ...
            'motionCovariance',diag(cfg.motionStandardDeviation.^2), ...
            'source',clouds.currentSources{k},'positionAid',aid);
        [g,state]=updateRobustPoseGraph(state,packet,map,cfg);
        pose(k,:)=g.poseXYTheta;results{k}=g;
        % A diagnostic LiDAR-only solve from the graph seed distinguishes
        % improved association from merely smoothing a fused trajectory.
        conditional{k}=matchLocalProbabilityCloud(map,clouds.sources{k},pose(k,:),cfg.registration,[]);
        seconds(k)=toc(timer);
        if mod(k,100)==0,fprintf('Graph %s %d/%d\n',cfg.featureLoss,k,n);end
    end
    estimate=struct('time',t,'pose',pose,'results',{results},'initialCandidates',{candidates}, ...
        'conditionalMatching',{conditional},'seconds',seconds,'cfg',cfg, ...
        'measurementType',"fusedGraphTrajectory",'independentLidarMeasurement',false, ...
        'inputMotion',"independent wheel/gyro/lateral odometry", ...
        'marginalizedAcquisitions',state.marginalizedAcquisitions, ...
        'uniqueObservationCount',state.totalObservations);
end
function R=rotation(yaw)
    R=[cos(yaw) -sin(yaw);sin(yaw) cos(yaw)];
end
function x=wrap(x)
    x=atan2(sin(x),cos(x));
end
