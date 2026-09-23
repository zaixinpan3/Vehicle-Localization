function matcher=createRobustGraphMatcher(clouds,map,motionPose,t,cfg)
% createRobustGraphMatcher Use a causal graph only to propose a registration seed.
% The existing observer still fuses conditional LiDAR geometry and GNSS once.
% Graph curvature is never exported as LiDAR information. Selection remains
% correlated with GNSS/history and does not imply independent observations.
    state=[];last=0;
    matcher=@match;
    function result=match(k,seed,aid)
        assert(k==last+1,'VehicleLocalization:GraphCallbackOrder','Use a fresh callback for each ordered replay.');
        delta=zeros(1,3);
        if k>1
            yaw=motionPose(k-1,3);R=[cos(yaw) -sin(yaw);sin(yaw) cos(yaw)];
            delta=[(motionPose(k,1:2)-motionPose(k-1,1:2))*R, ...
                atan2(sin(motionPose(k,3)-yaw),cos(motionPose(k,3)-yaw))];
        end
        timer=tic;
        packet=struct('time',t(k),'seed',seed,'relativeMotion',delta, ...
            'motionCovariance',diag(cfg.motionStandardDeviation.^2), ...
            'source',clouds.currentSources{k},'positionAid',aid);
        [graph,state]=updateRobustPoseGraph(state,packet,map,cfg);
        result=matchLocalProbabilityCloud(map,clouds.sources{k},seed,cfg.registration,aid,graph.poseXYTheta);
        result.graphAiding=struct('pose',graph.poseXYTheta,'converged',graph.converged, ...
            'informationAdded',false,'correspondences',graph.correspondences, ...
            'uniqueObservations',graph.uniqueObservationCount,'totalSeconds',toc(timer));
        last=k;
    end
end
