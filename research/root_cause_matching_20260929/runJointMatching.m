function runJointMatching()
% runJointMatching Diagnostic causal trajectory/map optimization, no GNSS aid.
% This is explicitly a wheel/gyro-and-map fused result, not a raw independent
% LiDAR measurement; each current acquisition contributes at most one factor.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));out='output/root_cause_matching_20260929';
    old=load('output/line_direction_matching_20260928/production/report.mat','report');calls=old.report.calls;odom=load('output/line_direction_matching_20260928/sources.mat','motion');motion=odom.motion;
    mc=featureMapBuildConfig();m=load(mc.probabilityCloudPath,'cloud');cfg=robustPoseGraphConfig();cfg.initialStandardDeviation=[5 5 deg2rad(30)];
    b=load('output/pole_boundary_recovery_20260929/replay.mat','currentClouds');history=[];wc=localizationSourceWindowConfig();current=cell(1170,1);
    for k=1:1170,[~,history,~,current{k}]=updateLocalizationSourceWindow(b.currentClouds{k},calls.timeSeconds(k),motion(k,:),history,wc);end
    for variant=1:2
        if variant==2,cfg.lidarInformationScale=10;end
        state=[];pose=calls{1,{'predictedX','predictedY','predictedPsi'}};rows=cell(1170,9);
        for k=1:1170
            delta=zeros(1,3);
            if k>1
                R=rotation(motion(k-1,3));delta=[(motion(k,1:2)-motion(k-1,1:2))*R,wrap(motion(k,3)-motion(k-1,3))];
                pose=[pose(1:2)+delta(1:2)*rotation(pose(3)).',wrap(pose(3)+delta(3))];
            end
            packet=struct('time',calls.timeSeconds(k),'seed',pose,'relativeMotion',delta, ...
                'motionCovariance',diag(cfg.motionStandardDeviation.^2),'source',current{k},'positionAid',[]);
            timer=tic;[r,state]=updateRobustPoseGraph(state,packet,m.cloud,cfg);ms=1000*toc(timer);pose=r.poseXYTheta;
            ref=calls{k,{'referenceX','referenceY','referencePsi'}};rows(k,:)={k,norm(pose(1:2)-ref(1:2)),rad2deg(wrap(pose(3)-ref(3))),pose(1),pose(2),pose(3),ms,r.converged,r.containsGnss};
            if mod(k,200)==0,fprintf('Joint %d frame %d\n',variant,k);end
        end
        replay=cell2table(rows,VariableNames={'frame','errorM','yawErrorDeg','x','y','psi','matchingMs','converged','containsGnss'});label="jointMatching"+variant;
        writetable(replay,fullfile(dest,label+".csv"));save(fullfile(out,label+".mat"),'replay','cfg');
        [peak,i]=max(replay.errorM(2:end));summary=table(label,i+1,peak,rms(replay.errorM),prctile(replay.errorM,95),median(replay.matchingMs),VariableNames={'variant','maximumFrame','maximumErrorM','rmseM','p95M','medianMs'});disp(summary);writetable(summary,fullfile(dest,label+"_summary.csv"));
    end
end
function R=rotation(x)
    R=[cos(x) -sin(x);sin(x) cos(x)];
end
function x=wrap(x)
    x=atan2(sin(x),cos(x));
end
