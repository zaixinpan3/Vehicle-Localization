function runGeometricHistory()
% runGeometricHistory Align each historical acquisition to current geometry.
% The relative-motion seed is independent wheel/gyro odometry. No map/reference
% pose enters scan-to-scan registration. Only supported corrections are used.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));out='output/root_cause_matching_20260929';
    b=load('output/pole_boundary_recovery_20260929/replay.mat','currentClouds');currentClouds=b.currentClouds;
    old=load('output/line_direction_matching_20260928/production/report.mat','report');calls=old.report.calls;
    odom=load('output/line_direction_matching_20260928/sources.mat','motion');motion=odom.motion;
    wc=localizationSourceWindowConfig();cfg=distributionRegistrationConfig();reg=cfg;
    reg.pyramid.mapMergeRadius=.5;reg.pyramid.sourceMergeRadius=.5;reg.pyramid.trustRadius=10;
    reg=rmfield(reg,'softPointAssociation');reg.maximumPoseCorrection=[.4 .4 deg2rad(3)];reg.geometric.maximumMatchDistance=1;
    reg.maximumIterationsPerScale=20;R=@(a)[cos(a) -sin(a);sin(a) cos(a)];
    sources=cell(1170,1);rows=cell(0,9);aligned=cell(1170,1);
    for k=1:1170
        first=max(1,k-wc.maximumFrames+1);history=[];fixed=lineCloud(currentClouds{k},reg);poses=motion(first:k,:);
        for frame=first:k-1
            seed=[(motion(frame,1:2)-motion(k,1:2))*R(motion(k,3)),motion(frame,3)-motion(k,3)];
            result=registerSemanticProbabilityCloud(fixed,currentClouds{frame},seed,reg);pose=seed;
            if result.accepted||result.directionalAccepted,pose=result.poseXYTheta;end
            poses(frame-first+1,:)=[motion(k,1:2)+pose(1:2)*R(motion(k,3)).',motion(k,3)+pose(3)];
            delta=pose-seed;rows(end+1,:)={k,frame,result.accepted,result.directionalAccepted,result.reason,delta(1),delta(2),rad2deg(delta(3)),result.similarity}; %#ok<AGROW>
        end
        aligned{k}=poses;
        for frame=first:k,[source,history]=updateLocalizationSourceWindow(currentClouds{frame},calls.timeSeconds(frame),poses(frame-first+1,:),history,wc);end
        sources{k}=source;
        if mod(k,100)==0,fprintf('Geometric history %d/1170\n',k);end
    end
    file=fullfile(out,'geometricHistory_sources.mat');save(file,'sources','currentClouds','aligned','reg','-v7.3');
    writetable(cell2table(rows,VariableNames={'frame','historyFrame','accepted','directional','reason','dx','dy','yawDeg','similarity'}),fullfile(dest,'history_alignment.csv'));
    rootCauseReplay(file,"geometricHistory",cfg);
end
function cloud=lineCloud(cloud,cfg)
    cloud=registrationSupport.projectSemanticProbabilityCloud(cloud,2);c=cloud.components;
    [tangent,valid]=sourceLineDirections(c,cfg.lineDirection);
    for k=find(valid).'
        t=tangent(k,:).';n=[-t(2);t(1)];variance=max(.03^2,n.'*c.covariance(:,:,k)*n);
        c.covariance(:,:,k)=t*t.'+variance*(n*n.');
    end
    if isfield(c,'meanXYZ'),c=rmfield(c,{'meanXYZ','covarianceXYZ','heightAvailable'});end
    cloud.components=c;
end
