function runUncertainHistory()
% runUncertainHistory Propagate uncertain historical rigid transforms into scatter.
% Older clouds are not independent noiseless replicas of the current frame.
% Engineering motion scales are explicit; they are not calibrated covariances.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));out='output/root_cause_matching_20260929';
    b=load('output/pole_boundary_recovery_20260929/replay.mat','currentClouds');currentClouds=b.currentClouds;
    g=load(fullfile(out,'geometricHistory_sources.mat'),'aligned');
    old=load('output/line_direction_matching_20260928/production/report.mat','report');calls=old.report.calls;
    odom=load('output/line_direction_matching_20260928/sources.mat','motion');motion=odom.motion;
    wc=localizationSourceWindowConfig();cfg=distributionRegistrationConfig();R=@(a)[cos(a) -sin(a);sin(a) cos(a)];summaries=table();
    for variant=1:4
        sources=cell(1170,1);
        for k=1:1170
            first=max(1,k-wc.maximumFrames+1);history=[];poses=motion(first:k,:);
            if variant>=2,poses=g.aligned{k};end
            if variant==3,poses(:,3)=motion(first:k,3);end
            if variant==4,poses(:,1:2)=motion(first:k,1:2);end
            for frame=first:k
                cloud=currentClouds{frame};dt=calls.timeSeconds(k)-calls.timeSeconds(frame);
                if dt>0
                    c=registrationSupport.projectSemanticProbabilityCloud(cloud,2);cloud=c;c=cloud.components;
                    J=c.mean*[0 1;-1 0];
                    for j=1:c.numComponents
                        c.covariance(:,:,j)=c.covariance(:,:,j)+(dt*.25)^2*eye(2)+(dt*deg2rad(2))^2*(J(j,:).'*J(j,:));
                    end
                    if isfield(c,'meanXYZ'),c=rmfield(c,{'meanXYZ','covarianceXYZ','heightAvailable'});end
                    cloud.components=c;
                end
                [source,history]=updateLocalizationSourceWindow(cloud,calls.timeSeconds(frame),poses(frame-first+1,:),history,wc);
            end
            sources{k}=source;
        end
        label="uncertainHistory"+variant;file=fullfile(out,label+"_sources.mat");save(file,'sources','currentClouds','wc','variant','-v7.3');
        row=rootCauseReplay(file,label,cfg);summaries=[summaries;row]; %#ok<AGROW>
        writetable(summaries,fullfile(dest,'uncertain_history.csv'));
    end
end
