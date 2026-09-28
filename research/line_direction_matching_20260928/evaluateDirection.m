function evaluateDirection()
% evaluateDirection Compare causal full-route solves on identical raw sources.
    root=setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));addpath('output/line_direction_matching_20260928/prototype');
    out='output/line_direction_matching_20260928';baseline=load('output/mississippi_matching_20260928/recursive/report.mat','report','cfg');
    calls=baseline.report.calls;cfg=baseline.cfg;n=height(calls);mapcfg=featureMapBuildConfig();
    loaded=load(mapcfg.probabilityCloudPath,'cloud');fixed=registrationSupport.projectSemanticProbabilityCloud(loaded.cloud,2);
    poses=readFramePoseTable(fullfile(root,'data',mapcfg.poseMatchCsvPath),1:n);store=matfile(fullfile(root,'data',mapcfg.pointCloudMatPath));
    modes=["baseline","direction_1deg","direction_0p5deg"];states=repmat(calls{1,{'predictedX','predictedY','predictedPsi'}},3,1);
    rows=cell(n*3,14);history=[];sources=cell(n,1);currentClouds=sources;first=0;frames=[];row=0;
    motion=baseline.report.deadReckoning{:,{'x','y','psi'}};
    for k=1:n
        if isempty(frames)||k>=first+numel(frames)
            first=k;frames=store.pointClouds(1,k:min(n,k+49));
        end
        [~,tilt]=poseRowToPlanarPose(poses(k,:));cfg.perception.coarseProbabilityCloud.projectionRotation=tilt;
        current=perceiveCoarseProbabilityCloud(frames(k-first+1),cfg.perception);currentClouds{k}=current;
        [source,history]=updateLocalizationSourceWindow(current,calls.timeSeconds(k),motion(k,:),history,cfg.sourceWindow);sources{k}=source;
        for v=1:3
            predicted=states(v,:);if k>1,predicted=compose(predicted,relative(motion(k-1,:),motion(k,:)));end
            [local,~]=selectLocalProbabilityCloud(fixed,predicted,cfg.registration.localMapRadius);t=tic;
            reg=cfg.registration;
            if v==1,r=registerSemanticProbabilityCloud(local,source,predicted,reg);
            else
                reg.lineDirection=struct('radius',4,'minimumComponents',3,'minimumAnisotropy',9,'minimumSpan',2.4,'standardDeviation',deg2rad(1/(v-1)));
                r=experimentalDirectionRegistration(local,source,predicted,reg);
            end
            ms=1000*toc(t);event=registrationSupport.registrationPoseMeasurement(r,calls.timeSeconds(k));
            states(v,:)=predicted;if ~isempty(event),states(v,:)=event.pose;end
            d=states(v,:)-calls{k,{'referenceX','referenceY','referencePsi'}};
            row=row+1;rows(row,:)={k,modes(v),r.accepted,r.directionalAccepted,r.reason,norm(d(1:2)),rad2deg(wrap(d(3))), ...
                states(v,1),states(v,2),states(v,3),ms,r.similarity,r.observableRank,max(abs(states(v,:)-calls{k,{'x','y','psi'}}))};
        end
        if mod(k,100)==0||k==n
            results=cell2table(rows(1:row,:),VariableNames={'frame','mode','accepted','directional','reason','errorM','yawErrorDeg','x','y','psi','matchingMs','similarity','rank','baselineDifference'});
            writetable(results,fullfile(dest,'full_route.csv'));fprintf('Direction evaluation %d/%d\n',k,n);
        end
    end
    assert(max(results.baselineDifference(results.mode=="baseline"))<1e-7,'Baseline reproduction failed.');
    save(fullfile(out,'sources.mat'),'sources','currentClouds','fixed','motion','calls','cfg','-v7.3');
    save(fullfile(out,'evaluation.mat'),'results');disp(groupsummary(results,'mode','mean',{'errorM','matchingMs'}));
end
function p=compose(a,b)
    r=[cos(a(3)) -sin(a(3));sin(a(3)) cos(a(3))];p=[a(1:2)+b(1:2)*r.' wrap(a(3)+b(3))];
end
function b=relative(a,c)
    r=[cos(a(3)) -sin(a(3));sin(a(3)) cos(a(3))];b=[(c(1:2)-a(1:2))*r wrap(c(3)-a(3))];
end
function x=wrap(x)
    x=atan2(sin(x),cos(x));
end
