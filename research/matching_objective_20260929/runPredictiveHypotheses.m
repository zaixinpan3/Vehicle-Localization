function runPredictiveHypotheses()
% runPredictiveHypotheses Select independently solved geometric hypotheses.
% Wheel/gyro prediction ranks poses; it adds no force to their LiDAR optima.
% These conditionally selected outputs are not independent of motion aiding.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));out='output/matching_objective_20260929';
    old=load('output/line_direction_matching_20260928/production/report.mat','report');calls=old.report.calls;
    o=load('output/line_direction_matching_20260928/sources.mat','motion');motion=o.motion;
    data=load('output/root_cause_matching_20260929/finalSurface_sources.mat','currentClouds','sources');
    m=load('output/mississippi_mapping_calibrated/probability_cloud.mat','cloud');fixed=registrationSupport.projectSemanticProbabilityCloud(m.cloud,2);
    history=[];confirmed=cell(1170,1);wc=localizationSourceWindowConfig();
    for k=1:1170,[~,history,~,confirmed{k}]=updateLocalizationSourceWindow(data.currentClouds{k},calls.timeSeconds(k),motion(k,:),history,wc);end
    for count=[3 4]
        state=calls{1,{'predictedX','predictedY','predictedPsi'}};rows=cell(1170,12);maximum=[];cfg=distributionRegistrationConfig();
        for k=1:1170
            predicted=state;if k>1,predicted=compose(state,relative(motion(k-1,:),motion(k,:)));end
            local=selectLocalProbabilityCloud(fixed,predicted,cfg.localMapRadius);timer=tic;result=cell(count,1);value=inf(count,1);
            for j=1:count
                reg=cfg;source=data.sources{k};
                if j>=2,reg.pyramid.trustRadius=1e6;end
                if j==3,reg=rmfield(reg,'softPointAssociation');end
                if j==4,source=confirmed{k};end
                result{j}=registerSemanticProbabilityCloud(local,source,predicted,reg);
                if result{j}.accepted
                    delta=result{j}.poseXYTheta-predicted;delta(3)=wrap(delta(3));
                    value(j)=sum((delta./[.15 .15 deg2rad(1)]).^2);
                end
            end
            [v,chosen]=min(value);if ~isfinite(v),chosen=1;end;r=result{chosen};ms=1000*toc(timer);
            event=registrationSupport.registrationPoseMeasurement(r,calls.timeSeconds(k));state=predicted;if ~isempty(event),state=event.pose;end
            ref=calls{k,{'referenceX','referenceY','referencePsi'}};e=state-ref;
            rows(k,:)={k,r.accepted,r.directionalAccepted,r.reason,norm(e(1:2)),rad2deg(wrap(e(3))),state(1),state(2),state(3),ms,chosen,nnz(isfinite(value))};
            if k>1&&(isempty(maximum)||norm(e(1:2))>maximum.errorM),maximum=struct('frame',k,'errorM',norm(e(1:2)),'result',r,'allResults',{result},'predicted',predicted,'reference',ref);end
        end
        label="predictiveHypotheses"+count;replay=cell2table(rows,VariableNames={'frame','accepted','directional','reason','errorM','yawErrorDeg','x','y','psi','matchingMs','selectedHypothesis','validHypotheses'});
        writetable(replay,fullfile(dest,label+".csv"));save(fullfile(out,label+".mat"),'replay','maximum','cfg','-v7.3');
        summary=table(label,maximum.frame,maximum.errorM,rms(replay.errorM),prctile(replay.errorM,95),nnz(replay.accepted),nnz(replay.directional),median(replay.matchingMs), ...
            VariableNames={'variant','maximumFrame','maximumErrorM','rmseM','p95M','full','directional','medianMs'});writetable(summary,fullfile(dest,label+"_summary.csv"));disp(summary);
    end
end
function p=compose(a,b)
    R=[cos(a(3)) -sin(a(3));sin(a(3)) cos(a(3))];p=[a(1:2)+b(1:2)*R.' wrap(a(3)+b(3))];
end
function b=relative(a,c)
    R=[cos(a(3)) -sin(a(3));sin(a(3)) cos(a(3))];b=[(c(1:2)-a(1:2))*R wrap(c(3)-a(3))];
end
function a=wrap(a)
    a=atan2(sin(a),cos(a));
end
