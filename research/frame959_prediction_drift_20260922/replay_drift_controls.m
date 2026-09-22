function summary=replay_drift_controls()
% replay_drift_controls Isolate prediction and association over frames 950:959.
% Oracle motion and map-target exclusions are diagnostic, never production.
    root=setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    saved=load('output/temporal_perception_20260922/five_frame_matching/report.mat','report','cfg');
    cfg=saved.cfg;c=saved.report.calls;d=saved.report.deadReckoning;
    map=load('output/mississippi_mapping_calibrated/probability_cloud.mat','cloud');
    full=registrationSupport.projectSemanticProbabilityCloud(map.cloud,2);
    variants=["recorded_motion","reference_lateral_increment","reference_full_increment","exclude_sign_target1300"];
    state=repmat([c.x(949) c.y(949) c.psi(949)],numel(variants),1);
    frames=945:959;mapCfg=featureMapBuildConfig();h=[];
    poses=readFramePoseTable(fullfile(root,'data',mapCfg.poseMatchCsvPath),frames);
    rows=cell(0,11);signRows=cell(0,9);reproduction=0;
    for j=1:numel(frames)
        k=frames(j);[~,tilt]=poseRowToPlanarPose(poses(j,:));cfg.perception.coarseProbabilityCloud.projectionRotation=tilt;
        raw=loadPointCloudFrame(fullfile(root,'data',mapCfg.pointCloudMatPath),k);
        current=perceiveCoarseProbabilityCloud(raw,cfg.perception);
        [source,h]=updateLocalizationSourceWindow(current,c.timeSeconds(k),[d.x(k) d.y(k) d.psi(k)],h,cfg.sourceWindow);
        if k<950,continue;end
        odometry=relative([d.x(k-1) d.y(k-1) d.psi(k-1)],[d.x(k) d.y(k) d.psi(k)]);
        reference=[c.referenceX(k) c.referenceY(k) c.referencePsi(k)];
        referenceStep=relative([c.referenceX(k-1) c.referenceY(k-1) c.referencePsi(k-1)],reference);
        for v=1:numel(variants)
            step=odometry;
            if v==2,step(2)=referenceStep(2);elseif v==3,step=referenceStep;end
            predicted=compose(state(v,:),step);
            keep=vecnorm(full.components.mean-predicted(1:2),2,2)<=100;
            if v==4,keep(1300)=false;end
            globalTargets=find(keep);fixed=subset(full,keep);
            result=registerSemanticProbabilityCloud(fixed,source,predicted,cfg.registration);
            state(v,:)=predicted;
            if result.accepted||result.directionalAccepted,state(v,:)=result.poseXYTheta;end
            rows(end+1,:)={variants(v),k,norm(predicted(1:2)-reference(1:2)), ...
                norm(state(v,1:2)-reference(1:2)),rad2deg(wrap(state(v,3)-reference(3))), ...
                result.accepted,result.directionalAccepted,result.reason,state(v,1),state(v,2),state(v,3)}; %#ok<AGROW>
            if v==1
                reproduction=max(reproduction,max(abs(state(v,:)-[c.x(k) c.y(k) c.psi(k)])));
                assert(reproduction<1e-7);
            end
            if isfield(result,'correspondences')
                p=result.correspondences;
                for z=find(p.semanticName=="trafficSign").'
                    xy=source.components.mean(p.source(z),:);
                    target=(fixed.components.mean(p.target(z),:)-reference(1:2))*rotation(reference(3));
                    signRows(end+1,:)={variants(v),k,p.source(z),globalTargets(p.target(z)), ...
                        source.components.detectionFrameCount(p.source(z)),norm(xy-target), ...
                        p.weight(z),p.robustWeight(z),p.squaredStandardizedResidual(z)}; %#ok<AGROW>
                end
            end
        end
    end
    controls=cell2table(rows,VariableNames={'variant','frame','predictedErrorM','matchedErrorM','yawErrorDeg', ...
        'accepted','directionalAccepted','reason','x','y','psi'});
    signs=cell2table(signRows,VariableNames={'variant','frame','source','globalTarget','detections', ...
        'referenceDistanceM','weight','robustWeight','squaredStandardizedResidual'});
    writetable(controls,fullfile(dest,'controls.csv'));writetable(signs,fullfile(dest,'sign_associations.csv'));
    summary=struct('firstStateFrame',949,'queryFrames',950:959,'warmupFrames',945:949, ...
        'variants',variants,'maximumReproductionDifference',reproduction, ...
        'sourceHistoryMotionUnchanged',true,'productionChanged',false);
    save('output/frame959_prediction_drift_20260922/replay.mat','controls','signs','summary');
    fid=fopen(fullfile(dest,'replay_validation.json'),'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(summary,PrettyPrint=true));disp(controls(controls.frame==959,:));disp(signs(signs.variant=="recorded_motion",:));
end
function cloud=subset(cloud,keep)
    c=cloud.components;c.mean=c.mean(keep,:);c.covariance=c.covariance(:,:,keep);
    for field=["semanticName","mixtureWeight","repeatability","semanticProbability","occupancyProbability","supportAmplitude"]
        if isfield(c,field),c.(field)=c.(field)(keep,:);end
    end
    c.numComponents=nnz(keep);cloud.components=c;
end
function r=rotation(a)
    r=[cos(a) -sin(a);sin(a) cos(a)];
end
function result=relative(a,b)
    result=[(b(1:2)-a(1:2))*rotation(a(3)),wrap(b(3)-a(3))];
end
function result=compose(a,b)
    result=[a(1:2)+b(1:2)*rotation(a(3)).',wrap(a(3)+b(3))];
end
function value=wrap(value)
    value=atan2(sin(value),cos(value));
end
