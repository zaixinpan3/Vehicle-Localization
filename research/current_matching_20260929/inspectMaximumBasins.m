function inspectMaximumBasins()
% inspectMaximumBasins Separate map merging, local minima and window motion.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    out='output/current_matching_20260929';d=load(fullfile(out,'diagnostic.mat'));
    p=load('output/line_direction_matching_20260928/production/report.mat','report');calls=p.report.calls;
    mc=featureMapBuildConfig();
    poses=readFramePoseTable(fullfile(pwd,'data',mc.poseMatchCsvPath),1:1170);
    cfgp=perceptionConfig('Mississippi');
    R=[cos(d.ref(3)) -sin(d.ref(3));sin(d.ref(3)) cos(d.ref(3))];
    rows=cell(0,8);results=cell(0,1);
    for radius=[0 .25 .5 .75 1 1.25 1.5 2]
        cfg=d.cfg;cfg.pyramid.mapMergeRadius=radius;
        r=registerSemanticProbabilityCloud(d.fixed,d.source,d.seed,cfg);results{end+1}=r; %#ok<AGROW>
        rows(end+1,:)=score("map_radius_"+radius,r,d.ref,radius); %#ok<AGROW>
    end
    cfg=d.cfg;cfg.pyramid.mapMergeRadius=0;cfg.pyramid.sourceMergeRadius=0;
    for k=1:3
        seeds=[d.seed;d.ref;d.results{1}.poseXYTheta];labels=["fine_from_prediction","fine_from_reference","fine_from_coarse"];
        r=registerSemanticProbabilityCloud(d.fixed,d.source,seeds(k,:),cfg);results{end+1}=r; %#ok<AGROW>
        rows(end+1,:)=score(labels(k),r,d.ref,0); %#ok<AGROW>
        pairs=r.correspondences;pairs.sourceBody=d.source.components.mean(pairs.source,:);
        pairs.targetReferenceBody=(d.fixed.components.mean(pairs.target,:)-d.ref(1:2))*R;
        pairs.globalTarget=d.ids(pairs.target);
        writetable(pairs,fullfile(dest,labels(k)+"_pairs.csv"));
    end
    history=[];
    for frame=d.summary.frame-4:d.summary.frame
        c=calls(frame,:);refPose=c{1,{'referenceX','referenceY','referencePsi'}};
        raw=loadPointCloudFrame(fullfile(pwd,'data',mc.pointCloudMatPath),frame);
        [~,tilt]=poseRowToPlanarPose(poses(frame,:));cfgp.coarseProbabilityCloud.projectionRotation=tilt;
        current=perceiveCoarseProbabilityCloud(raw,cfgp);
        [referenceSource,history]=updateLocalizationSourceWindow(current,c.timeSeconds,refPose,history,localizationSourceWindowConfig());
    end
    r=registerSemanticProbabilityCloud(d.fixed,referenceSource,d.seed,d.cfg);
    rows(end+1,:)=score("reference_window_motion",r,d.ref,d.cfg.pyramid.mapMergeRadius);
    controls=cell2table(rows,VariableNames={'variant','accepted','reason','errorM','yawErrorDeg','mapRadiusM','coarseRetained','fineShiftM'});
    writetable(controls,fullfile(dest,'basin_controls.csv'));disp(controls);
    % Original curb tangent axes for a geometry plot, in reference body axes.
    c=d.fixed.components;nr=cell(c.numComponents,7);
    for k=1:c.numComponents
        [v,e]=eig(c.covariance(:,:,k),'vector');[big,j]=max(e);t=v(:,j).'*R;xy=(c.mean(k,:)-d.ref(1:2))*R;
        nr(k,:)={d.ids(k),c.semanticName(k),xy(1),xy(2),t(1),t(2),sqrt(big)};
    end
    writetable(cell2table(nr,VariableNames={'globalId','class','x','y','tangentX','tangentY','majorStd'}),fullfile(dest,'map_axes.csv'));
    files=[string(mfilename('fullpath'))+".m",string(fullfile(dest,'diagnoseMaximum.m'))];issues=cell(2,1);
    for k=1:2,issues{k}=checkcode(files(k),'-config=factory');end
    fid=fopen(fullfile(dest,'code_analysis.json'),'w');fprintf(fid,'%s\n',jsonencode(struct('files',files,'issues',{issues})));fclose(fid);
    save(fullfile(out,'basins.mat'),'controls','results','referenceSource','-v7.3');
    fprintf('MAXIMUM_BASINS_COMPLETED\n');
end
function row=score(label,r,ref,radius)
    delta=r.poseXYTheta-ref;
    row={label,r.accepted,r.reason,norm(delta(1:2)),rad2deg(atan2(sin(delta(3)),cos(delta(3)))),radius,r.pyramid.coarseRetained,r.pyramid.refinementShiftM};
end
