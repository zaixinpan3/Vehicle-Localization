function inspectBasins827()
% inspectBasins827 Separate map merging, local minima and window motion.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    out='output/frame827_matching_diagnosis_20260928';d=load(fullfile(out,'diagnostic.mat'));
    s=load('output/line_direction_matching_20260928/sources.mat');
    p=load('output/line_direction_matching_20260928/production/report.mat','report');calls=p.report.calls;
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
    for frame=810:827
        c=calls(frame,:);refPose=c{1,{'referenceX','referenceY','referencePsi'}};
        [referenceSource,history]=updateLocalizationSourceWindow(s.currentClouds{frame},c.timeSeconds,refPose,history,s.cfg.sourceWindow);
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
    nr=cell(0,9);
    for frame=820:835
        c=calls(frame,:);seed=c{1,{'predictedX','predictedY','predictedPsi'}};ref=c{1,{'referenceX','referenceY','referencePsi'}};
        [local,~]=selectLocalProbabilityCloud(s.fixed,seed,d.cfg.localMapRadius);
        cfg=d.cfg;cfg.pyramid.mapMergeRadius=0;
        r=registerSemanticProbabilityCloud(local,s.sources{frame},seed,d.cfg);
        assert(max(abs(r.poseXYTheta-c{1,{'x','y','psi'}}))<1e-7);
        alt=registerSemanticProbabilityCloud(local,s.sources{frame},seed,cfg);
        body=(r.poseXYTheta(1:2)-ref(1:2))*[cos(ref(3)) -sin(ref(3));sin(ref(3)) cos(ref(3))];
        nr(end+1,:)={frame,c.positionErrorM,norm(alt.poseXYTheta(1:2)-ref(1:2)),alt.accepted,alt.reason, ...
            nnz(s.sources{frame}.components.semanticName=="curb"),nnz(s.sources{frame}.components.semanticName=="pole"),nnz(s.sources{frame}.components.semanticName=="trafficSign"),body(1)}; %#ok<AGROW>
    end
    writetable(cell2table(nr,VariableNames={'frame','productionErrorM','noMapMergeErrorM','alternativeAccepted','alternativeReason','curbs','poles','signs','forwardErrorM'}),fullfile(dest,'neighborhood_controls.csv'));
    files=[string(mfilename('fullpath'))+".m",string(fullfile(dest,'diagnoseFrame827.m'))];issues=cell(2,1);
    for k=1:2,issues{k}=checkcode(files(k),'-config=factory');end
    fid=fopen(fullfile(dest,'code_analysis.json'),'w');fprintf(fid,'%s\n',jsonencode(struct('files',files,'issues',{issues})));fclose(fid);
    save(fullfile(out,'basins.mat'),'controls','results','referenceSource','-v7.3');
    fprintf('FRAME827_BASINS_COMPLETED\n');
end
function row=score(label,r,ref,radius)
    delta=r.poseXYTheta-ref;
    row={label,r.accepted,r.reason,norm(delta(1:2)),rad2deg(atan2(sin(delta(3)),cos(delta(3)))),radius,r.pyramid.coarseRetained,r.pyramid.refinementShiftM};
end
