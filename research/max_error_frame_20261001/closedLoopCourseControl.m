function closedLoopCourseControl()
% closedLoopCourseControl Rematch around the maximum with course-rotated motion.
% The observer runs over the whole route. Scans in a window around the frame
% with the largest fused discrepancy are rematched from the fused seed; all
% other frames reuse the saved closed-loop packets. The oracle rotates the
% body-frame translation by the route-median INSPVA course offset in both the
% observer propagation and the source-window transport. It uses evaluation
% data and is diagnostic only.
    root=setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    out='output/max_error_frame_20261001';
    b=load(fullfile(out,'run','observer','experiment.mat'),'data','lateral','cfg','runs','reference');
    saved=load(fullfile(out,'run','matching','report.mat'),'cfg','report');
    d=load(fullfile(out,'diagnostic.mat'),'summary','frame','at');
    est=b.runs{1}.estimate;calls=saved.report.calls;frame=d.frame;at=d.at;ref=b.reference;n=numel(frame);
    first=at-47;last=min(n,at+23);ids=(first-4:last).';rowsOf=arrayfun(@(f)find(calls.frame==f),frame(ids));
    cfg=saved.cfg;cfg.sourceWindow=localizationSourceWindowConfig();registrationCfg=distributionRegistrationConfig();
    offset=deg2rad(d.summary.oracleConstantCourseOffsetDeg);
    odometry=saved.report.deadReckoning{rowsOf,{'x','y','psi'}};rotated=odometry;
    for k=2:numel(ids)
        step=(odometry(k,1:2)-odometry(k-1,1:2))*rot(odometry(k-1,3));
        rotated(k,1:2)=rotated(k-1,1:2)+(rot(odometry(k-1,3)+offset)*step.').';
    end
    mapCfg=featureMapBuildConfig();
    poses=readFramePoseTable(fullfile(root,'data',mapCfg.poseMatchCsvPath),frame(ids).');
    store=matfile(fullfile(root,'data',mapCfg.pointCloudMatPath));block=store.pointClouds(1,frame(ids).');
    motions={odometry,rotated};sources={cell(n,1),cell(n,1)};histories={[],[]};
    for k=1:numel(ids)
        [~,tilt]=poseRowToPlanarPose(poses(k,:));cfg.perception.coarseProbabilityCloud.projectionRotation=tilt;
        cloud=perceiveCoarseProbabilityCloud(block(k),cfg.perception);
        for j=1:2
            [sources{j}{ids(k)},histories{j}]=updateLocalizationSourceWindow(cloud,calls.timeSeconds(rowsOf(k)), ...
                motions{j}(k,:),histories{j},cfg.sourceWindow);
        end
    end
    map=load(saved.report.metadata.sourceMap,'cloud');fixed=map.cloud;
    frozen=b.data.lidar;data=rmfield(b.data,'lidar');
    runs=cell(1,2);
    for j=1:2
        lateral=b.lateral;
        if j==2,lateral.lateralVelocity=lateral.lateralVelocity+data.highRate.longitudinalSpeed*tan(offset);end
        current=data;
        current.lidarMatcher=@(k,seed,aid) matcher(k,seed,aid,frozen,first,last,sources{j},fixed,registrationCfg);
        runs{j}=runSynchronousLocalizationObserver(current,b.cfg,lateral);
    end
    reproduction=max(abs(runs{1}.z-est.z),[],'all');
    assert(reproduction<1e-9,'Rematching the window with production motion must reproduce the saved run.');
    window=(first:last).';columns=cell(1,2);
    for j=1:2
        fused=zeros(numel(window),3);matched=nan(numel(window),3);
        for i=1:numel(window)
            k=window(i);fused(i,:)=bodyError(runs{j}.pose(k,:),ref(k,:));r=runs{j}.matchingResults{k};
            if r.accepted,matched(i,:)=bodyError(r.poseXYTheta,ref(k,:));end
        end
        columns{j}=[fused,matched];
    end
    trace=array2table([frame(window),est.time(window),columns{1},columns{2}],VariableNames={'frame','time', ...
        'productionFusedLongitudinalM','productionFusedLateralM','productionFusedYawDeg', ...
        'productionMatchLongitudinalM','productionMatchLateralM','productionMatchYawDeg', ...
        'oracleFusedLongitudinalM','oracleFusedLateralM','oracleFusedYawDeg', ...
        'oracleMatchLongitudinalM','oracleMatchLateralM','oracleMatchYawDeg'});
    writetable(trace,fullfile(dest,'closed_loop_control.csv'));
    i=find(window==at);settled=window>=first+20;
    summary=struct('frame',frame(at),'rematchedFrames',[frame(first),frame(last)],'reproductionMaximumStateDifference',reproduction, ...
        'courseOffsetDeg',rad2deg(offset), ...
        'production',struct('fusedErrorM',hypot(columns{1}(i,1),columns{1}(i,2)),'fusedLateralM',columns{1}(i,2), ...
        'matchLateralM',columns{1}(i,5),'windowFusedRmseM',rms(hypot(columns{1}(settled,1),columns{1}(settled,2)))), ...
        'oracle',struct('fusedErrorM',hypot(columns{2}(i,1),columns{2}(i,2)),'fusedLateralM',columns{2}(i,2), ...
        'matchLateralM',columns{2}(i,5),'windowFusedRmseM',rms(hypot(columns{2}(settled,1),columns{2}(settled,2)))), ...
        'scope',"Oracle course rotation over the whole route in the observer and in rematched source windows; frames outside the window reuse saved packets");
    fid=fopen(fullfile(dest,'closed_loop_control.json'),'w');fprintf(fid,'%s\n',jsonencode(summary,PrettyPrint=true));fclose(fid);
    disp(summary);disp(summary.production);disp(summary.oracle);disp(trace(max(1,i-12):min(height(trace),i+6),[1 4 7 10 13 14]));
end
function r=matcher(k,seed,aid,frozen,first,last,sources,map,cfg)
    if k>=first && k<=last
        r=matchLocalProbabilityCloud(map,sources{k},seed,cfg,aid);
    else
        r=struct('poseXYTheta',frozen.pose(k,:),'information',frozen.information(:,:,k),'accepted',logical(frozen.valid(k)), ...
            'directionalAccepted',false,'reason',"savedPacket");
    end
end
function e=bodyError(pose,ref)
    delta=(pose(1:2)-ref(1:2))*rot(ref(3));e=[delta,rad2deg(atan2(sin(pose(3)-ref(3)),cos(pose(3)-ref(3))))];
end
function r=rot(a),r=[cos(a) -sin(a);sin(a) cos(a)];end
