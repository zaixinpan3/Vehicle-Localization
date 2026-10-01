function diagnoseMaximumFrameMatching()
% diagnoseMaximumFrameMatching Isolate source-window transport at the maximum.
% Recomputes the coarse clouds of the five-scan horizon ending at the frame
% with the largest fused discrepancy and rematches them with the saved fused
% seed and GNSS selection aid. Reference transport, the constant course
% rotation and the reference seed are offline oracles, never runtime inputs.
    root=setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    out='output/max_error_frame_20261001';
    b=load(fullfile(out,'run','observer','experiment.mat'),'data','cfg','runs','reference');
    saved=load(fullfile(out,'run','matching','report.mat'),'cfg','report');
    d=load(fullfile(out,'diagnostic.mat'),'summary','frame','at');
    est=b.runs{1}.estimate;calls=saved.report.calls;at=d.at;target=d.frame(at);ref=b.reference(at,:);
    cfg=saved.cfg;cfg.sourceWindow=localizationSourceWindowConfig();registrationCfg=distributionRegistrationConfig();
    frames=(target-4:target).';rowsOf=arrayfun(@(f)find(calls.frame==f),frames);
    assert(calls.timeSeconds(rowsOf(end))-calls.timeSeconds(rowsOf(1))<=cfg.sourceWindow.maximumAgeSeconds);
    mapCfg=featureMapBuildConfig();
    poses=readFramePoseTable(fullfile(root,'data',mapCfg.poseMatchCsvPath),frames.');
    store=matfile(fullfile(root,'data',mapCfg.pointCloudMatPath));block=store.pointClouds(1,frames.');
    odometry=saved.report.deadReckoning{rowsOf,{'x','y','psi'}};
    truth=calls{rowsOf,{'referenceX','referenceY','referencePsi'}};
    offset=deg2rad(d.summary.oracleConstantCourseOffsetDeg);
    % Rotate each odometry translation increment by the route-median course
    % offset; yaw increments are unchanged.
    rotated=odometry;
    for k=2:numel(frames)
        step=(odometry(k,1:2)-odometry(k-1,1:2))*rot(odometry(k-1,3));
        rotated(k,1:2)=rotated(k-1,1:2)+(rot(odometry(k-1,3)+offset)*step.').';
    end
    motions={odometry,truth,rotated};labels=["odometry","reference_motion_oracle","course_rotated_odometry_oracle"];
    sources=cell(1,3);histories={[],[],[]};raw=cell(numel(frames),1);transport=cell(numel(frames),7);
    for k=1:numel(frames)
        [~,tilt]=poseRowToPlanarPose(poses(k,:));cfg.perception.coarseProbabilityCloud.projectionRotation=tilt;
        raw{k}=perceiveCoarseProbabilityCloud(block(k),cfg.perception);
        for j=1:3
            [sources{j},histories{j},~,current]=updateLocalizationSourceWindow(raw{k},calls.timeSeconds(rowsOf(k)), ...
                motions{j}(k,:),histories{j},cfg.sourceWindow);
            if j==1,single=current;end
        end
        own=(odometry(k,1:2)-odometry(end,1:2))*rot(odometry(end,3));
        actual=(truth(k,1:2)-truth(end,1:2))*rot(truth(end,3));
        transport(k,:)={frames(k),calls.timeSeconds(rowsOf(end))-calls.timeSeconds(rowsOf(k)),own(1),own(2),actual(1),actual(2),own(2)-actual(2)};
    end
    relativeMotion=cell2table(transport,VariableNames={'frame','ageSeconds','odometryLongitudinalM','odometryLateralM', ...
        'referenceLongitudinalM','referenceLateralM','lateralDisagreementM'});
    writetable(relativeMotion,fullfile(dest,'window_transport.csv'));disp(relativeMotion);
    map=load(saved.report.metadata.sourceMap,'cloud');fixed=map.cloud;
    seed=est.matchingSeeds(at,:);aid=struct('valid',b.data.gnss.valid(at),'timestamp',est.time(at));
    if aid.valid
        [aid.position,I]=correctGnssOutputPoint(b.data.gnss.position(at,:),b.data.gnss.information(:,:,at),seed(3),b.cfg.gnss.outputPoint);
        aid.covariance=I\eye(2);
    end
    names=["production","reference_seed","without_position_aid","current_scan_only",labels(2:3),"reference_motion_and_seed"];
    inputs={sources{1},seed,aid;sources{1},ref,aid;sources{1},seed,[];single,seed,aid; ...
        sources{2},seed,aid;sources{3},seed,aid;sources{2},ref,aid};
    rows=cell(numel(names),11);results=cell(numel(names),1);
    for j=1:numel(names)
        r=matchLocalProbabilityCloud(fixed,inputs{j,1},inputs{j,2},registrationCfg,inputs{j,3});results{j}=r;
        e=r.poseXYTheta-ref;bodyError=e(1:2)*rot(ref(3));c=inputs{j,1}.components;
        rows(j,:)={names(j),norm(e(1:2)),bodyError(1),bodyError(2),rad2deg(wrap(e(3))),r.accepted,string(r.reason),r.observableRank, ...
            nnz(c.semanticName=="pole"),nnz(c.semanticName=="curb"),nnz(c.semanticName=="trafficSign")};
    end
    reproduction=max(abs(results{1}.poseXYTheta-est.matchingResults{at}.poseXYTheta));
    assert(reproduction<1e-7,'The recomputed horizon does not reproduce the saved closed-loop match.');
    controls=cell2table(rows,VariableNames={'variant','errorM','longitudinalM','lateralM','yawErrorDeg','accepted','reason','rank', ...
        'poles','curbs','signs'});
    writetable(controls,fullfile(dest,'matching_controls.csv'));disp(controls);
    pairs=results{1}.correspondences;
    writetable(pairs(:,{'source','globalTarget','semanticName','squaredStandardizedResidual','weight','robustWeight','slidingFraction'}), ...
        fullfile(dest,'matching_pairs.csv'));
    summary=struct('frame',target,'reproductionMaximumPoseDifference',reproduction,'seed',seed,'reference',ref, ...
        'seedBodyErrorM',(seed(1:2)-ref(1:2))*rot(ref(3)),'information',results{1}.information, ...
        'curvatureEigenvalues',results{1}.curvatureEigenvalues,'similarity',results{1}.similarity, ...
        'oldestScanLateralDisagreementM',relativeMotion.lateralDisagreementM(1), ...
        'meanLateralDisagreementM',mean(relativeMotion.lateralDisagreementM), ...
        'oracleScope',"Reference transport, course rotation and reference seed use evaluation data and are diagnostic only");
    fid=fopen(fullfile(dest,'matching_summary.json'),'w');fprintf(fid,'%s\n',jsonencode(summary,PrettyPrint=true));fclose(fid);
    save(fullfile(out,'matching_diagnostic.mat'),'results','names','sources','single','raw','seed','aid','ref','controls','relativeMotion');
    disp(summary);
end
function r=rot(a),r=[cos(a) -sin(a);sin(a) cos(a)];end
function a=wrap(a),a=atan2(sin(a),cos(a));end
