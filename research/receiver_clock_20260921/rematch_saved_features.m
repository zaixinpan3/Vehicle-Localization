function report=rematch_saved_features(root)
% rematch_saved_features Re-evaluate historical D2D after clock/map rebuilding.
% Diagnostic fine-feature input only; production localization remains coarse.
% The historical solver/configuration are restored from an existing snapshot.
    oldPath=path;cleanup=onCleanup(@()path(oldPath));
    runtime=fullfile(root,'output/reference_free_785_20260920/historical_runtime');
    addpath(genpath(runtime),'-begin');
    assert(contains(which('registerSemanticProbabilityCloud'),'historical_runtime'));
    source=load(fullfile(root,'output/saved_perception_inspva_20260915/experiment.mat'),'cfg','pcfg');
    loaded=load(fullfile(root,'output/mississippi_mapping_synchronized/feature_observations.mat'),'featureData');
    data=loaded.featureData;
    loaded=load(fullfile(root,'output/mississippi_mapping_synchronized/probability_cloud.mat'),'cloud');
    fixed=loaded.cloud;cfg=source.cfg;rows=cell(1170,9);results=cell(1170,1);timer=tic;
    for k=1:1170
        pose=poseRowToPlanarPose(data.framePoseTable(k,:));
        moving=buildSavedFeatureProbabilityCloud(data,k,pose,source.pcfg);
        result=registerSemanticProbabilityCloud(fixed,moving,pose,cfg);results{k}=result;
        error=result.poseXYTheta-pose;
        rows(k,:)={k,data.framePoseTable.receiver_time_sec(k),result.accepted,result.observableRank, ...
            norm(error(1:2)),rad2deg(atan2(sin(error(3)),cos(error(3)))), ...
            result.poseXYTheta(1),result.poseXYTheta(2),result.poseXYTheta(3)};
    end
    calls=cell2table(rows,VariableNames={'frame','time','accepted','rank','positionErrorM','headingErrorDeg','x','y','yaw'});
    e=calls.positionErrorM(calls.accepted);
    report=struct('frames',1170,'accepted',nnz(calls.accepted),'positionRmseM',rms(e), ...
        'positionMedianM',median(e),'positionP95M',prctile(e,95),'positionMaximumM',max(e), ...
        'frame307ErrorM',calls.positionErrorM(307),'frame307Accepted',calls.accepted(307), ...
        'seconds',toc(timer),'clockModelId',fixed.clockModelId,'solverRevision',"d6502080", ...
        'referenceSeedEveryFrame',true,'source',"unchanged saved fine labels reprojected with corrected full poses", ...
        'limitation',"Historical fine-input reference-seeded same-drive diagnostic; not production coarse-only localization or independent ground truth");
    out=fullfile(root,'research/receiver_clock_20260921');
    writetable(calls,fullfile(out,'rematched_fine_calls.csv'));
    fid=fopen(fullfile(out,'rematched_fine_summary.json'),'w');assert(fid>=0);fileCleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));
    save(fullfile(root,'output/receiver_clock_20260921/rematched_fine.mat'),'report','calls','results','cfg','-v7.3');
    disp(report);
end
