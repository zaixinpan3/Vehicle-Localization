function metrics=scoreDowntownReferenceLocalization(datasetFolder,localizationFolder)
% scoreDowntownReferenceLocalization Separate post-run pseudo-reference scorer.
% Direct map-frame planar discrepancies; no fit or trajectory alignment.
    truth=readtable(fullfile(datasetFolder,'mapping_poses.csv'));run=readtable(fullfile(localizationFolder,'trajectory.csv'));
    [covered,index]=ismember(run.frame_index,truth.frame_index);ref=truth(index(covered),:);run=run(covered,:);reference=zeros(height(run),3);
    for k=1:height(run),reference(k,:)=poseSupport.poseRowToPlanarPose(ref(k,:));end
    delta=[run.x_m,run.y_m,run.yaw_rad]-reference;delta(:,3)=atan2(sin(delta(:,3)),cos(delta(:,3)));
    e=vecnorm(delta(:,1:2),2,2);heading=rad2deg(delta(:,3));
    metrics=struct('samples',numel(e),'positionRmseM',sqrt(mean(e.^2)),'positionMedianM',median(e), ...
        'positionP95M',prctile(e,95),'positionMaximumM',max(e),'headingRmseDeg',sqrt(mean(heading.^2)), ...
        'headingMaximumAbsDeg',max(abs(heading)),'matchedFrames',nnz(run.accepted | run.directional_accepted), ...
        'referenceKind',"pseudo_ground_truth",'alignment',"None; fixed map coordinates",'sameDriveCorrelated',true);
    if all(ismember({'dead_reckoning_x_m','dead_reckoning_y_m','dead_reckoning_yaw_rad'},run.Properties.VariableNames))
        dr=[run.dead_reckoning_x_m,run.dead_reckoning_y_m,run.dead_reckoning_yaw_rad]-reference;
        dr(:,3)=atan2(sin(dr(:,3)),cos(dr(:,3)));
        metrics.deadReckoningPositionRmseM=sqrt(mean(sum(dr(:,1:2).^2,2)));
        metrics.deadReckoningHeadingRmseDeg=rad2deg(sqrt(mean(dr(:,3).^2)));
    end
    errors=array2table([run.frame_index,run.native_time_sec,reference,delta,e], ...
        'VariableNames',{'frame_index','native_time_sec','reference_x_m','reference_y_m','reference_yaw_rad','error_x_m','error_y_m','error_yaw_rad','position_error_m'});
    writetable(errors,fullfile(localizationFolder,'reference_errors.csv'));
    f=fopen(fullfile(localizationFolder,'metrics.json'),'w');fprintf(f,'%s\n',jsonencode(metrics,PrettyPrint=true));fclose(f);disp(metrics);
end
