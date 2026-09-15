function report=designMncavMotionAidedGains(outputFolder,options)
% designMncavMotionAidedGains Select physical gains on the first 60 s only.
% Minimize position RMSE subject to no increase in design-segment position
% RMSE, peak, P95 and heading RMSE versus the identical continuous LiDAR input.
% All gains must also satisfy the structured common quadratic certificate.
% The remaining recorded samples do not enter any selection criterion.
    arguments
        outputFolder (1,1) string="output/mncav_motion_aided_20260914/design"
        options.InputFile (1,1) string="output/mncav_zero_delay_20260914/experiment.mat"
    end
    setupVehicleLocalization;if ~isfolder(outputFolder),mkdir(outputFolder);end
    base=load(options.InputFile,'data','lateralInput','pvaPose','uniformIndices');
    selected=base.data.highRate.time<=60;n=nnz(selected);data=base.data;
    for name=string(fieldnames(data.highRate)).',data.highRate.(name)=data.highRate.(name)(selected);end
    data.lidar.time=data.lidar.time(selected);data.lidar.pose=data.lidar.pose(selected,:);
    data.lidar.information=data.lidar.information(:,:,selected);
    lateral=base.lateralInput;
    for name=string(fieldnames(lateral)).',lateral.(name)=lateral.(name)(selected);end
    indices=base.uniformIndices;indices=indices(indices<=n);reference=base.pvaPose(indices,:);
    raw=metrics(data.lidar.pose(indices,:),reference);
    [p,v,a,y]=ndgrid([3,4,5,6,8,10],[1,4,8],[4,12],[.5,1,2,4]);
    gains=[p(:),v(:),a(:),y(:)];values=NaN(size(gains));margins=zeros(size(gains,1),1);
    for k=1:size(gains,1)
        cfg=motionAidedObserverConfig;cfg.gains=gains(k,:);
        design=designMotionAidedObserverGains(cfg);margins(k)=design.translationDissipationMargin;
        estimate=runMotionAidedVehicleObserver(data,lateral,cfg);
        values(k,:)=metrics(estimate.pose(indices,:),reference);
        if mod(k,24)==0,fprintf('Completed %d/%d design-segment gain candidates.\n',k,size(gains,1));end
    end
    feasible=all(values<=raw+1e-10,2);
    assert(any(feasible),'VehicleLocalization:MotionAccuracyGateFailed','No tested gain satisfies all design criteria.');
    candidates=find(feasible);[~,best]=min(values(candidates,1));winner=candidates(best);
    cfg=motionAidedObserverConfig;cfg.gains=gains(winner,:);design=designMotionAidedObserverGains(cfg);
    tableOut=array2table([gains,values,margins,feasible],VariableNames= ...
        {'positionGain','velocityGain','accelerationGain','headingGain','positionRmseM', ...
        'positionMaximumM','positionP95M','headingRmseDeg','certificateMargin','accuracyFeasible'});
    report=struct('selectionEndSeconds',60,'candidateCount',size(gains,1), ...
        'feasibleCount',nnz(feasible),'winnerRow',winner,'gains',cfg.gains, ...
        'rawDesignMetrics',raw,'selectedDesignMetrics',values(winner,:), ...
        'metricOrder',"position RMSE, maximum, P95 (m), heading RMSE (deg)", ...
        'selection',"Minimum position RMSE among candidates no worse on all four design metrics; no reserved-segment objective", ...
        'inputFile',options.InputFile,'cfg',cfg,'design',design);
    writetable(tableOut,fullfile(outputFolder,'gain_candidates.csv'));
    save(fullfile(outputFolder,'selected_design.mat'),'report','cfg','design');
    fid=fopen(fullfile(outputFolder,'selection.json'),'w');assert(fid>=0);cleanup=onCleanup(@() fclose(fid));
    fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));disp(report.gains);
end

function m=metrics(pose,reference)
    e=vecnorm(pose(:,1:2)-reference(:,1:2),2,2);y=atan2(sin(pose(:,3)-reference(:,3)),cos(pose(:,3)-reference(:,3)));
    m=[rms(e),max(e),prctile(e,95),rad2deg(rms(y))];
end
