function report=auditFullObserverLidarComparison(outputFolder)
% auditFullObserverLidarComparison Compare raw matching and fusion at equal times.
% Add outputs at already existing LiDAR integration boundaries using identical
% left-held inputs. Verify the original 100 Hz state sequence is unchanged.
% No estimator, gain, measurement, map or reference is tuned by this audit.
    arguments
        outputFolder (1,1) string="output/full_observer_comparison_20260916"
    end
    setupVehicleLocalization();if ~isfolder(outputFolder),mkdir(outputFolder);end
    saved=load('output/full_observer_20260916/experiment.mat');
    data=saved.data;originalTime=data.highRate.time;
    nativeTime=data.lidar.time;query=unique([originalTime;nativeTime]);
    for name=string(fieldnames(data.highRate)).'
        if name~="time",data.highRate.(name)=interp1(originalTime,data.highRate.(name),query,'previous');end
    end
    data.highRate.time=query;
    lateral=struct('time',query,'lateralVelocity',interp1(originalTime,saved.lateral.lateralVelocity,query,'previous'), ...
        'sideSlipAngleRate',interp1(originalTime,saved.lateral.sideSlipAngleRate,query,'previous'));
    [present,uniform]=ismember(originalTime,query);assert(all(present));
    [present,native]=ismember(nativeTime,query);assert(all(present));
    full=runFullLocalizationObserver(data,saved.lateralDesign,saved.cfg,LateralInputs=lateral);
    fullDifference=max(abs(full.z(uniform,:)-saved.runs{1}.estimate.z),[],'all');assert(fullDifference==0);
    withoutGnss=rmfield(data,'gnss');
    lidarMotion=runFullLocalizationObserver(withoutGnss,saved.lateralDesign,saved.cfg,LateralInputs=lateral);
    lidarDifference=max(abs(lidarMotion.z(uniform,:)-saved.runs{2}.estimate.z),[],'all');assert(lidarDifference==0);
    mapping=load('output/saved_perception_inspva_20260915/experiment.mat','time','reference');
    frameIndex=(1:numel(nativeTime)).';
    frameClockDifference=max(abs(nativeTime-mapping.time(frameIndex)));
    assert(frameClockDifference<1e-10,'VehicleLocalization:FrameClockMismatch','Frame IDs must preserve acquisition times.');
    reference=mapping.reference(frameIndex,:);valid=data.lidar.valid;
    old=load('output/mncav_interface_audit_20260916/correction_experiment.mat','runs');old=old.runs{1,4}.estimate;
    [present,oldIndex]=ismember(mapping.time(frameIndex),old.time);assert(all(present));
    methods=["raw_lidar_matching","full_observer","lidar_motion_observer","previous_offline_observer"];
    poses={data.lidar.pose,full.pose(native,:),lidarMotion.pose(native,:),old.pose(oldIndex,:)};
    rows=cell(0,11);frameRows=cell(numel(methods),1);
    populations={"same_accepted_frames",valid;"accepted_before60",valid & nativeTime<=60; ...
        "accepted_after60",valid & nativeTime>60;"accepted_80_86",valid & nativeTime>=80 & nativeTime<86};
    for j=1:numel(methods)
        p=poses{j};e=vecnorm(p(:,1:2)-reference(:,1:2),2,2);
        yaw=atan2(sin(p(:,3)-reference(:,3)),cos(p(:,3)-reference(:,3)));
        frameRows{j}=table(frameIndex,nativeTime,valid,repmat(methods(j),numel(nativeTime),1),e,rad2deg(yaw), ...
            VariableNames={'frame','time','accepted','method','positionErrorM','headingErrorDeg'});
        for k=1:size(populations,1)
            mask=populations{k,2};metrics=score(e(mask),yaw(mask));
            rows(end+1,:)=[{methods(j),populations{k,1},nnz(mask)},num2cell(metrics)]; %#ok<AGROW>
        end
    end
    metrics=cell2table(rows,VariableNames={'method','population','samples','positionRmseM','positionMedianM', ...
        'positionP95M','positionMaximumM','fractionAtMost5cm','fractionAtMost10cm','headingRmseDeg','headingMaximumDeg'});
    frameErrors=vertcat(frameRows{:});
    uniformReference=saved.reference;estimate=saved.runs{1}.estimate;diag=estimate.diagnostics;
    e=vecnorm(estimate.position-uniformReference(:,1:2),2,2);
    yaw=atan2(sin(estimate.heading-uniformReference(:,3)),cos(estimate.heading-uniformReference(:,3)));
    masks={"all_uniform",true(size(originalTime));"both_channels",diag.mode==3; ...
        "gnss_without_lidar",diag.mode==1;"time_80_86",originalTime>=80 & originalTime<86; ...
        "outside_80_86",originalTime<80 | originalTime>=86};
    segments=cell(size(masks,1),8);
    for k=1:size(masks,1)
        mask=masks{k,2};segments(k,:)={masks{k,1},nnz(mask),rms(e(mask)),median(e(mask)), ...
            max(e(mask)),sum(e(mask).^2)/sum(e.^2),rad2deg(rms(yaw(mask))),nnz(diag.headingMode(mask)==1)};
    end
    segments=cell2table(segments,VariableNames={'population','samples','positionRmseM','positionMedianM', ...
        'positionMaximumM','fractionOfTotalSquaredPositionError','headingRmseDeg','gnssHeadingSamples'});
    % Diagnostic control only: preserve GNSS XY and nearly remove its derived
    % heading feedback. This is not a proposed gain or a production selection.
    controlCfg=saved.cfg;controlCfg.gnss.headingGain=1e-12;
    headingControl=runFullLocalizationObserver(saved.data,saved.lateralDesign,controlCfg,LateralInputs=saved.lateral);
    controlError=vecnorm(headingControl.position-uniformReference(:,1:2),2,2);
    controlYaw=atan2(sin(headingControl.heading-uniformReference(:,3)),cos(headingControl.heading-uniformReference(:,3)));
    controlMetrics=score(controlError,controlYaw);
    report=struct('sameAcceptedSamples',nnz(valid),'rawReference',"Exact stored INSPVA frame poses used in the raw matching baseline", ...
        'fullUniformStateDifference',fullDifference,'lidarUniformStateDifference',lidarDifference, ...
        'maximumCsvFrameClockDifference',frameClockDifference, ...
        'nativePoseInjectionChanged',false,'gainsTuned',false,'metrics',metrics,'segments',segments, ...
        'headingControl',struct('gain',controlCfg.gnss.headingGain,'uniformMetrics',controlMetrics, ...
            'scope',"Diagnostic removal of derived GNSS yaw; GNSS position and all other inputs preserved; not selected for deployment"), ...
        'limitations',"Same-drive map and reference-seeded matching; ODOM GNSS/INS aiding; previous offline observer uses future-endpoint reconstruction; accepted-only matching has no valid result during failed frames");
    save(fullfile(outputFolder,'audit.mat'),'report','frameErrors','full','lidarMotion','headingControl', ...
        'reference','uniformReference','query','native','uniform','-v7.3');
    writetable(metrics,fullfile(outputFolder,'paired_metrics.csv'));
    writetable(frameErrors,fullfile(outputFolder,'frame_errors.csv'));
    writetable(segments,fullfile(outputFolder,'error_segments.csv'));
    fid=fopen(fullfile(outputFolder,'summary.json'),'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));
    disp(metrics(metrics.population=="same_accepted_frames",:));disp(segments);disp(report.headingControl);
end

function values=score(e,yaw)
    values=[rms(e),median(e),prctile(e,95),max(e),mean(e<=.05),mean(e<=.1),rad2deg(rms(yaw)),rad2deg(max(abs(yaw)))];
end
