function runObserverControls()
% runObserverControls Isolate LiDAR position and heading errors offline.
% Preserve validity, geometry information, initial state and all motion inputs.
    setupVehicleLocalization();
    s=load('output/matching_refinement_20260919/observer_final/experiment.mat');
    variants=["production","reference_lidar_xy","reference_lidar_yaw","reference_lidar_pose"];
    rows=cell(0,7);
    for v=variants
        data=s.data;
        if any(v==["reference_lidar_xy","reference_lidar_pose"]), data.lidar.pose(:,1:2)=s.reference(:,1:2); end
        if any(v==["reference_lidar_yaw","reference_lidar_pose"]), data.lidar.pose(:,3)=s.reference(:,3); end
        e=runFullLocalizationObserver(data,s.lateralDesign,s.cfg,LateralInputs=s.lateral);
        err=vecnorm(e.position-s.reference(:,1:2),2,2);
        yaw=rad2deg(atan2(sin(e.heading-s.reference(:,3)),cos(e.heading-s.reference(:,3))));
        rows(end+1,:)={v,rms(err),prctile(err,95),max(err),rms(yaw),max(err(e.time>=8 & e.time<=10)),max(err(e.time>=80 & e.time<=85))}; %#ok<AGROW>
    end
    results=cell2table(rows,VariableNames={'variant','positionRmseM','positionP95M','maximumM','yawRmseDeg','peak8to10M','peak80to85M'});
    writetable(results,'output/error_attribution_20260919/observer_controls.csv');disp(results);
    e=s.runs{1}.estimate; h=s.data.highRate;
    corrected=e.diagnostics.gnssPositionAtObserverPoint;
    raw=table(h.time,h.longitudinalSpeed,s.lateral.lateralVelocity,h.yawRate, ...
        s.data.lidar.pose(:,1),s.data.lidar.pose(:,2),s.data.lidar.pose(:,3),s.data.lidar.valid, ...
        corrected(:,1),corrected(:,2),s.data.gnss.valid, ...
        s.reference(:,1),s.reference(:,2),s.reference(:,3),e.velocity(:,1),e.velocity(:,2), ...
        VariableNames={'time','wheelSpeed','rawLateralVelocity','yawRate','lidarX','lidarY','lidarPsi','lidarValid', ...
        'gnssX','gnssY','gnssValid','referenceX','referenceY','referencePsi','estimatedVx','estimatedVy'});
    writetable(raw,'output/error_attribution_20260919/aligned_inputs.csv');
end
