function report=runMncavWheelSpeedExperiment(outputFolder)
% runMncavWheelSpeedExperiment Select wheel fusion on a separate drive.
% Calibration: 12:11:24, fit before 40 s and select on the remaining samples.
% Localization: current wheel-only 12:09:31 full-observer inputs; the lateral
% observer is recomputed. Reference velocity is scoring-only.
    arguments
        outputFolder (1,1) string="output/mncav_wheel_only_20260916/calibration"
    end
    setupVehicleLocalization();cfg=wheelSpeedObserverConfig();
    tab=readtable(fullfile(outputFolder,'12-11-24','motion.csv'));
    d=motionInput(tab,readtable(fullfile(outputFolder,'12-11-24','wheels.csv')));
    selection=tab.time>=40;rows=cell(0,7);candidates=cell(0,1);configs=cell(0,1);
    for method=["robust","mean","rear"]
        for tau=[.01,.04,.08,.15]
            c=cfg;c.method=method;c.filterTimeConstant=tau;
            v=estimateWheelLongitudinalSpeed(d,c);assert(all(v.valid));
            e=v.longitudinalSpeed(selection)-tab.referenceVx(selection);
            rows(end+1,:)={method,tau,rms(e),mean(e),prctile(abs(e),95),max(abs(e)),nnz(selection)}; %#ok<AGROW>
            candidates{end+1}=v;configs{end+1}=c; %#ok<AGROW>
        end
    end
    candidateMetrics=cell2table(rows,VariableNames={'method','timeConstant','rmseMps','biasMps','p95Mps','maximumMps','samples'});
    [~,best]=min(candidateMetrics.rmseMps);selectedCfg=configs{best};selected=candidates{best};
    writetable(candidateMetrics,fullfile(outputFolder,'calibration_candidates.csv'));
    disp(candidateMetrics);disp(selectedCfg);
    stored=load('output/mncav_wheel_only_20260916/full_observer/experiment.mat');
    h=stored.data.highRate;native=readtable(fullfile(outputFolder,'12-09-31','wheels.csv'));
    d=h;d.wheels=struct('time',native.time,'angularVelocity',native{:,2:5});
    selectedCfg.initialSpeed=0; % Same known stationary start as the frozen experiment.
    wheel=estimateWheelLongitudinalSpeed(d,selectedCfg);
    assert(all(wheel.valid(h.time>=native.time(1))),'Unexpected unbridged wheel outage.');
    data=stored.data;data.highRate.longitudinalSpeed=max(0,wheel.longitudinalSpeed);
    lateral=runLateralVelocityObserver(data.highRate,stored.lateralDesign,lateralObserverConfig("mncav"));
    estimate=runFullLocalizationObserver(data,stored.lateralDesign,stored.cfg,LateralInputs=lateral);
    ref=readtable(fullfile(outputFolder,'12-09-31','reference.csv'));
    referenceVx=interp1(ref.time,ref.referenceVx,h.time,'linear');assert(all(isfinite(referenceVx)));
    speedMetrics=velocityScore(data.highRate.longitudinalSpeed,referenceVx);
    speedMetrics=addvars(speedMetrics,"wheel_fusion",Before=1,NewVariableNames="method");
    positionMetrics=poseScore(estimate.pose,stored.reference);
    positionMetrics=addvars(positionMetrics,"wheel_fusion",Before=1,NewVariableNames="method");
    % Exact accepted-frame states: event times already used by the integrator.
    lidar=data.lidar;accepted=lidar.valid;union=unique([h.time;lidar.time]);
    expanded=data;expanded.highRate.time=union;
    for name=string(fieldnames(h)).'
        if name~="time",expanded.highRate.(name)=interp1(h.time,data.highRate.(name),union,'previous');end
    end
    expandedLateral=struct('time',union);
    for name=["lateralVelocity","sideSlipAngleRate"]
        expandedLateral.(name)=interp1(h.time,lateral.(name),union,'previous');
    end
    atNative=runFullLocalizationObserver(expanded,stored.lateralDesign,stored.cfg,LateralInputs=expandedLateral);
    [~,uniformIndex]=ismember(h.time,union);assert(isequal(atNative.z(uniformIndex,:),estimate.z));
    matching=load('output/saved_perception_inspva_20260915/experiment.mat','time','reference');
    frameIndex=(1:numel(lidar.time)).';
    assert(max(abs(lidar.time-matching.time(frameIndex)))<1e-10);
    [~,nativeIndex]=ismember(lidar.time,union);
    frameReference=matching.reference(frameIndex,:);
    pairedMetrics=poseScore(atNative.pose(nativeIndex(accepted),:),frameReference(accepted,:));
    frameError=vecnorm(atNative.position(nativeIndex,:)-frameReference(:,1:2),2,2);
    writetable(table(frameIndex,lidar.time,accepted,frameError, ...
        VariableNames={'frame','time','accepted','positionErrorM'}),fullfile(outputFolder,'frame_errors.csv'));
    output=table(h.time,data.highRate.longitudinalSpeed,referenceVx,wheel.valid,wheel.sourceAge, ...
        VariableNames={'time','wheelVx','referenceVx','wheelValid','wheelAge'});
    writetable(output,fullfile(outputFolder,'velocity_comparison.csv'));
    writetable(speedMetrics,fullfile(outputFolder,'velocity_metrics.csv'));
    writetable(positionMetrics,fullfile(outputFolder,'localization_metrics.csv'));
    report=struct('selectedConfiguration',selectedCfg,'selectionRow',best, ...
        'calibrationSelected',velocityScore(selected.longitudinalSpeed(selection),tab.referenceVx(selection)), ...
        'velocityMetrics',speedMetrics,'localizationMetrics',positionMetrics,'pairedWheelMetrics',pairedMetrics, ...
        'initialPredictionOnlySamples',nnz(~wheel.valid),'rejectedWheelSamples',sum(wheel.rejectedWheels), ...
        'runtimeReferenceVelocityUsed',false,'globalObserverGainsChanged',false);
    save(fullfile(outputFolder,'experiment.mat'),'report','data','lateral','wheel','selected','candidateMetrics', ...
        'estimate','atNative','referenceVx','speedMetrics','positionMetrics','stored','-v7.3');
    fid=fopen(fullfile(outputFolder,'summary.json'),'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));disp(report);
end

function d=motionInput(tab,wh)
    d=struct('time',tab.time,'steeringAngle',tab.steeringAngle,'yawRate',tab.yawRate, ...
        'longitudinalAcceleration',tab.longitudinalAcceleration, ...
        'wheels',struct('time',wh.time,'angularVelocity',wh{:,2:5}));
end

function result=velocityScore(v,reference)
    e=v-reference;result=table(numel(e),rms(e),mean(e),prctile(abs(e),95),max(abs(e)), ...
        VariableNames={'samples','rmseMps','biasMps','p95Mps','maximumMps'});
end

function result=poseScore(p,reference)
    e=vecnorm(p(:,1:2)-reference(:,1:2),2,2);a=atan2(sin(p(:,3)-reference(:,3)),cos(p(:,3)-reference(:,3)));
    result=table(numel(e),rms(e),median(e),prctile(e,95),max(e),rad2deg(rms(a)), ...
        VariableNames={'samples','positionRmseM','positionMedianM','positionP95M','positionMaximumM','headingRmseDeg'});
end
