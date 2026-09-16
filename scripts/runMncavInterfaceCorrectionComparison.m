function report=runMncavInterfaceCorrectionComparison(outputFolder)
% runMncavInterfaceCorrectionComparison Freeze gains and test audited inputs.
% Original accepted measurements, information, evaluation times and gains are
% retained. The LiDAR displacement adapter has fixed 2 s / 4 s constants,
% chosen before this replay; it receives no INSPVA state or error metrics.
    arguments
        outputFolder (1,1) string="output/mncav_interface_audit_20260916"
    end
    setupVehicleLocalization();if ~isfolder(outputFolder),mkdir(outputFolder);end
    source="output/mncav_inspva_observer_20260915";
    old=load(fullfile(source,'experiment.mat'));
    gaps=load(fullfile(source,'motion_gap_experiment.mat'),'experiments');
    frames=readtable(fullfile(source,'frame_errors.csv'),TextType="string");
    audit=readtable('output/mncav_inspva_median_20260915/motion_audit.csv');
    interface=jsondecode(fileread('config/mncavReplayInterface.json'));
    parameters=jsondecode(fileread(fullfile(outputFolder,'vehicle_parameters.json')));
    [prepared,~,~]=prepareMncavObserverReplay( ...
        'output/mississippi_20240607_120931_20260907/sensors',fullfile(outputFolder,'vehicle_parameters.json'));
    h=prepared.highRate;
    assert(isequal(h.time,old.high.time));
    for name=["longitudinalSpeed","longitudinalAcceleration","lateralAcceleration","yawRate"]
        assert(max(abs(h.(name)-old.high.(name)))<1e-12,'Only the steering conversion may change.');
    end
    expected=old.high.steeringAngle-interface.steeringWheelOffsetRad/parameters.steeringRatio;
    steeringDifference=max(abs(expected-h.steeringAngle));assert(steeringDifference<1e-12);
    correctedLateral=runLateralVelocityObserver(h,old.lateralDesign,old.lateralCfg);
    variants=["previous_observer","steering_only","lidar_bias_legacy_steering","steering_and_lidar_bias"];
    rows=cell(0,11);lateralRows=cell(0,7);runs=cell(3,4);frameRows=cell(3,4);checks=struct();
    for j=1:3
        e=old.experiments{j};mode=old.report.cases(j).mode;
        f=frames(frames.mode==mode,:);accepted=f.fullMeasurement==1;
        native=e.frameIndices;ix=e.uniformIndices;t=e.data.highRate.time;
        poseTime=t(native(accepted));refVy=interp1(audit.time,audit.insVy,t);
        assert(all(isfinite(refVy)));
        for v=1:4
            input=e.lateralInput;data=e.data;correction=struct();
            if v==2 || v==4
                assert(t(end)-h.time(end)<=.02);
                query=min(max(t,h.time(1)),h.time(end));
                data.highRate.steeringAngle=interp1(h.time,h.steeringAngle,query);
                for name=["lateralVelocity","sideSlipAngle","sideSlipAngleRate"]
                    input.(name)=interp1(h.time,correctedLateral.(name),query);
                end
            end
            if v>=3
                [input,correction]=correctLateralVelocityFromLidar(data,input,poseTime);
            end
            if v==1
                data=gaps.experiments{j}.data;estimate=gaps.experiments{j}.estimate;
            else
                [data,reconstruction]=reconstructMotionAidedLidarGaps(data,input,poseTime);
                estimate=runMotionAidedVehicleObserver(data,input,old.cfg);
                assert(reconstruction.maximumAcceptedPoseMismatch==0);
            end
            assert(isequal(data.lidar.pose(native(accepted),:),e.data.lidar.pose(native(accepted),:)));
            assert(isequal(data.lidar.information,e.data.lidar.information));
            assert(estimate.diagnostics.rateEnvelopeSatisfied);
            populations={"native_full",native(accepted),e.frameReference(accepted,:); ...
                "native_all",native,e.frameReference;"uniform_all",ix,e.reference(ix,:); ...
                "uniform_first60",ix(t(ix)<=60),e.reference(ix(t(ix)<=60),:); ...
                "uniform_after60",ix(t(ix)>60),e.reference(ix(t(ix)>60),:)};
            for k=1:size(populations,1)
                selected=populations{k,2};ref=populations{k,3};
                values=score(estimate.pose(selected,:),ref);
                rows(end+1,:)=[{mode,populations{k,1},variants(v),numel(selected)},num2cell(values)]; %#ok<AGROW>
            end
            moving=data.highRate.longitudinalSpeed(ix)>=5;straight=moving & abs(data.highRate.yawRate(ix))<.03;
            groups={moving,straight,moving&t(ix)<=60,moving&t(ix)>60};
            groupNames=["moving","straight","moving_first60","moving_after60"];
            for k=1:4
                selected=ix(groups{k});error=input.lateralVelocity(selected)-refVy(selected);
                lateralRows(end+1,:)={mode,variants(v),groupNames(k),numel(selected),rms(error),mean(error),median(abs(error))}; %#ok<AGROW>
            end
            if j==1 && v==4
                repeated=runMotionAidedVehicleObserver(data,input,old.cfg);
                refinedCfg=old.cfg;refinedCfg.maximumIntegrationStep=old.cfg.maximumIntegrationStep/2;
                refined=runMotionAidedVehicleObserver(data,input,refinedCfg);
                checks.repeatedStateDifference=max(abs(repeated.z-estimate.z),[],'all');
                checks.halfStepMaximumPositionDifferenceM=max(vecnorm(refined.position-estimate.position,2,2));
                assert(checks.repeatedStateDifference==0 && checks.halfStepMaximumPositionDifferenceM<1e-5);
            end
            runs{j,v}=struct('data',data,'lateral',input,'estimate',estimate,'correction',correction);
            frameRows{j,v}=table(f.frame,f.time,repmat(mode,height(f),1),repmat(variants(v),height(f),1),accepted, ...
                vecnorm(estimate.position(native,:)-e.frameReference(:,1:2),2,2), ...
                VariableNames={'frame','time','mode','variant','fullMeasurement','positionErrorM'});
            fprintf('%s %s: accepted RMSE %.5f m, median %.5f m\n',mode,variants(v), ...
                rms(frameRows{j,v}.positionErrorM(accepted)),median(frameRows{j,v}.positionErrorM(accepted)));
        end
    end
    metrics=cell2table(rows,VariableNames={'mode','population','method','samples','positionRmseM', ...
        'positionMedianM','positionP95M','positionMaximumM','fractionAtMost5cm','fractionAtMost10cm','headingRmseDeg'});
    raw=old.report.metrics(old.report.metrics.method=="lidar_measurement",:);
    metrics=[raw;metrics];
    lateralMetrics=cell2table(lateralRows,VariableNames={'mode','variant','population','samples','rmseMps','biasMps','medianAbsMps'});
    checks.steeringConversionMaximumDifference=steeringDifference;
    checks.exactAcceptedPosePreservation=true;checks.informationUnchanged=true;
    metadata=struct('confirmedVehicle',"UMN MnCAV",'vehicle',old.lateralCfg.vehicle, ...
        'globalGains',old.cfg.gains,'lateralGains',old.lateralDesign.gains,'gainsRetuned',false, ...
        'steeringInterface',interface,'biasWindowSeconds',2,'biasTimeConstantSeconds',4, ...
        'referenceInjected',false,'lidarDelaySeconds',0,'matchingRerun',false, ...
        'scope',"Diagnostic replay on previously examined same-drive reference-seeded matching; temporal later segment is not a fresh independent test", ...
        'stateInterpretation',"LiDAR adapter corrects effective mapped-reference velocity, not independently calibrated physical CG velocity", ...
        'certificateScope',"Existing lateral LPV and seven-state conditional certificates retained; no new joint physical-bias or hybrid certificate claimed");
    report=struct('metadata',metadata,'checks',checks,'metrics',metrics,'lateralMetrics',lateralMetrics);
    save(fullfile(outputFolder,'correction_experiment.mat'),'report','runs','correctedLateral','-v7.3');
    writetable(metrics,fullfile(outputFolder,'correction_metrics.csv'));
    writetable(lateralMetrics,fullfile(outputFolder,'lateral_metrics.csv'));
    writetable(vertcat(frameRows{:}),fullfile(outputFolder,'correction_frame_errors.csv'));
    fid=fopen(fullfile(outputFolder,'correction_summary.json'),'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));
    e=old.experiments{1};ix=e.uniformIndices;t=e.data.highRate.time(ix);
    vyRef=interp1(audit.time,audit.insVy,t);values=[t,vyRef];
    for v=1:4,values=[values,runs{1,v}.lateral.lateralVelocity(ix)];end %#ok<AGROW>
    writetable(array2table(values,VariableNames={'time','referenceVy','previousVy','steeringVy','biasLegacyVy','biasSteeringVy'}), ...
        fullfile(outputFolder,'lateral_comparison.csv'));
    fig=figure('Color','w','Position',[80,80,1200,850]);tiledlayout(fig,2,1,TileSpacing='compact');
    nexttile;plot(t,values(:,[2,3,4,6]));grid on;xlabel('Receiver time (s)');ylabel('Lateral velocity (m/s)');
    legend('INSPVA reference projection','Previous lateral output','Steering correction only','LiDAR motion correction',Location='best');
    title('Physical observer output versus effective mapped-reference velocity');
    nexttile;f=frames(frames.mode=="per_frame_zero",:);accepted=f.fullMeasurement==1;
    raw=sort(f.lidarMeasurementErrorM(accepted));plot(100*raw,(1:numel(raw))/numel(raw));hold on;
    for v=[1,2,4]
        err=sort(frameRows{1,v}.positionErrorM(accepted));plot(100*err,(1:numel(err))/numel(err));
    end
    grid on;xlabel('Position discrepancy at identical accepted frames (cm)');ylabel('Fraction of frames');
    legend('Original LiDAR','Previous observer','Steering correction only','Steering + LiDAR motion correction',Location='southeast');
    exportgraphics(fig,fullfile(outputFolder,'correction_comparison.png'),Resolution=160);
    exportgraphics(fig,fullfile(outputFolder,'correction_comparison.pdf'),ContentType='vector');
    disp(metrics(metrics.population=="native_full",:));
end
function values=score(pose,reference)
    e=vecnorm(pose(:,1:2)-reference(:,1:2),2,2);yaw=atan2(sin(pose(:,3)-reference(:,3)),cos(pose(:,3)-reference(:,3)));
    values=[rms(e),median(e),prctile(e,95),max(e),mean(e<=.05),mean(e<=.1),rad2deg(rms(yaw))];
end
