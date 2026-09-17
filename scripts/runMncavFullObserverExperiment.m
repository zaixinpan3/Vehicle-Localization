function report=runMncavFullObserverExperiment(outputFolder)
% runMncavFullObserverExperiment Exercise simultaneous and missing sources.
% Recorded BESTPOS XY supplies the receiver position channel, still INS aided.
% Evaluation-drive INSPVA positions enter scoring only; a separate drive
% calibrates the relative receiver output point. Frozen matching/map remain reference
% assisted. Matching is frozen; frame alignment interpolates real GNSS samples
% and reports the required future-endpoint wait, without motion extrapolation.
    arguments
        outputFolder (1,1) string="output/mncav_bestpos_alignment_20260917"
    end
    setupVehicleLocalization();if ~isfolder(outputFolder),mkdir(outputFolder);end
    sensorFolder="output/mncav_wheel_only_20260916/sensors";
    parameterFile="output/mncav_interface_audit_20260916/vehicle_parameters.json";
    [prepared,~,inputMetadata]=prepareMncavObserverReplay(sensorFolder,parameterFile,table(),0,IncludeOdom=false);
    h=prepared.highRate;t=h.time;
    prior=load('output/mncav_inspva_observer_20260915/experiment.mat','lateralDesign');
    lateralDesign=prior.lateralDesign;
    lateral=runLateralVelocityObserver(h,lateralDesign,lateralDesign.cfg);
    calls=readtable('output/saved_perception_inspva_20260915/calls.csv',TextType="string");
    calls=calls(calls.mode=="per_frame_zero" & calls.time<=t(end),:);
    n=height(calls);information=zeros(3,3,n);
    for k=1:n
        c=calls(k,:);information(:,:,k)=[c.informationXX,c.informationXY,c.informationXPsi; ...
            c.informationXY,c.informationYY,c.informationYPsi;c.informationXPsi,c.informationYPsi,c.informationPsiPsi];
    end
    lidar=struct('time',calls.time,'pose',[calls.measurementX,calls.measurementY,calls.measurementPsi], ...
        'information',information,'valid',logical(calls.fullPose),'delay',0);
    bestpos=readtable('output/mncav_synchronous_bestpos_20260917/bestpos.csv');
    time=bestpos.time;valid=logical(bestpos.valid);
    information=nan(2,2,numel(time));
    for k=find(valid).'
        information(:,:,k)=[bestpos.informationXX(k),bestpos.informationXY(k);bestpos.informationXY(k),bestpos.informationYY(k)];
    end
    gnss=struct('time',time,'position',[bestpos.x,bestpos.y], ...
        'information',information,'valid',valid,'delay',0);
    data=struct('highRate',h,'gnss',gnss,'lidar',lidar);
    cfg=mncavFullObserverConfig();
    [data,lateral,synchronization]=synchronizeLocalizationInputs(data,lateral,cfg);
    h=data.highRate;t=h.time;gnss=data.gnss;lidar=data.lidar;time=gnss.time;valid=gnss.valid;
    % Identical initial state in every ablation, including GNSS-only replay.
    assert(lidar.valid(1) && lidar.time(1)==t(1),'Initial LiDAR pose is required.');
    pose=lidar.pose(1,:);rotation=[cos(pose(3)),-sin(pose(3));sin(pose(3)),cos(pose(3))];
    velocity=rotation*[h.longitudinalSpeed(1);lateral.lateralVelocity(1)];
    acceleration=rotation*[h.longitudinalAcceleration(1);h.lateralAcceleration(1)];
    cfg.initialState=[pose(1);velocity(1);acceleration(1);pose(2);velocity(2);acceleration(2);pose(3)];
    native=readtable('output/mncav_inspva_observer_20260915/native_reference.csv');
    reference=interp1(native.time,[native.x,native.y,native.psi],t,'linear');
    scenarios=["both","lidar_only","gnss_only","gnss_outage","lidar_outage","both_outage","alternating"];
    runs=cell(numel(scenarios),1);rows=cell(0,12);
    for k=1:numel(scenarios)
        current=data;name=scenarios(k);
        if name=="lidar_only",current=rmfield(current,'gnss');end
        if name=="gnss_only",current=rmfield(current,'lidar');end
        if ismember(name,["gnss_outage","both_outage"])
            current.gnss.valid(time>=40 & time<60)=false;
        end
        if ismember(name,["lidar_outage","both_outage"])
            current.lidar.valid(lidar.time>=40 & lidar.time<60)=false;
        end
        if name=="alternating"
            current.gnss.valid=valid & mod(floor(time),2)==0;
            current.lidar.valid=lidar.valid & mod(floor(lidar.time),2)==1;
        end
        timer=tic;estimate=runFullLocalizationObserver(current,lateralDesign,cfg,LateralInputs=lateral);seconds=toc(timer);
        runs{k}=struct('scenario',name,'estimate',estimate,'seconds',seconds);
        populations={"full",true(size(t));"outage_40_60",t>=40 & t<60;"recovery_60_70",t>=60 & t<70};
        for j=1:size(populations,1)
            mask=populations{j,2};metrics=score(estimate.pose(mask,:),reference(mask,:));
            rows(end+1,:)=[{name,populations{j,1},nnz(mask)},num2cell(metrics), ...
                {nnz(estimate.diagnostics.mode(mask)==3),nnz(estimate.diagnostics.mode(mask)==0),seconds}]; %#ok<AGROW>
        end
        fprintf('%s: position %.5f m, heading %.4f deg, %.2f s.\n',name, ...
            rows{end-2,4},rows{end-2,9},seconds);
    end
    metrics=cell2table(rows,VariableNames={'scenario','population','samples','positionRmseM', ...
        'positionMedianM','positionP95M','positionMaximumM','fractionAtMost10cm','headingRmseDeg', ...
        'bothActiveSamples','neitherActiveSamples','runtimeSeconds'});
    report=struct('metadata',struct('gnssSource',"/novatel/oem7/bestpos XY; reported uncertainty projected to UTM; recorded solution types are INS aided", ...
        'lidarSource',"Frozen INSPVA-map per_frame_zero full-pose measurements", ...
        'inputMetadata',inputMetadata,'synchronization',synchronization, ...
        'evaluation',"INSPVA on native LiDAR frame timestamps, approximately 10 Hz", ...
        'matchingRerun',false,'referencePositionInput',false,'zeroLidarProcessingDelay',true, ...
        'offlineSynchronization',true, ...
        'gnssOutputPointCalibration',cfg.gnss.outputPoint, ...
        'initialization',"Common first LiDAR pose plus wheel/lateral velocity and IMU acceleration; not a cold-start GNSS-only test", ...
        'limitations',"Same-drive map and per-frame INSPVA matching seeds; shared receiver reference; empirical planar output-point alignment is not a hardware survey; upstream motion preparation offline"), ...
        'design',runs{1}.estimate.observer,'metrics',metrics);
    old=load('output/mncav_wheel_only_20260916/calibration/experiment.mat','atNative');
    [found,index]=ismember(t,old.atNative.time);assert(all(found),'Historical comparison needs exact common timestamps.');
    accepted=lidar.valid;paired=cell(0,9);
    baseline=load('output/mncav_synchronous_bestpos_20260917/experiment.mat','runs');
    assert(isequal(baseline.runs{1}.estimate.time,t),'Unaligned comparison needs identical frame times.');
    labels=["aligned_bestpos_10hz","unaligned_bestpos_10hz","historical_transport_100hz","raw_lidar"];
    posesToCompare={runs{1}.estimate.pose,baseline.runs{1}.estimate.pose,old.atNative.pose(index,:),lidar.pose};
    for k=1:numel(labels)
        paired(end+1,:)=[{labels(k),"accepted_lidar_frames",nnz(accepted)},num2cell(score(posesToCompare{k}(accepted,:),reference(accepted,:)))]; %#ok<AGROW>
    end
    report.pairedComparison=cell2table(paired,VariableNames={'method','population','samples','positionRmseM', ...
        'positionMedianM','positionP95M','positionMaximumM','fractionAtMost10cm','headingRmseDeg'});
    writetable(report.pairedComparison,fullfile(outputFolder,'paired_comparison.csv'));
    % Isolate geometry/uncertainty at the old gain before testing matched gains.
    controlRows=cell(0,8);controlNames=["unaligned","uncertainty_only","geometry_only","aligned_gain1","aligned_gain4"];
    for k=1:numel(controlNames)
        controlCfg=cfg;
        if controlNames(k)~="aligned_gain4",controlCfg.gnss.positionGain=1;end
        if ismember(controlNames(k),["unaligned","uncertainty_only"])
            controlCfg.gnss.outputPoint.bodyOffset=[0;0];
        end
        if ismember(controlNames(k),["unaligned","geometry_only"])
            controlCfg.gnss.outputPoint.bodyCovariance=zeros(2);controlCfg.gnss.outputPoint.headingStdRad=0;
        end
        control=runFullLocalizationObserver(data,lateralDesign,controlCfg,LateralInputs=lateral);
        controlRows(end+1,:)=[{controlNames(k)},num2cell(score(control.pose,reference)), ...
            {rms(vecnorm(control.position(accepted,:)-reference(accepted,1:2),2,2))}]; %#ok<AGROW>
    end
    report.alignmentControls=cell2table(controlRows,VariableNames={'method','positionRmseM', ...
        'positionMedianM','positionP95M','positionMaximumM','fractionAtMost10cm','headingRmseDeg','acceptedPositionRmseM'});
    writetable(report.alignmentControls,fullfile(outputFolder,'alignment_controls.csv'));
    wheel=prepared.wheelVelocity;
    save(fullfile(outputFolder,'experiment.mat'),'data','lateral','lateralDesign','cfg','runs','reference','wheel','report','bestpos','-v7.3');
    writetable(metrics,fullfile(outputFolder,'metrics.csv'));
    fid=fopen(fullfile(outputFolder,'summary.json'),'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));
    fig=figure('Visible','off','Color','w','Position',[100,100,1250,800]);tiledlayout(2,1);
    nexttile;hold on;names=["both","lidar_only","gnss_only"];
    for k=1:3,plot(t,vecnorm(runs{k}.estimate.position-reference(:,1:2),2,2),DisplayName=names(k));end
    ylabel('Position discrepancy (m)');xlabel('Receiver time (s)');grid on;
    legend(Interpreter='none',FontName='DejaVu Sans',FontSize=9,NumColumns=1,Position=[.72,.77,.25,.15]);
    title('Synchronized localization at native LiDAR frame times');
    nexttile;hold on;
    for k=4:6,plot(t,vecnorm(runs{k}.estimate.position-reference(:,1:2),2,2),DisplayName=scenarios(k));end
    xline(40,'k:',HandleVisibility='off');xline(60,'k:',HandleVisibility='off');
    ylabel('Position discrepancy (m)');xlabel('Receiver time (s)');
    legend(Interpreter='none',FontName='DejaVu Sans',FontSize=9,NumColumns=1,Position=[.72,.28,.25,.12]);
    grid on;title('Declared channel withdrawals from 40 to 60 seconds');
    exportgraphics(fig,fullfile(outputFolder,'comparison.png'),Resolution=160);close(fig);
    disp(metrics(metrics.population=="full",:));
end

function m=score(pose,reference)
    e=vecnorm(pose(:,1:2)-reference(:,1:2),2,2);a=atan2(sin(pose(:,3)-reference(:,3)),cos(pose(:,3)-reference(:,3)));
    m=[rms(e),median(e),prctile(e,95),max(e),mean(e<=.1),rad2deg(rms(a))];
end
