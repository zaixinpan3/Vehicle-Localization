function report=runMncavFullObserverExperiment(outputFolder)
% runMncavFullObserverExperiment Exercise simultaneous and missing sources.
% Recorded ODOM XY supplies the GNSS/INS aiding channel, not pure GNSS.
% INSPVA positions enter scoring only; frozen matching/map remain reference
% assisted. No matching is rerun and no future pose interpolation is used.
    arguments
        outputFolder (1,1) string="output/full_observer_20260916"
    end
    setupVehicleLocalization();if ~isfolder(outputFolder),mkdir(outputFolder);end
    sensorFolder="output/mississippi_20240607_120931_20260907/sensors";
    parameterFile="output/mncav_interface_audit_20260916/vehicle_parameters.json";
    [prepared,~,inputMetadata]=prepareMncavObserverReplay(sensorFolder,parameterFile,table(),0);
    inputMetadata.reference="Legacy ODOM reference returned by exporter is discarded; INSPVA evaluation is supplied separately";
    h=prepared.highRate;t=h.time;
    prior=load('output/mncav_inspva_observer_20260915/experiment.mat','lateralDesign','experiments');
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
    folder='data/raw/Missisipi/gnss';stem='raw_data_2024-06-07-12-09-31_0';
    odom=readtable(fullfile(folder,[stem,'_odom.csv']));ins=readtable(fullfile(folder,[stem,'_inspva.csv']));
    poses=readFramePoseTable(fullfile(folder,[stem,'_front_lidar_pose_match_1_1170.csv']),1:1170);
    origin=ins.stamp_sec(1);receiver=ins.gps_seconds-ins.gps_seconds(1);
    bridge=@(stamp) interp1(ins.stamp_sec-origin,receiver,stamp-origin,'linear','extrap');
    time=bridge(odom.stamp_sec)-bridge(poses.lidar_stamp_sec(1));
    selected=time>=t(1) & time<=t(end);odom=odom(selected,:);time=time(selected);
    assert(isequal(time,prepared.gps.timestamp) && isequal([odom.x_m,odom.y_m],prepared.gps.pose));
    valid=isfinite(odom.pose_cov_xx) & isfinite(odom.pose_cov_yy) & odom.pose_cov_xx>0 & odom.pose_cov_yy>0;
    information=nan(2,2,numel(time));
    for k=find(valid).',information(:,:,k)=diag(1./[odom.pose_cov_xx(k),odom.pose_cov_yy(k)]);end
    gnss=struct('time',time,'position',[odom.x_m,odom.y_m], ...
        'information',information,'valid',valid,'delay',0);
    data=struct('highRate',h,'gnss',gnss,'lidar',lidar);
    cfg=fullObserverConfig();
    % Identical initial state in every ablation, including GNSS-only replay.
    old=load('output/mncav_interface_audit_20260916/correction_experiment.mat','runs');
    baseline=old.runs{1,4}.estimate;
    cfg.initialState=baseline.z(1,:).';
    [exists,ix]=ismember(t,baseline.time);assert(all(exists));
    baselinePose=baseline.pose(ix,:);
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
    report=struct('metadata',struct('gnssSource',"/novatel/oem7/odom XY; diagonal recorded pose covariance; GNSS/INS aiding, not pure GNSS", ...
        'lidarSource',"Frozen INSPVA-map per_frame_zero full-pose measurements", ...
        'inputMetadata',inputMetadata,'evaluation',"Native INSPVA on identical 100 Hz timestamps", ...
        'matchingRerun',false,'referencePositionInput',false,'zeroProcessingDelay',true, ...
        'initialization',"Common prior LiDAR/motion initial state in all ablations; not a cold-start GNSS-only test", ...
        'limitations',"Same-drive map and per-frame INSPVA matching seeds; shared receiver reference; unknown physical output-point transform; upstream motion preparation offline"), ...
        'design',designFullObserverGains(cfg),'metrics',metrics,'legacyOfflineMetrics',score(baselinePose,reference));
    save(fullfile(outputFolder,'experiment.mat'),'data','lateral','lateralDesign','cfg','runs','reference','baselinePose','report','-v7.3');
    writetable(metrics,fullfile(outputFolder,'metrics.csv'));
    fid=fopen(fullfile(outputFolder,'summary.json'),'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));
    fig=figure('Visible','off','Color','w','Position',[100,100,1250,800]);tiledlayout(2,1);
    nexttile;hold on;names=["both","lidar_only","gnss_only"];
    for k=1:3,plot(t,vecnorm(runs{k}.estimate.position-reference(:,1:2),2,2),DisplayName=names(k));end
    plot(t,vecnorm(baselinePose(:,1:2)-reference(:,1:2),2,2),'k:',DisplayName='previous offline baseline');
    ylabel('Position discrepancy (m)');xlabel('Receiver time (s)');grid on;
    legend(Interpreter='none',FontName='DejaVu Sans',FontSize=9,NumColumns=1,Position=[.72,.77,.25,.15]);
    title('Same recorded measurements; sampled dual-source runtime');
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
