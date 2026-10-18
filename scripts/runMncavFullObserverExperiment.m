function report=runMncavFullObserverExperiment(outputFolder,options)
% runMncavFullObserverExperiment Current overlap-gradient localization scenarios.
% Coarse source horizons are built from raw scans and measured wheel/gyro/lateral
% motion. No registration replay, optimized pose or registration acceptance is
% required. Historical motion caches without lateral validity are rejected.
% BESTPOS is INS aided; scoring uses the corrected same-drive receiver reference.
% Initialization retains one declared reference pose plus (0.5,-0.4 m,2 deg).
    arguments
        outputFolder (1,1) string="output/mncav_overlap_localization/observer"
        options.MatchingFolder (1,1) string=""
        options.CoarseSourceFile (1,1) string=""
        options.LidarChannel (1,1) string="overlapGradient"
        options.MapFile (1,1) string=""
        options.FrameCalibration (1,1) struct=struct()
        options.OverlapScaleLadder (1,:) double {mustBeNonnegative,mustBeFinite}=[]
    end
    setupVehicleLocalization();options.LidarChannel=activeLocalizationChannel(options.LidarChannel);
    if ~isfolder(outputFolder),mkdir(outputFolder);end
    overlapConfig=overlapGradientConfig();
    if ~isempty(options.OverlapScaleLadder),overlapConfig.scaleLadder=options.OverlapScaleLadder;end
    sensorFolder="output/mncav_wheel_only_20260916/sensors";
    [prepared,~,inputMetadata]=prepareMncavObserverReplay(sensorFolder,"",table(),0,IncludeOdom=false);
    h=prepared.highRate;
    lateralDesign=designLateralObserverGains(lateralObserverConfig());
    lateral=runLateralVelocityObserver(h,lateralDesign,lateralObserverConfig("mncav"));
    motion=struct('time',h.time,'longitudinalSpeed',h.longitudinalSpeed, ...
        'lateralVelocity',lateral.lateralVelocity,'yawRate',h.yawRate,'clockModelId',prepared.clockModelId, ...
        'velocityValid',lateral.velocityValid);
    if strlength(options.CoarseSourceFile)>0
        sourceCache=load(options.CoarseSourceFile);
    else
        sourceCache=prepareMississippiLocalizationClouds(options.MatchingFolder, ...
            MotionInputs=motion,MapFile=options.MapFile,FrameCalibration=options.FrameCalibration);
    end
    assert(all(isfield(sourceCache,{'tilt','initialPose','sourceMap','motionSegment','motionValidityPolicy'})) && ...
        sourceCache.motionValidityPolicy=="uninterrupted-valid-motion-v1", ...
        'VehicleLocalization:CoarseCacheMismatch','Regenerate the source cache with current provenance and initialization.');
    lidarCalibrationSupport.assertSensorOnlyLidarTilt(sourceCache.tilt);
    assert(isequaln(sourceCache.cfg.sourceWindow,localizationSourceWindowConfig()) && ...
        string(sourceCache.clockModelId)==prepared.clockModelId, ...
        'VehicleLocalization:CoarseCacheMismatch','Coarse horizon configuration or clock is stale.');
    calls=sourceCache.calls;n=height(calls);
    bestMetadata=jsondecode(fileread('output/receiver_synchronized_inputs/bestpos_metadata.json'));
    referenceMetadata=jsondecode(fileread('output/receiver_reference/reference_metadata.json'));
    assert(string(bestMetadata.clock.modelId)==prepared.clockModelId && ...
        string(referenceMetadata.clock.modelId)==prepared.clockModelId, ...
        'VehicleLocalization:ClockMismatch','BESTPOS, reference and motion clocks must agree.');
    bestpos=readtable('output/receiver_synchronized_inputs/bestpos.csv');
    information=nan(2,2,height(bestpos));
    for k=find(logical(bestpos.valid)).'
        information(:,:,k)=[bestpos.informationXX(k),bestpos.informationXY(k);bestpos.informationXY(k),bestpos.informationYY(k)];
    end
    gnss=struct('time',bestpos.time,'position',[bestpos.x,bestpos.y], ...
        'information',information,'valid',logical(bestpos.valid),'delay',0);
    data=struct('highRate',h,'gnss',gnss,'lidarOverlap',struct('time',calls.timeSeconds, ...
        'map',sourceCache.fixed,'sources',{sourceCache.sources},'valid',true(n,1),'config',overlapConfig));
    [~,cfg]=synthesizeMncavInvariantDesign(mncavFullObserverConfig());
    [data,lateral,synchronization]=fullObserverSupport.synchronizeLocalizationInputs(data,lateral,cfg);
    h=data.highRate;t=h.time;n=numel(t);gnss=data.gnss;time=gnss.time;valid=gnss.valid;
    [covered,sourceIndex]=ismember(t,calls.timeSeconds);
    assert(all(covered),'VehicleLocalization:CoarseCacheMismatch','Source clocks must cover every aligned frame.');
    sourceClouds=sourceCache.sources(sourceIndex);
    pose=sourceCache.initialPose;rotation=[cos(pose(3)),-sin(pose(3));sin(pose(3)),cos(pose(3))];
    velocity=rotation*[h.longitudinalSpeed(1);lateral.lateralVelocity(1)];
    cfg.initialState=[pose(1);velocity(1);0;pose(2);velocity(2);0;pose(3)];
    data.lidarOverlap=struct('map',sourceCache.fixed,'sources',{sourceClouds},'valid',true(n,1),'config',overlapConfig);
    native=readtable('output/receiver_reference/corrected_native_reference.csv');
    reference=interp1(native.time,[native.x,native.y,native.psi],t,'linear');
    scenarios=["both","lidar_only","gnss_only","gnss_outage","lidar_outage","both_outage","alternating"];
    runs=cell(numel(scenarios),1);rows=cell(0,12);
    for k=1:numel(scenarios)
        current=data;name=scenarios(k);
        if name=="lidar_only",current=rmfield(current,'gnss');end
        if name=="gnss_only",current=rmfield(current,'lidarOverlap');end
        if ismember(name,["gnss_outage","both_outage"]),current.gnss.valid(time>=40 & time<60)=false;end
        if ismember(name,["lidar_outage","both_outage"]),current.lidarOverlap.valid(t>=40 & t<60)=false;end
        if name=="alternating"
            current.gnss.valid=valid & mod(floor(time),2)==0;
            current.lidarOverlap.valid=mod(floor(t),2)==1;
        end
        timer=tic;estimate=runFullLocalizationObserver(current,lateralDesign,cfg,LateralInputs=lateral);seconds=toc(timer);
        runs{k}=struct('scenario',name,'estimate',estimate,'seconds',seconds);
        populations={"full",true(size(t));"outage_40_60",t>=40 & t<60;"recovery_60_70",t>=60 & t<70};
        for j=1:size(populations,1)
            mask=populations{j,2};metrics=score(estimate.pose(mask,:),reference(mask,:));
            rows(end+1,:)=[{name,populations{j,1},nnz(mask)},num2cell(metrics), ...
                {nnz(estimate.diagnostics.mode(mask)==3),nnz(estimate.diagnostics.mode(mask)==0),seconds}]; %#ok<AGROW>
        end
        fprintf('%s: position %.5f m, heading %.4f deg, %.2f s.\n',name,rows{end-2,4},rows{end-2,9},seconds);
    end
    metrics=cell2table(rows,VariableNames={'scenario','population','samples','positionRmseM', ...
        'positionMedianM','positionP95M','positionMaximumM','fractionAtMost10cm','headingRmseDeg', ...
        'bothActiveSamples','neitherActiveSamples','runtimeSeconds'});
    report=struct('metadata',struct('gnssSource',"INS-aided BESTPOS XY; reported uncertainty projected to UTM", ...
        'lidarSource',"Fresh measured-motion coarse horizons and overlap gradient", ...
        'inputMetadata',inputMetadata,'synchronization',synchronization, ...
        'evaluation',"corrected same-drive receiver reference on native LiDAR timestamps", ...
        'lidarChannel',options.LidarChannel,'overlapScaleLadder',overlapConfig.scaleLadder, ...
        'registrationExecuted',false,'currentScanOnlyFrames',nnz(sourceCache.motionSegment(sourceIndex)==0), ...
        'velocityAidingFrames',nnz(lateral.velocityValid),'sourceMap',sourceCache.sourceMap,'sourcePreparationSeconds',sum(sourceCache.seconds), ...
        'gnssInformationAddedToLidar',false,'referencePositionInput',false,'zeroLidarProcessingDelay',true, ...
        'offlineSynchronization',true,'gnssOutputPointCalibration',cfg.gnss.outputPoint, ...
        'initialization',"One reference pose plus declared offset, wheel/lateral velocity; acceleration and internal yaw rate start at zero", ...
        'limitations',"Same-drive map and reference; INS-aided BESTPOS correlated with reference; offline motion reconstruction"), ...
        'design',runs{1}.estimate.observer,'metrics',metrics);
    accepted=runs{1}.estimate.diagnostics.lidarOverlap.available;
    % Old timestamp-warped trajectories are not paired with corrected epochs.
    % Each changed lever arm needs gains solved for its own geometry; runtime
    % uncertainty changes alone do not alter synthesis inputs.
    zeroPointCfg=cfg;zeroPointCfg.gnss.outputPoint.bodyOffset=[0;0];
    [~,zeroPointCfg]=synthesizeMncavInvariantDesign(zeroPointCfg);
    controlRows=cell(0,8);controlNames=["unaligned","uncertainty_only","geometry_only","aligned"];
    for k=1:numel(controlNames)
        controlCfg=cfg;
        if ismember(controlNames(k),["unaligned","uncertainty_only"])
            controlCfg=zeroPointCfg;
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
    report.alignmentControlGainPolicy="Fresh gains for each lever arm; shared within equal synthesis inputs";
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
    if isempty(pose),m=nan(1,6);return;end
    e=vecnorm(pose(:,1:2)-reference(:,1:2),2,2);a=atan2(sin(pose(:,3)-reference(:,3)),cos(pose(:,3)-reference(:,3)));
    m=[rms(e),median(e),prctile(e,95),max(e),mean(e<=.1),rad2deg(rms(a))];
end
