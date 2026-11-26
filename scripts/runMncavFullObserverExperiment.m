function report=runMncavFullObserverExperiment(outputFolder,options)
% runMncavFullObserverExperiment Replay the recorded BESTPOS, HEADING2, motion and LiDAR measurements.
% Coarse source horizons are built from raw scans and measured wheel/gyro/lateral
% motion. No registration replay, optimized pose or registration acceptance is
% required. Historical motion caches without lateral validity are rejected.
% BESTPOS is INS aided; scoring uses the corrected same-drive receiver reference.
% Initialization retains one declared reference pose plus (0.5,-0.4 m,2 deg).
    arguments
        outputFolder (1,1) string="output/mncav_overlap_localization/observer"
        options.MatchingFolder (1,1) string=""
        options.CoarseSourceFile (1,1) string=""
        options.MapFile (1,1) string=""
        options.FrameCalibration (1,1) struct=struct()
    end
    setupVehicleLocalization();
    if ~isfolder(outputFolder),mkdir(outputFolder);end
    overlapConfig=overlapGradientConfig();
    sensorFolder="output/mncav_wheel_only_20260916/sensors";
    [prepared,inputMetadata]=prepareMncavMotionInputs(sensorFolder);
    h=prepared.highRate;
    lateralDesign=designLateralObserverGains(lateralObserverConfig());
    lateral=runLateralVelocityObserver(h,lateralDesign,lateralObserverConfig());
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
    localizationSupport.assertSensorOnlyLidarTilt(sourceCache.tilt);
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
    % Decode independent baseline azimuth directly from its receiver log.
    [status,raw]=system('uv run --offline --quiet --with numpy --with rosbags --with pyproj python scripts/readMncavGnssHeading.py');
    assert(status==0,'VehicleLocalization:MissingGnssHeading','Cannot read native HEADING2: %s',raw);
    receiverHeading=jsondecode(raw);
    assert(string(receiverHeading.clockModelId)==prepared.clockModelId,'VehicleLocalization:ClockMismatch','HEADING2 clock mismatch.');
    data=struct('highRate',h,'gnss',gnss,'gnssHeading',receiverHeading,'lidarOverlap',struct('time',calls.timeSeconds, ...
        'map',sourceCache.fixed,'sources',{sourceCache.sources},'valid',true(n,1),'config',overlapConfig));
    [~,cfg]=synthesizeMncavInvariantDesign(mncavFullObserverConfig());
    [data,lateral,synchronization]=localizationSupport.synchronizeLocalizationInputs(data,lateral,cfg);
    h=data.highRate;t=h.time;n=numel(t);
    [covered,sourceIndex]=ismember(t,calls.timeSeconds);
    assert(all(covered),'VehicleLocalization:CoarseCacheMismatch','Source clocks must cover every aligned frame.');
    sourceClouds=sourceCache.sources(sourceIndex);
    pose=sourceCache.initialPose;rotation=[cos(pose(3)),-sin(pose(3));sin(pose(3)),cos(pose(3))];
    velocity=rotation*[h.longitudinalSpeed(1);lateral.lateralVelocity(1)];
    cfg.initialState=[pose(1);velocity(1);0;pose(2);velocity(2);0;pose(3)];
    data.lidarOverlap=struct('map',sourceCache.fixed,'sources',{sourceClouds},'valid',true(n,1),'config',overlapConfig);
    native=readtable('output/receiver_reference/corrected_native_reference.csv');
    reference=interp1(native.time,[native.x,native.y,native.psi],t,'linear');
    timer=tic;
    estimate=runFullLocalizationObserver(data,cfg,LateralInputs=lateral);
    seconds=toc(timer);
    runs={struct('scenario',"both",'estimate',estimate,'seconds',seconds)};
    d=estimate.diagnostics;
    gnssActive=d.gnssPositionValid | d.independentHeadingValid;
    errors=score(estimate.pose,reference);
    metrics=cell2table([{"both","full",n},num2cell(errors), ...
        {nnz(gnssActive & d.lidarValid),nnz(~gnssActive & ~d.lidarValid),seconds}], ...
        VariableNames={'scenario','population','samples','positionRmseM', ...
        'positionMedianM','positionP95M','positionMaximumM','fractionAtMost10cm','headingRmseDeg', ...
        'bothActiveSamples','neitherActiveSamples','runtimeSeconds'});
    report=struct('metadata',struct('gnssSource',"BESTPOS position and available HEADING2 heading in UTM 15N", ...
        'lidarSource',"Fresh measured-motion coarse horizons and overlap gradient", ...
        'headingSource',rmfield(receiverHeading,{'time','heading','valid'}),'inputMetadata',inputMetadata,'synchronization',synchronization, ...
        'evaluation',"corrected same-drive receiver reference on native LiDAR timestamps", ...
        'lidarChannel',"overlapGradient",'overlapFormulation',"exact single-scale Cauchy-Schwarz", ...
        'currentScanOnlyFrames',nnz(sourceCache.motionSegment(sourceIndex)==0), ...
        'velocityAidingFrames',nnz(lateral.velocityValid),'sourceMap',sourceCache.sourceMap,'sourcePreparationSeconds',sum(sourceCache.seconds), ...
        'gnssInformationAddedToLidar',false,'referencePositionInput',false,'zeroLidarProcessingDelay',true, ...
        'offlineSynchronization',true,'gnssOutputPointCalibration',cfg.gnss.outputPoint, ...
        'initialization',"One reference pose plus declared offset, wheel/lateral velocity; acceleration and internal yaw rate start at zero", ...
        'limitations',"Same-drive map and reference; INS-aided BESTPOS correlated with reference; offline motion reconstruction"), ...
        'design',runs{1}.estimate.observer,'metrics',metrics);
    accepted=runs{1}.estimate.diagnostics.lidarOverlap.available;
    % Old timestamp-warped trajectories are not paired with corrected epochs.
    % Receiver transport changes the measured point while sharing the same metric.
    zeroPointCfg=cfg;zeroPointCfg.gnss.outputPoint.bodyOffset=[0;0];
    controlRows=cell(0,8);controlNames=["unaligned","uncertainty_only","geometry_only","aligned"];
    for k=1:numel(controlNames)
        controlCfg=cfg;
        if ismember(controlNames(k),["unaligned","uncertainty_only"])
            controlCfg=zeroPointCfg;
        end
        if ismember(controlNames(k),["unaligned","geometry_only"])
            controlCfg.gnss.outputPoint.bodyCovariance=zeros(2);controlCfg.gnss.outputPoint.headingStdRad=0;
        end
        control=runFullLocalizationObserver(data,controlCfg,LateralInputs=lateral);
        controlRows(end+1,:)=[{controlNames(k)},num2cell(score(control.pose,reference)), ...
            {rms(vecnorm(control.position(accepted,:)-reference(accepted,1:2),2,2))}]; %#ok<AGROW>
    end
    report.alignmentControls=cell2table(controlRows,VariableNames={'method','positionRmseM', ...
        'positionMedianM','positionP95M','positionMaximumM','fractionAtMost10cm','headingRmseDeg','acceptedPositionRmseM'});
    report.alignmentControlGainPolicy="One four-vertex design shared by all receiver geometry controls";
    writetable(report.alignmentControls,fullfile(outputFolder,'alignment_controls.csv'));
    wheel=prepared.wheelVelocity;
    save(fullfile(outputFolder,'experiment.mat'),'data','lateral','lateralDesign','cfg','runs','reference','wheel','report','bestpos','-v7.3');
    writetable(metrics,fullfile(outputFolder,'metrics.csv'));
    fid=fopen(fullfile(outputFolder,'summary.json'),'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));
    fig=figure('Visible','off','Color','w','Position',[100,100,1250,800]);tiledlayout(2,1);
    nexttile;plot(t,vecnorm(estimate.position-reference(:,1:2),2,2));
    ylabel('Position discrepancy (m)');grid on;
    title('Recorded GNSS, motion and LiDAR');
    nexttile;angle=estimate.pose(:,3)-reference(:,3);
    plot(t,rad2deg(atan2(sin(angle),cos(angle))));
    ylabel('Heading discrepancy (deg)');xlabel('Receiver time (s)');grid on;
    exportgraphics(fig,fullfile(outputFolder,'comparison.png'),Resolution=160);close(fig);
    disp(metrics(metrics.population=="full",:));
end

function m=score(pose,reference)
    if isempty(pose),m=nan(1,6);return;end
    e=vecnorm(pose(:,1:2)-reference(:,1:2),2,2);a=atan2(sin(pose(:,3)-reference(:,3)),cos(pose(:,3)-reference(:,3)));
    m=[rms(e),median(e),prctile(e,95),max(e),mean(e<=.1),rad2deg(rms(a))];
end
