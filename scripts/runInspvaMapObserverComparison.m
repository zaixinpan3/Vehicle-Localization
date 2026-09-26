function report=runInspvaMapObserverComparison(outputFolder,options)
% runInspvaMapObserverComparison Add measured motion to fixed new-map LiDAR.
% Reuse all three per-frame matching diagnostics, without rematching or gain
% selection. Run the actual MnCAV lateral observer and seven-state observer.
% INSPVA is evaluation only within this replay; upstream matching used
% reference-relative seeds and an INSPVA-built map. No GNSS/INS position
% channel enters the observer itself.
% Gap interpolation is explicitly offline and identical for the comparison.
    arguments
        outputFolder (1,1) string="output/mncav_inspva_observer_20260915"
        options.MatchingFolder (1,1) string="output/saved_perception_inspva_20260915"
        options.MotionFile (1,1) string="output/mncav_zero_delay_20260914/experiment.mat"
        options.ReferenceFile (1,1) string="output/mncav_inspva_observer_20260915/native_reference.csv"
        options.MaximumOfflineGap (1,1) double {mustBePositive}=4
        options.Gains (1,4) double {mustBePositive}=[4,4,12,4]
    end
    setupVehicleLocalization();if ~isfolder(outputFolder),mkdir(outputFolder);end
    base=load(options.MotionFile,'data','uniformIndices','lateralCfg','lateral','report');
    assertMncavReplayCurrent(base.report.metadata.input.parameters);
    high=base.data.highRate;
    for name=string(fieldnames(high)).',high.(name)=high.(name)(base.uniformIndices);end
    lateralCfg=lateralObserverConfig("mncav");
    assert(isequaln(lateralCfg.vehicle,base.lateralCfg.vehicle),'VehicleLocalization:VehicleParameterMismatch', ...
        'Recorded motion preparation and lateral dynamics must use the same MnCAV parameters.');
    timer=tic;lateralDesign=designLateralObserverGains(lateralCfg);lateralSynthesisSeconds=toc(timer);
    timer=tic;lateral=runLateralVelocityObserver(high,lateralDesign,lateralCfg);lateralSeconds=toc(timer);
    lateralReproduction=max(abs(lateral.lateralVelocity-base.lateral.lateralVelocity));
    assert(lateralReproduction<1e-8,'VehicleLocalization:LateralReplayMismatch','Expected unchanged lateral replay.');
    cfg=motionAidedObserverConfig();cfg.gains=options.Gains;
    design=designMotionAidedObserverGains(cfg);
    adapter=improvedObserverConfig("lidar","mncav");
    adapter.measurement.fixedLidarDelay=0;adapter.lidar=cfg.lidar;
    calls=readtable(fullfile(options.MatchingFolder,'calls.csv'),TextType="string");
    stored=load(fullfile(options.MatchingFolder,'experiment.mat'),'reference','time');
    nativeReference=readtable(options.ReferenceFile);
    modes=["per_frame_zero","per_frame_positive","per_frame_negative"];
    metricRows=cell(0,11);caseReports=cell(3,1);experiments=cell(3,1);frameRows=cell(3,1);
    for j=1:numel(modes)
        current=calls(calls.mode==modes(j),:);
        assert(isequal(current.frame,(1:1170).'),'VehicleLocalization:IncompletePrecomputation','Every processed frame must be retained.');
        frameTime=stored.time;assert(max(abs(frameTime-current.time))<1e-9);
        accepted=current.fullPose==1;
        information=zeros(3,3,height(current));
        for k=1:height(current)
            c=current(k,:);
            information(:,:,k)=[c.informationXX,c.informationXY,c.informationXPsi; ...
                c.informationXY,c.informationYY,c.informationYPsi;c.informationXPsi,c.informationYPsi,c.informationPsiPsi];
        end
        originalPose=[current.measurementX,current.measurementY,current.measurementPsi];
        [data,lateralInput,reconstruction]=reconstructFrameAlignedLidarSignals(high,lateral, ...
            frameTime,frameTime(accepted),originalPose(accepted,:),information(:,:,accepted),adapter, ...
            MaximumOfflineGap=options.MaximumOfflineGap);
        t=data.highRate.time;frames=reconstruction.frameIndicesInIntegrationGrid;
        [covered,uniform]=ismember(high.time,t);assert(all(covered));
        timer=tic;estimate=runMotionAidedVehicleObserver(data,lateralInput,cfg);runtime=toc(timer);
        reference=interp1(nativeReference.time,[nativeReference.x,nativeReference.y,nativeReference.psi],t,'linear');
        assert(all(isfinite(reference),'all'),'Native INSPVA must cover the full trajectory.');
        % Use the exact previous frame reference at native times to reproduce
        % the 14.8634 cm LiDAR baseline, independent of native CSV roundoff.
        frameReference=stored.reference;
        poseDifference=data.lidar.pose(frames(accepted),:)-originalPose(accepted,:);
        poseDifference(:,3)=wrap(poseDifference(:,3));
        poseMismatch=max(abs(poseDifference),[],'all');
        infoMismatch=max(abs(data.lidar.information(:,:,frames(accepted))-information(:,:,accepted)),[],'all');
        assert(poseMismatch<1e-10 && infoMismatch<1e-10,'VehicleLocalization:FrameInjectionMismatch', ...
            'Every accepted LiDAR measurement must be preserved at its timestamp.');
        populations={"native_full",frames(accepted),frameReference(accepted,:); ...
            "native_all",frames,frameReference;"uniform_all",uniform,reference(uniform,:); ...
            "uniform_first60",uniform(t(uniform)<=60),reference(uniform(t(uniform)<=60),:); ...
            "uniform_after60",uniform(t(uniform)>60),reference(uniform(t(uniform)>60),:); ...
            "native_without_full_measurement",frames(~accepted),frameReference(~accepted,:)};
        for k=1:size(populations,1)
            ix=populations{k,2};ref=populations{k,3};
            versions={"lidar_linear_reconstruction",data.lidar.pose(ix,:);"motion_aided_observer",estimate.pose(ix,:)};
            if populations{k,1}=="native_full",versions{1,1}="lidar_measurement";versions{1,2}=originalPose(accepted,:);end
            for m=1:2
                values=score(versions{m,2},ref);
                metricRows(end+1,:)=[{modes(j),populations{k,1},versions{m,1},numel(ix)},num2cell(values)]; %#ok<AGROW>
            end
        end
        numerical=struct();
        if j==1
            repeated=runMotionAidedVehicleObserver(data,lateralInput,cfg);
            numerical.repeatedStateDifference=max(abs(repeated.z-estimate.z),[],'all');
            refinedCfg=cfg;refinedCfg.maximumIntegrationStep=cfg.maximumIntegrationStep/2;
            refined=runMotionAidedVehicleObserver(data,lateralInput,refinedCfg);
            numerical.halfStepMaximumPositionDifferenceM=max(vecnorm(refined.position-estimate.position,2,2));
            assert(numerical.repeatedStateDifference==0 && numerical.halfStepMaximumPositionDifferenceM<1e-5);
        end
        caseReports{j}=struct('mode',modes(j),'processedFrames',height(current),'fullMeasurements',nnz(accepted), ...
            'directionalFrames',nnz(current.directionalPose),'rejectedFrames',nnz(current.measurementType=="rejected"), ...
            'maximumPoseInjectionMismatch',poseMismatch,'maximumInformationInjectionMismatch',infoMismatch, ...
            'reconstruction',reconstruction,'runtimeSeconds',runtime,'numericalChecks',numerical,'diagnostics',estimate.diagnostics);
        experiments{j}=struct('data',data,'lateralInput',lateralInput,'estimate',estimate,'reference',reference, ...
            'frameReference',frameReference,'frameIndices',frames,'uniformIndices',uniform);
        frameRows{j}=table(current.frame,frameTime,repmat(modes(j),height(current),1),accepted, ...
            vecnorm(originalPose(:,1:2)-frameReference(:,1:2),2,2), ...
            vecnorm(estimate.position(frames,:)-frameReference(:,1:2),2,2), ...
            vecnorm(data.lidar.pose(frames,1:2)-frameReference(:,1:2),2,2), ...
            'VariableNames',{'frame','time','mode','fullMeasurement','lidarMeasurementErrorM','observerErrorM','lidarReconstructionErrorM'});
        fprintf('%s: %d full measurements, maximum gap %.3f s, observer %.2f s.\n',modes(j),nnz(accepted), ...
            reconstruction.maximumOriginalPoseGapSeconds,runtime);
    end
    metrics=cell2table(metricRows,VariableNames={'mode','population','method','samples', ...
        'positionRmseM','positionMedianM','positionP95M','positionMaximumM','fractionAtMost5cm','fractionAtMost10cm','headingRmseDeg'});
    frames=vertcat(frameRows{:});
    metadata=struct('matchingFolder',options.MatchingFolder,'motionFile',options.MotionFile,'referenceFile',options.ReferenceFile, ...
        'fixedGlobalGains',cfg.gains,'gainsRetuned',false,'vehicle',lateralCfg.vehicle, ...
        'parameters',base.report.metadata.input.parameters, ...
        'lateralSynthesisSeconds',lateralSynthesisSeconds,'lateralRuntimeSeconds',lateralSeconds, ...
        'lateralCertified',lateralDesign.certified,'lateralReproductionDifference',lateralReproduction, ...
        'fixedLidarDelaySeconds',0,'measurementInformationScale',cfg.lidar.gainInformationScale, ...
        'motionInputs',"Recorded speed, steering and corrected IMU; actual MnCAV lateral observer supplies vy and betaDot", ...
        'gnssPositionInputUsed',false,'referenceInjected',false,'matchingRerun',false, ...
        'continuousBaseline',"Offline linear interpolation of the same full accepted poses; interpolated values are not additional LiDAR measurements", ...
        'directionalPolicy',"Full-information continuous observer: directional frames have no new full-pose knot; retain their evaluation times", ...
        'limitations',"Same-drive map and reference-seeded matching; existing calibrated motion and gains; gaps reconstructed using future poses offline; no independent or online-causal accuracy claim");
    report=struct('metadata',metadata,'cases',vertcat(caseReports{:}),'metrics',metrics);
    save(fullfile(outputFolder,'experiment.mat'),'report','cfg','design','lateralCfg','lateralDesign','lateral','high','experiments','-v7.3');
    writetable(metrics,fullfile(outputFolder,'metrics.csv'));writetable(frames,fullfile(outputFolder,'frame_errors.csv'));
    writetable(array2table([lateral.time,lateral.lateralVelocity,lateral.sideSlipAngle,lateral.sideSlipAngleRate], ...
        VariableNames={'time','lateralVelocity','sideSlipAngle','sideSlipAngleRate'}),fullfile(outputFolder,'lateral.csv'));
    writeJson(fullfile(outputFolder,'summary.json'),report);
    writeJson(fullfile(outputFolder,'gains.json'),struct('global',design,'lateralGains',lateralDesign.gains, ...
        'lateralH2Bound',lateralDesign.h2Bound,'lateralMaxCertificateMargin',lateralDesign.maxCertificateMargin));
    e=experiments{1};t=e.data.highRate.time;ix=e.uniformIndices;
    continuousLidar=vecnorm(e.data.lidar.pose(ix,1:2)-e.reference(ix,1:2),2,2);
    continuousObserver=vecnorm(e.estimate.position(ix,:)-e.reference(ix,1:2),2,2);
    writetable(table(t(ix),continuousLidar,continuousObserver,VariableNames={'time','lidarReconstructionErrorM','observerErrorM'}), ...
        fullfile(outputFolder,'uniform_errors_zero.csv'));
    fig=figure('Color','w','Name','INSPVA-map observer comparison','Position',[100,100,1250,850]);
    tiledlayout(fig,2,1,TileSpacing='compact');
    nexttile;plot(t(ix),continuousLidar);hold on;plot(t(ix),continuousObserver);grid on;
    xlabel('Receiver time (s)');ylabel('Position discrepancy from INSPVA (m)');
    title('Same offline LiDAR reconstruction, zero-seed matching input');legend('LiDAR interpolation','Motion-aided observer');
    nexttile;mask=frames.mode==modes(1)&frames.fullMeasurement;
    rawError=sort(frames.lidarMeasurementErrorM(mask));observerError=sort(frames.observerErrorM(mask));
    plot(100*rawError,(1:numel(rawError))/numel(rawError));hold on;
    plot(100*observerError,(1:numel(observerError))/numel(observerError));grid on;
    xlabel('Position discrepancy at full accepted LiDAR frames (cm)');ylabel('Fraction of frames');
    legend('Original LiDAR measurement','Observer at the same timestamps',Location='southeast');
    exportgraphics(fig,fullfile(outputFolder,'comparison.png'),'Resolution',160);
    exportgraphics(fig,fullfile(outputFolder,'comparison.pdf'),'ContentType','vector');
    disp(metrics(metrics.population=="native_full"|metrics.population=="uniform_all",:));
end

function values=score(pose,reference)
    error=vecnorm(pose(:,1:2)-reference(:,1:2),2,2);yaw=wrap(pose(:,3)-reference(:,3));
    values=[rms(error),median(error),prctile(error,95),max(error),mean(error<=.05),mean(error<=.1),rad2deg(rms(yaw))];
end
function value=wrap(value)
    value=atan2(sin(value),cos(value));
end
function writeJson(path,value)
    fid=fopen(path,'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(value,PrettyPrint=true));
end
