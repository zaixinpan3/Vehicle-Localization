function report=runMncavMotionAidedExperiment(outputFolder,options)
% runMncavMotionAidedExperiment Evaluate the new cascade on fixed real matching.
% Re-synthesize nominal MnCAV lateral gains and rerun its original inputs.
% Reuse all 1170 precomputed matching frames; no scan is rematched or dropped.
% Compare uniform times, raw frame outputs and accepted measurements separately.
% INSPVA and mixed ODOM are diagnostic references, not independent truth.
    arguments
        outputFolder (1,1) string="output/mncav_motion_aided_20260914"
        options.InputFile (1,1) string="output/mncav_zero_delay_20260914/experiment.mat"
        options.CallsFile (1,1) string="output/mncav_zero_delay_20260914/precomputed_calls.csv"
        options.DesignFile (1,1) string=""
    end
    root=setupVehicleLocalization;if ~isfolder(outputFolder),mkdir(outputFolder);end
    base=load(options.InputFile);calls=readtable(options.CallsFile);
    assert(isequal(calls.frame,(1:1170).'),'VehicleLocalization:IncompletePrecomputation','All processed frames are required.');
    cfg=motionAidedObserverConfig;
    if strlength(options.DesignFile)>0,chosen=load(options.DesignFile,'cfg');cfg=chosen.cfg;end
    design=designMotionAidedObserverGains(cfg);
    lateralCfg=lateralObserverConfig("mncav");lateralDesign=designLateralObserverGains(lateralCfg);
    high=base.data.highRate;
    for name=string(fieldnames(high)).',high.(name)=high.(name)(base.uniformIndices);end
    lateral=runLateralVelocityObserver(high,lateralDesign,lateralCfg);
    lateralDifference=max(abs(lateral.lateralVelocity-base.lateral.lateralVelocity));
    assert(lateralDifference<1e-8,'VehicleLocalization:LateralReplayMismatch','Frozen matching must retain its original lateral motion aid.');
    time=base.data.highRate.time;lateralInput=struct('time',time);
    query=min(max(time,high.time(1)),high.time(end));
    for name=["lateralVelocity","sideSlipAngle","sideSlipAngleRate"]
        lateralInput.(name)=interp1(high.time,lateral.(name),query,'linear');
    end
    timer=tic;estimate=runMotionAidedVehicleObserver(base.data,lateralInput,cfg);runSeconds=toc(timer);
    repeated=runMotionAidedVehicleObserver(base.data,lateralInput,cfg);
    reproductionError=max(abs(estimate.z-repeated.z),[],'all');assert(reproductionError==0);
    halfCfg=cfg;halfCfg.maximumIntegrationStep=cfg.maximumIntegrationStep/2;
    refined=runMotionAidedVehicleObserver(base.data,lateralInput,halfCfg);
    refinementError=max(abs(estimate.z-refined.z),[],'all');
    % Initialization control: original dynamics receive the same initial state.
    legacyCfg=base.cfg;legacyCfg.observer.initialState=estimate.z(1,:).';
    legacyMatched=runImprovedVehicleObserver(base.data,struct(),base.design,legacyCfg,LateralInputs=lateralInput);
    frameIndices=base.report.metadata.reconstruction.frameIndicesInIntegrationGrid;
    frameClockRoundTrip=max(abs(time(frameIndices)-calls.timeSeconds));
    assert(frameClockRoundTrip<1e-9,'VehicleLocalization:FrameClockMismatch', ...
        'The calls CSV must correspond to the exact stored frame knots.');
    rows={};uniform=base.uniformIndices;
    populations={"uniform_all",uniform;"uniform_design",uniform(time(uniform)<=60); ...
        "uniform_reserved",uniform(time(uniform)>60);"native_all",frameIndices; ...
        "native_accepted",frameIndices(calls.accepted==1)};
    for r=1:2
        reference=base.pvaPose;referenceName="INSPVA";
        if r==2,reference=base.referencePose;referenceName="mixed_ODOM";end
        for k=1:size(populations,1)
            ix=populations{k,2};baseline=base.data.lidar.pose(ix,:);
            if startsWith(populations{k,1},"native")
                native=[calls.x,calls.y,calls.psi];
                if populations{k,1}=="native_accepted",native=native(calls.accepted==1,:);end
                baseline=native;
            end
            versions={"map_matching",baseline;"previous_observer",base.estimate.pose(ix,:); ...
                "previous_same_initial",legacyMatched.pose(ix,:);"motion_aided",estimate.pose(ix,:)};
            for j=1:size(versions,1)
                m=metrics(versions{j,2},reference(ix,:));
                rows(end+1,:)=[{referenceName,populations{k,1},versions{j,1},numel(ix)},num2cell(m)]; %#ok<AGROW>
            end
        end
    end
    scores=cell2table(rows,VariableNames={'reference','population','method','samples', ...
        'positionRmseM','positionMaximumM','positionP95M','headingRmseDeg'});
    gates=true(2,5);
    for r=1:2
        refs=["INSPVA","mixed_ODOM"];
        for k=1:5
            mask=scores.reference==refs(r) & scores.population==populations{k,1};
            old=scores{mask & scores.method=="map_matching",5:8};
            new=scores{mask & scores.method=="motion_aided",5:8};gates(r,k)=all(new<=old+1e-10);
        end
    end
    summary=struct('gains',cfg.gains,'gainUnits',"1/s",'uniformSamples',numel(uniform), ...
        'processedFrames',height(calls),'acceptedFrames',nnz(calls.accepted), ...
        'fixedLidarDelaySeconds',0,'maximumCsvFrameTimeRoundTripSeconds',frameClockRoundTrip, ...
        'comparisonGateAllMetrics',gates, ...
        'gateRows',{{'INSPVA','mixed_ODOM'}},'gateColumns',string(populations(:,1)).', ...
        'allComparisonGatesPassed',all(gates,'all'),'lateralGainSynthesisCertified',lateralDesign.certified, ...
        'maximumLateralReplayDifference',lateralDifference,'exactReproductionDifference',reproductionError, ...
        'halfStepMaximumStateDifference',refinementError,'runtimeSeconds',runSeconds, ...
        'design',design,'runtimeDiagnostics',estimate.diagnostics, ...
        'mapMatchingReused',true,'heldOutDrive',false, ...
        'limitations',"One previously inspected drive; >60 s excluded from this gain selection, not a never-seen independent drive. Map/query share the drive; references share the receiver. No universal dominance claim.");
    report=struct('summary',summary,'scores',scores,'inputFile',options.InputFile,'callsFile',options.CallsFile);
    writetable(scores,fullfile(outputFolder,'metrics.csv'));
    writetable(array2table([time,base.data.lidar.pose,estimate.pose,base.pvaPose],VariableNames= ...
        {'time','lidarX','lidarY','lidarPsi','observerX','observerY','observerPsi','pvaX','pvaY','referencePsi'}), ...
        fullfile(outputFolder,'trajectories.csv'));
    save(fullfile(outputFolder,'experiment.mat'),'report','cfg','design','estimate','lateralCfg', ...
        'lateralDesign','lateral','lateralInput','legacyMatched','frameIndices','-v7.3');
    fid=fopen(fullfile(outputFolder,'summary.json'),'w');assert(fid>=0);
    fprintf(fid,'%s\n',jsonencode(summary,PrettyPrint=true));fclose(fid);
    gains=struct('vehicle',lateralCfg.vehicle,'lateralGains',lateralDesign.gains, ...
        'lateralH2Bound',lateralDesign.h2Bound,'physicalGlobalGains',cfg.gains,'globalCertificate',design);
    fid=fopen(fullfile(outputFolder,'gains.json'),'w');assert(fid>=0);
    fprintf(fid,'%s\n',jsonencode(gains,PrettyPrint=true));fclose(fid);
    fig=figure('Color','w','Name','MnCAV motion-aided localization','Position',[100,100,1300,820]);
    tiledlayout(fig,2,2,'TileSpacing','compact');
    nexttile;plot(base.pvaPose(:,1),base.pvaPose(:,2),'k');hold on;
    plot(estimate.position(:,1),estimate.position(:,2),'Color',[0,.45,.2],'LineWidth',1.2);axis equal;grid on;
    xlabel('Map X (m)');ylabel('Map Y (m)');legend('INSPVA reference','Motion-aided observer',Location='best');
    nexttile;plot(time,vecnorm(base.data.lidar.pose(:,1:2)-base.pvaPose(:,1:2),2,2));hold on;
    plot(time,vecnorm(base.estimate.position-base.pvaPose(:,1:2),2,2));
    plot(time,vecnorm(estimate.position-base.pvaPose(:,1:2),2,2),'Color',[0,.45,.2],'LineWidth',1.2);xline(60,'--');grid on;
    xlabel('Receiver time (s)');ylabel('Position discrepancy (m)');
    legend('Actual LiDAR input','Previous observer','Motion-aided observer','Gain selection ends',Location='northwest');
    nexttile;native=vecnorm([calls.x,calls.y]-base.pvaPose(frameIndices,1:2),2,2);
    plot(calls.timeSeconds,native);hold on;plot(calls.timeSeconds,vecnorm(estimate.position(frameIndices,:)-base.pvaPose(frameIndices,1:2),2,2),'Color',[0,.45,.2],'LineWidth',1.2);
    grid on;xlabel('Native frame time (s)');ylabel('Position discrepancy (m)');
    legend('Matching / prediction on rejection','Motion-aided observer',Location='northwest');
    nexttile;plot(time,rad2deg(atan2(sin(base.data.lidar.pose(:,3)-base.pvaPose(:,3)),cos(base.data.lidar.pose(:,3)-base.pvaPose(:,3)))));hold on;
    plot(time,rad2deg(atan2(sin(estimate.heading-base.pvaPose(:,3)),cos(estimate.heading-base.pvaPose(:,3)))),'Color',[0,.45,.2],'LineWidth',1.2);
    grid on;xlabel('Receiver time (s)');ylabel('Heading discrepancy (deg)');legend('Actual LiDAR input','Motion-aided observer',Location='northwest');
    exportgraphics(fig,fullfile(outputFolder,'comparison.png'),'Resolution',160);
    exportgraphics(fig,fullfile(outputFolder,'comparison.pdf'),'ContentType','vector');
    disp(scores(scores.reference=="INSPVA" & scores.population=="uniform_all",:));
    fprintf('All population/reference gates passed: %d. Repository: %s\n',all(gates,'all'),root);
end

function m=metrics(pose,reference)
    e=vecnorm(pose(:,1:2)-reference(:,1:2),2,2);y=atan2(sin(pose(:,3)-reference(:,3)),cos(pose(:,3)-reference(:,3)));
    m=[rms(e),max(e),prctile(e,95),rad2deg(rms(y))];
end
