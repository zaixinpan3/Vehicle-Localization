function report=runSavedPerceptionMatchingBaseline(outputFolder,options)
% runSavedPerceptionMatchingBaseline Match every saved fine-perception frame.
% Three reference-relative seeds diagnose local registration. A fourth run
% uses only the first reference pose and previous full matches for prediction.
% Recorded pose inverts the saved global features; it is not passed into the
% recursive prediction after initialization. All frames contributed to the map.
    arguments
        outputFolder (1,1) string="output/saved_perception_baseline_20260914"
        options.ObservationFile (1,1) string="output/mississippi_perception_video_20260912/feature_observations.mat"
        options.MapFile (1,1) string="output/mississippi_mapping_20260912/probability_cloud.mat"
        options.MapRebuilt (1,1) logical=false
    end
    setupVehicleLocalization();if ~isfolder(outputFolder),mkdir(outputFolder);end
    loaded=load(options.ObservationFile,'featureData');data=loaded.featureData;
    loaded=load(options.MapFile,'cloud');fixed=loaded.cloud;
    cfg=distributionRegistrationConfig();pcfg=coarseSemanticProbabilityCloudConfig();
    n=data.numFrames;assert(isequal(data.frameIndices(:),(1:n).') && n==1170);
    reference=zeros(n,3);for k=1:n,reference(k,:)=poseRowToPlanarPose(data.framePoseTable(k,:));end
    % The prior exact frame clock and PVA reference are evaluation metadata.
    % No prior matching pose, motion measurement or observer state is used.
    if ismember('pose_source',data.framePoseTable.Properties.VariableNames)
        assert(all(data.framePoseTable.pose_source=="INSPVA"),'Expected native INSPVA mapping poses.');
        time=data.framePoseTable.receiver_time_sec;
        pvaReference=reference;
        referenceUse="INSPVA poses at LiDAR timestamps invert saved global features and provide evaluation/diagnostic seeds; no pose reset in recursive mode";
    else
        previous=readtable('output/mncav_zero_delay_20260914/precomputed_calls.csv');
        assert(isequal(previous.frame,data.frameIndices(:)));
        assert(max(abs(previous.rosStamp-data.framePoseTable.lidar_stamp_sec))<1e-6);
        time=previous.timeSeconds;
        pva=readtable('output/mncav_error_diagnosis_20260914/pva_reference.csv');
        pvaReference=[interp1(pva.time,[pva.x,pva.y],time,'linear'),reference(:,3)];
        referenceUse="Recorded pose reverses stored global features; PVA and prior cache supply evaluation/clock only";
    end
    assert(all(isfinite(pvaReference),'all') && all(diff(time)>0));
    offsets=[.5,-.4,deg2rad(2);-.5,.4,-deg2rad(2);0,0,0];
    names=["per_frame_positive","per_frame_negative","per_frame_zero","lidar_only_recursive"];
    rows=cell(4*n,37);runs=cell(n,4);clouds=cell(n,1);
    buildSeconds=zeros(n,1);roundTrip=zeros(n,1);acceptedTimes=zeros(0,1);acceptedPoses=zeros(0,3);
    initial=reference(1,:)+offsets(1,:);velocity=zeros(1,3);lastState=initial;
    timer=tic;
    for k=1:n
        buildTimer=tic;
        [moving,roundTrip(k)]=buildSavedFeatureProbabilityCloud(data,k,reference(k,:),pcfg);
        buildSeconds(k)=toc(buildTimer);clouds{k}=moving;
        for j=1:4
            if j<=3
                predicted=reference(k,:)+offsets(j,:);
            elseif k==1
                predicted=initial;
            else
                predicted=lastState+velocity*(time(k)-time(k-1));
                predicted(3)=wrap(predicted(3));
            end
            callTimer=tic;result=registerSemanticProbabilityCloud(fixed,moving,predicted,cfg);
            registrationSeconds=toc(callTimer);
            event=registrationSupport.registrationPoseMeasurement(result,time(k));
            full=result.accepted;directional=result.directionalAccepted;
            assert((full || directional)==~isempty(event));
            output=predicted;measurement=nan(1,3);info=zeros(3);kind="rejected";
            if ~isempty(event)
                output=event.pose;measurement=event.pose;info=event.information;kind=event.measurementType;
                assert(norm(info-info.','fro')<=1e-9*max(1,norm(info,'fro')));
                assert(min(eig((info+info.')/2))>=-1e-9*max(1,norm(info,2)));
                if full,[~,flag]=chol(info);assert(flag==0);end
            end
            if j==4
                lastState=output;
                if full
                    acceptedTimes(end+1,1)=time(k);acceptedPoses(end+1,:)=output; %#ok<AGROW>
                    if numel(acceptedTimes)>=2
                        delta=acceptedPoses(end,:)-acceptedPoses(end-1,:);delta(3)=wrap(delta(3));
                        velocity=delta/(acceptedTimes(end)-acceptedTimes(end-1));
                    end
                end
            end
            e=output-reference(k,:);ep=output-pvaReference(k,:);e(3)=wrap(e(3));
            candidate=result.poseXYTheta-reference(k,:);candidate(3)=wrap(candidate(3));
            row=(k-1)*4+j;
            rows(row,:)={k,time(k),names(j),full,directional,kind,string(result.reason), ...
                predicted(1),predicted(2),predicted(3),output(1),output(2),output(3), ...
                measurement(1),measurement(2),measurement(3), ...
                result.poseXYTheta(1),result.poseXYTheta(2),result.poseXYTheta(3), ...
                norm(e(1:2)),norm(ep(1:2)),rad2deg(e(3)),norm(candidate(1:2)), ...
                result.similarity,result.observableRank,result.iterations,result.converged, ...
                moving.components.numComponents,registrationSeconds,buildSeconds(k),roundTrip(k), ...
                info(1,1),info(1,2),info(1,3),info(2,2),info(2,3),info(3,3)};
            runs{k,j}=struct('result',result,'measurement',event);
        end
        if mod(k,100)==0 || k==n
            calls=makeTable(rows(1:4*k,:));writetable(calls,fullfile(outputFolder,'calls.csv'));
            recursive=calls.mode==names(4);
            fprintf('Saved perception %d/%d: recursive full=%d, rejected=%d, latest PVA=%.3f m; elapsed %.1f s.\n', ...
                k,n,nnz(calls.fullPose(recursive)),nnz(calls.measurementType(recursive)=="rejected"),calls.pvaErrorM(end),toc(timer));
        end
    end
    elapsedSeconds=toc(timer);calls=makeTable(rows);
    metricRows=cell(0,13);
    for j=1:4
        selected=calls.mode==names(j);
        for population=["all_outputs","full_measurements"]
            mask=selected;if population=="full_measurements",mask=mask & calls.fullPose;end
            for ref=["recorded_mapping_pose","INSPVA"]
                e=calls.mappingErrorM(mask);if ref=="INSPVA",e=calls.pvaErrorM(mask);end
                metricRows(end+1,:)={names(j),population,ref,numel(e),rms(e),median(e),prctile(e,95),max(e), ...
                    mean(e<=.05),mean(e<=.1),rms(calls.yawErrorDeg(mask)), ...
                    nnz(calls.fullPose(selected)),nnz(calls.directionalPose(selected))}; %#ok<AGROW>
            end
        end
    end
    metrics=cell2table(metricRows,VariableNames={'mode','population','reference','samples','positionRmseM', ...
        'positionMedianM','positionP95M','positionMaximumM','fractionAtMost5cm','fractionAtMost10cm', ...
        'headingRmseDeg','fullPoseCount','directionalPoseCount'});
    metadata=struct('observationFile',options.ObservationFile,'mapFile',options.MapFile, ...
        'sourceRepresentation',"Saved fine curb/pole/trafficSign points aggregated into XY Gaussian cells", ...
        'mapComponents',fixed.components.numComponents,'sourceCellSizeM',pcfg.resolution, ...
        'frames',n,'registrationCalls',4*n,'offsets',offsets,'elapsedSeconds',elapsedSeconds, ...
        'maximumRoundTripErrorM',max(roundTrip),'randomness',"None", ...
        'seededModes',"Every frame is initialized relative to its recorded mapping pose; diagnostic only", ...
        'recursivePrediction',"One initial pose plus positive offset; map-frame constant velocity/yaw rate from latest two full accepted matches; rejected frames propagate; no reference reset", ...
        'directionalHandling',"Directional event corrects its observable directions; only full events update predictor velocity", ...
        'referenceUse',referenceUse, ...
        'rejectedHandling',"Measurement fields remain NaN; all_outputs retains seed/prediction, and rejected solver candidate is separate", ...
        'mapOverlap',"Query frames contributed to this map; in-sample consistency, not independent ground-truth accuracy", ...
        'observerUsed',false,'motionSensorsUsed',false,'perceptionRerun',false,'mapRebuilt',options.MapRebuilt, ...
        'timingScope',"Matching on precomputed fine perception; excludes original perception, mapping and sensor latency", ...
        'matlabVersion',version);
    report=struct('metadata',metadata,'metrics',metrics);
    save(fullfile(outputFolder,'experiment.mat'),'report','calls','runs','clouds','cfg','pcfg','reference','pvaReference','time','buildSeconds','-v7.3');
    writetable(calls,fullfile(outputFolder,'calls.csv'));writetable(metrics,fullfile(outputFolder,'metrics.csv'));
    writeJson(fullfile(outputFolder,'summary.json'),report);
    fig=figure('Color','w','Name','Saved perception: map matching only','Position',[100,100,1250,950]);
    tiledlayout(fig,3,1,TileSpacing='compact');
    nexttile;hold on;
    for j=1:3,mask=calls.mode==names(j);plot(time,calls.pvaErrorM(mask),'DisplayName',names(j));end
    xlabel('Receiver time (s)');ylabel('Position discrepancy from INSPVA (m)');grid on;legend(Interpreter='none');
    title('Per-frame seeded diagnostics; rejected outputs retain their seed');
    nexttile;mask=calls.mode==names(4);plot(time,calls.pvaErrorM(mask));grid on;
    xlabel('Receiver time (s)');ylabel('Recursive position discrepancy (m)');
    title('LiDAR-only constant-velocity prediction: all frames, no reset');
    nexttile;hold on;
    for j=1:3
        mask=calls.mode==names(j) & calls.fullPose;e=sort(calls.pvaErrorM(mask));
        plot(100*e,(1:numel(e))/numel(e),'DisplayName',names(j));
    end
    xlabel('Accepted full-pose discrepancy from INSPVA (cm)');ylabel('Fraction of accepted frames');
    grid on;legend(Interpreter='none',Location='southeast');xlim([0,100]);ylim([0,1]);
    exportgraphics(fig,fullfile(outputFolder,'matching_errors.png'),'Resolution',160);
    exportgraphics(fig,fullfile(outputFolder,'matching_errors.pdf'),'ContentType','vector');
    disp(metrics);
end
function value=wrap(value)
    value=atan2(sin(value),cos(value));
end
function calls=makeTable(rows)
    calls=cell2table(rows,VariableNames={'frame','time','mode','fullPose','directionalPose','measurementType','reason', ...
        'initialX','initialY','initialPsi','outputX','outputY','outputPsi', ...
        'measurementX','measurementY','measurementPsi','candidateX','candidateY','candidatePsi', ...
        'mappingErrorM','pvaErrorM','yawErrorDeg','candidateMappingErrorM','similarity','observableRank', ...
        'iterations','converged','sourceComponents','registrationSeconds','sourceBuildSeconds','roundTripM', ...
        'informationXX','informationXY','informationXPsi','informationYY','informationYPsi','informationPsiPsi'});
end
function writeJson(path,value)
    fid=fopen(path,'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(value,PrettyPrint=true));
end
