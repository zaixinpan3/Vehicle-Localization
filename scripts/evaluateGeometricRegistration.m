function report = evaluateGeometricRegistration(outputFolder, frames, calibration)
% evaluateGeometricRegistration: Query-excluded map consistency, not ground truth.
% Fixed starts compare legacy density overlap with semantic Gaussian geometry.
% Inputs are cached by frame and explicit calibration identifier, with the
% transform checked on cache reuse. Every map uses the next six frames only.
    arguments
        outputFolder (1,1) string
        frames (1,:) double = [260 550 900 120 350 700 1050]
        calibration (1,1) struct = lidarFrameCalibrationConfig()
    end
    calibration=validateLidarFrameCalibration(calibration);
    root=fileparts(fileparts(mfilename('fullpath')));
    if ~isfolder(outputFolder), mkdir(outputFolder); end
    cfg=featureMapBuildConfig(); cfg.logEnabled=false; cfg.frameCalibration=calibration;
    pcfg=perceptionConfig(); pcfg.frameCalibration=calibration;
    matPath=fullfile(root,'data',cfg.pointCloudMatPath);
    posePath=fullfile(root,'data',cfg.poseMatchCsvPath);
    tag=string(matlab.lang.makeValidName(calibration.identifier));
    starts=[.5 -.4 deg2rad(2);-.5 .4 -deg2rad(2);0 0 0];
    rows=cell(0,17); times=cell(0,5);
    for frameIndex=frames
        cachePath=fullfile(outputFolder,sprintf('input_%d_%s.mat',frameIndex,tag));
        if isfile(cachePath)
            loaded=load(cachePath,'item'); item=loaded.item;
            assert(isequal(item.calibration,calibration),'Cached calibration differs; use another output folder.');
        else
            item=struct('frame',frameIndex,'calibration',calibration);
            row=readFramePoseTable(posePath,frameIndex);
            [item.pose,item.tilt,item.height]=poseRowToPlanarPose(row);
            frame=loadPointCloudFrame(matPath,frameIndex);
            pcfg.coarseProbabilityCloud.projectionRotation=item.tilt;
            item.coarse=perceiveCoarseProbabilityCloud(frame,pcfg);
            future=frameIndex+(1:6); poses=readFramePoseTable(posePath,future);
            item.observations=collectFeatureObservations(matPath,future,poses,pcfg,cfg);
            item.map=buildSlidingWindowMap(item.observations,cfg);
            item.fixed=temporalMapToProbabilityCloud(item.map);
            save(cachePath,'item','-v7.3');
        end
        for variant=["densityXY","geometricXY","geometricHeight"]
            registration=distributionRegistrationConfig(); registration.heightTranslation=item.height;
            registration.method="geometricD2D";
            if variant=="densityXY", registration.method="densityOverlap"; end
            if variant=="geometricHeight", registration.heightMode="xyz"; end
            for start=1:size(starts,1)
                result=registerSemanticProbabilityCloud(item.fixed,item.coarse,item.pose+starts(start,:),registration);
                delta=result.poseXYTheta-item.pose;
                r=[cos(item.pose(3)) -sin(item.pose(3));sin(item.pose(3)) cos(item.pose(3))];
                body=delta(1:2)*r;
                rank=NaN; matches=NaN;
                if isfield(result,'observableRank'), rank=result.observableRank; end
                if isfield(result,'matchedFraction'), matches=result.matchedFraction; end
                rows(end+1,:)={frameIndex,tag,variant,start,result.accepted,result.reason, ...
                    norm(delta(1:2)),rad2deg(atan2(sin(delta(3)),cos(delta(3)))), ...
                    body(1),body(2),rank,matches,result.iterations,result.similarity, ...
                    result.poseXYTheta(1),result.poseXYTheta(2),result.poseXYTheta(3)}; %#ok<AGROW>
            end
            seconds=timeit(@() registerSemanticProbabilityCloud(item.fixed,item.coarse,item.pose+starts(1,:),registration));
            times(end+1,:)={frameIndex,tag,variant,1000*seconds,item.coarse.components.numComponents}; %#ok<AGROW>
        end
        fprintf('Evaluated geometric registration frame %d (%s).\n',frameIndex,tag);
    end
    report=struct('description',"Same-route recorded-pose consistency; query excluded from six-frame map. No independent truth.", ...
        'calibration',calibration,'frames',frames,'starts',starts);
    report.metrics=cell2table(rows,'VariableNames',{'frame','calibration','variant','start','accepted','reason', ...
        'translationDifferenceM','yawDifferenceDeg','longitudinalDifferenceM','lateralDifferenceM', ...
        'observableRank','matchedFraction','iterations','compatibility','x','y','yaw'});
    report.timing=cell2table(times,'VariableNames',{'frame','calibration','variant','registrationMs','sourceComponents'});
    writetable(report.metrics,fullfile(outputFolder,'registration_'+tag+'.csv'));
    writetable(report.timing,fullfile(outputFolder,'timing_'+tag+'.csv'));
    save(fullfile(outputFolder,'evaluation_'+tag+'.mat'),'report');
    disp(report.metrics(report.metrics.start==1,1:12));
end
