function report = evaluateHeightProbabilityCloud(outputFolder)
% evaluateHeightProbabilityCloud: Rebuild local height maps and compare D2D.
% Query frames are excluded from each six-frame map. Differences are from
% recorded mapping poses, not independent ground-truth localization accuracy.
% Perturbations are deterministic; production uncertainty defaults are used.
    arguments
        outputFolder (1,1) string
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    mapCfg=featureMapBuildConfig(); mapCfg.logEnabled=false;
    matPath=fullfile(root,'data',mapCfg.pointCloudMatPath);
    posePath=fullfile(root,'data',mapCfg.poseMatchCsvPath);
    frames=[260 550 900];
    heightOffsets=[0 0.1 -0.1 0.3 0.3];
    tiltOffsets=[0 0 0 0 deg2rad(0.5)];
    poseOffsets=[0.5 -0.4 deg2rad(2);-0.5 0.4 -deg2rad(2)];
    rows=cell(numel(frames)*numel(heightOffsets)*2*size(poseOffsets,1),14);
    timing=zeros(numel(frames),4);
    maps=cell(numel(frames),1);
    index=0;
    for k=1:numel(frames)
        queryFrame=frames(k);
        mappingFrames=queryFrame+(1:6);
        mappingPoses=readFramePoseTable(posePath,mappingFrames);
        observations=collectFeatureObservations(matPath,mappingFrames,mappingPoses,perceptionConfig(),mapCfg);
        maps{k}=buildSlidingWindowMap(observations,mapCfg);
        fixed=temporalMapToProbabilityCloud(maps{k},1);
        assert(all(fixed.components.heightAvailable));
        queryPose=readFramePoseTable(posePath,queryFrame);
        [pose,tilt,height]=poseRowToPlanarPose(queryPose);
        frame=loadPointCloudFrame(matPath,queryFrame);
        pcfg=perceptionConfig(); pcfg.coarseProbabilityCloud.projectionRotation=tilt;
        moving=perceiveCoarseProbabilityCloud(frame,pcfg);
        timing(k,:)=[queryFrame,1e3*timeit(@() perceiveCoarseProbabilityCloud(frame,pcfg)), ...
            moving.components.numComponents,fixed.components.numComponents];
        for variant=1:numel(heightOffsets)
            a=tiltOffsets(variant);
            pcfg.coarseProbabilityCloud.projectionRotation=[cos(a) 0 sin(a);0 1 0;-sin(a) 0 cos(a)]*tilt;
            moving=perceiveCoarseProbabilityCloud(frame,pcfg);
            for mode=["xy","xyz"]
                cfg=distributionRegistrationConfig(); cfg.method="densityOverlap"; cfg.heightMode=mode;
                cfg.heightTranslation=height+heightOffsets(variant);
                for startIndex=1:size(poseOffsets,1)
                    timer=tic;
                    result=registerSemanticProbabilityCloud(fixed,moving,pose+poseOffsets(startIndex,:),cfg);
                    seconds=toc(timer);
                    yaw=result.poseXYTheta(3)-pose(3);
                    index=index+1;
                    rows(index,:)={queryFrame,mappingFrames(1),mappingFrames(end),mode, ...
                        heightOffsets(variant),rad2deg(a),startIndex,result.accepted,result.reason, ...
                        result.similarity,norm(result.poseXYTheta(1:2)-pose(1:2)), ...
                        abs(rad2deg(atan2(sin(yaw),cos(yaw)))),seconds,result.height.heightUsed};
                end
            end
        end
    end
    report=struct();
    report.metrics=cell2table(rows,'VariableNames',{'frameIndex','mapStartFrame','mapEndFrame','mode', ...
        'heightOffsetMeters','tiltOffsetDegrees','initialPoseCase','accepted','reason','similarity', ...
        'mapPoseTranslationDifferenceMeters','mapPoseYawDifferenceDegrees','registrationSeconds','heightUsed'});
    report.runtime=array2table(timing,'VariableNames',{'frameIndex','coarseMilliseconds','sourceComponents','mapComponents'});
    report.description="Same-route map-pose consistency; query frame excluded; no independent accuracy claim.";
    if ~isfolder(outputFolder), mkdir(outputFolder); end
    writetable(report.metrics,fullfile(outputFolder,'registration_height.csv'));
    writetable(report.runtime,fullfile(outputFolder,'coarse_runtime.csv'));
    save(fullfile(outputFolder,'height_map_evaluation.mat'),'report','maps','frames','heightOffsets','tiltOffsets','poseOffsets');
    disp(report.runtime);
    disp(report.metrics);
end
