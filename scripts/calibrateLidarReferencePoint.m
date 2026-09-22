function report=calibrateLidarReferencePoint(inputFolder,outputFolder)
% calibrateLidarReferencePoint Fit an independent-drive origin increment.
% Fixed early/later frame intervals separate fit and validation pairs.
    arguments
        inputFolder (1,1) string="output/lidar_origin_20260922/calibration_inputs"
        outputFolder (1,1) string="output/lidar_origin_20260922/calibration"
    end
    setupVehicleLocalization();if ~isfolder(outputFolder),mkdir(outputFolder);end
    poses=readtable(fullfile(inputFolder,'poses.csv'));frames=poses.frame_index;n=height(poses);
    pcfg=perceptionConfig("Mississippi");pcfg.frameCalibration=lidarFrameCalibrationConfig();pcfg.executionMode="offline";
    adapter=load('output/saved_perception_inspva_20260915/experiment.mat','pcfg');
    data=struct('featureNames',pcfg.featureNames,'frameIndices',frames.','framePoseTable',poses, ...
        'pointsByFeatureFrame',{cell(numel(pcfg.featureNames),n)},'frameCalibration',pcfg.frameCalibration);
    cloud=cell(n,1);reference=zeros(n,3);
    for k=1:n
        loaded=load(fullfile(inputFolder,sprintf('frame_%04d.mat',frames(k))),'frame');frame=loaded.frame;
        result=perceiveFrame(frame,pcfg);points=[double(frame.x(:)),double(frame.y(:)),double(frame.z(:))];
        [R,t]=poseRowToRigidTransform(poses(k,:));
        for c=1:numel(pcfg.featureNames)
            mask=result.featureMasks.(pcfg.featureNames(c));p=points(mask(:),:);
            data.pointsByFeatureFrame{c,k}=p*R.'+t;
        end
        reference(k,:)=poseRowToPlanarPose(poses(k,:));
        cloud{k}=buildSavedFeatureProbabilityCloud(data,k,reference(k,:),adapter.pcfg);
    end
    cfg=distributionRegistrationConfig();rows=cell(0,12);queryPose=zeros(0,3);fixedPose=queryPose;matchedPose=queryPose;
    for q=1:n
        for f=1:q-1
            if (frames(q)<200)~=(frames(f)<200) || frames(q)-frames(f)>30 || ...
                    abs(reference(q,3)-reference(f,3))<.08,continue;end
            target=cloud{f};R=rotation(reference(f,3));
            target.components.mean=target.components.mean*R.'+reference(f,1:2);
            target.components.covariance=pagemtimes(pagemtimes(R,target.components.covariance),R.');
            match=registerSemanticProbabilityCloud(target,cloud{q},reference(q,:),cfg);
            d=match.poseXYTheta-reference(q,:);d(3)=atan2(sin(d(3)),cos(d(3)));
            good=match.accepted && abs(d(3))<deg2rad(1) && match.matchedFraction>=.2;
            rows(end+1,:)={frames(q),frames(f),frames(q)<200,good,match.accepted,match.reason, ...
                d(1),d(2),d(3),match.similarity,match.matchedFraction,height(match.correspondences)}; %#ok<AGROW>
            queryPose(end+1,:)=reference(q,:);fixedPose(end+1,:)=reference(f,:);matchedPose(end+1,:)=match.poseXYTheta; %#ok<AGROW>
        end
    end
    pairs=cell2table(rows,VariableNames={'queryFrame','fixedFrame','training','qualityPassed','accepted','reason', ...
        'errorX','errorY','yawErrorRad','similarity','matchedFraction','matches'});
    use=pairs.training & pairs.qualityPassed;
    [calibration,fit]=fitLidarTranslationCalibration(queryPose(use,:),fixedPose(use,:),matchedPose(use,:));
    calibration.identifier="mncav-front-lidar-reference-12-11-24-planar-v1";
    prediction=zeros(height(pairs),2);
    for k=1:height(pairs)
        prediction(k,:)=((rotation(queryPose(k,3))-rotation(fixedPose(k,3)))*calibration.translation(1:2).').';
    end
    pairs.correctedResidualM=vecnorm([pairs.errorX,pairs.errorY]-prediction,2,2);
    validation=~pairs.training & pairs.qualityPassed;
    report=struct('calibration',calibration,'fit',fit,'validationPairs',nnz(validation), ...
        'validationBeforeRmseM',sqrt(mean(pairs.errorX(validation).^2+pairs.errorY(validation).^2)), ...
        'validationAfterRmseM',sqrt(mean(pairs.correctedResidualM(validation).^2)), ...
        'evaluationDriveUsed',false,'trainingFrames',frames(frames<200).', ...
        'validationFrames',frames(frames>200).','referencePoint',"recorded_INS_output_point", ...
        'inputFolder',inputFolder,'limitations',"Empirical planar fit; vertical offset, rotation and absolute scan latency remain uncalibrated");
    save(fullfile(outputFolder,'fit.mat'),'report','pairs','data','cloud','reference','queryPose','fixedPose','matchedPose','-v7.3');
    writetable(pairs,fullfile(outputFolder,'pairs.csv'));
    writetable([pairs,array2table([queryPose,fixedPose,matchedPose], ...
        VariableNames={'qx','qy','qa','fx','fy','fa','mx','my','ma'})],fullfile(outputFolder,'relative_poses.csv'));
    fid=fopen(fullfile(outputFolder,'summary.json'),'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));disp(report);
end

function R=rotation(yaw)
    R=[cos(yaw),-sin(yaw);sin(yaw),cos(yaw)];
end
