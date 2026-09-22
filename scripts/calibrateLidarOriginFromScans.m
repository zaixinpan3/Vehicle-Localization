function report=calibrateLidarOriginFromScans(inputFolder,outputFolder)
% calibrateLidarOriginFromScans Cross-check origin offset with raw 3-D surfaces.
% Independent calibration drive only; reference poses initialize relative ICP.
    arguments
        inputFolder (1,1) string="output/lidar_origin_20260922/calibration_inputs"
        outputFolder (1,1) string="output/lidar_origin_20260922/raw_calibration"
    end
    setupVehicleLocalization();if ~isfolder(outputFolder),mkdir(outputFolder);end
    poses=readtable(fullfile(inputFolder,'poses.csv'));n=height(poses);frames=poses.frame_index;
    clouds=cell(n,1);rotation=zeros(3,3,n);position=zeros(n,3);planar=zeros(n,3);
    for k=1:n
        s=load(fullfile(inputFolder,sprintf('frame_%04d.mat',frames(k))),'frame');
        p=[double(s.frame.x(:)),double(s.frame.y(:)),double(s.frame.z(:))];r=vecnorm(p(:,1:2),2,2);
        p=p(all(isfinite(p),2)&r>3&r<45&p(:,3)>-4&p(:,3)<6,:);
        clouds{k}=pcdownsample(pointCloud(p),'gridAverage',.20);
        [rotation(:,:,k),position(k,:)]=poseRowToRigidTransform(poses(k,:));planar(k,:)=poseRowToPlanarPose(poses(k,:));
    end
    rows=cell(0,13);queryPose=zeros(0,3);fixedPose=queryPose;matchedPose=queryPose;transforms=cell(0,1);
    for q=1:n
        for f=1:q-1
            if (frames(q)<200)~=(frames(f)<200) || frames(q)-frames(f)>20 || ...
                    frames(q)-frames(f)<10 || abs(planar(q,3)-planar(f,3))<.08,continue;end
            R=rotation(:,:,f).'*rotation(:,:,q);t=(position(q,:)-position(f,:))*rotation(:,:,f);
            initial=rigidtform3d(R,t);
            [transform,~,rmse]=pcregistericp(clouds{q},clouds{f},InitialTransform=initial, ...
                Metric="pointToPlane",MaxIterations=50,InlierRatio=.8,Tolerance=[.0001,.001]);
            worldR=rotation(:,:,f)*transform.R;worldT=transform.Translation*rotation(:,:,f).'+position(f,:);
            measured=[worldT(1:2),atan2(worldR(2,1),worldR(1,1))];d=measured-planar(q,:);d(3)=atan2(sin(d(3)),cos(d(3)));
            moved=transformPointsForward(transform,clouds{q}.Location);
            [~,distance]=findNearestNeighbors(clouds{f},moved,1);
            overlap=mean(distance<=.30);medianDistance=median(distance);
            good=overlap>=.6 && abs(d(3))<deg2rad(1);
            rows(end+1,:)={frames(q),frames(f),frames(q)<200,good,rmse,d(1),d(2),d(3), ...
                clouds{q}.Count,clouds{f}.Count,rad2deg(planar(q,3)-planar(f,3)),overlap,medianDistance}; %#ok<AGROW>
            queryPose(end+1,:)=planar(q,:);fixedPose(end+1,:)=planar(f,:);matchedPose(end+1,:)=measured; %#ok<AGROW>
            transforms{end+1,1}=transform; %#ok<AGROW>
        end
        if mod(q,10)==0,fprintf('Raw calibration %d/%d; pairs %d.\n',q,n,size(rows,1));drawnow;end
    end
    pairs=cell2table(rows,VariableNames={'queryFrame','fixedFrame','training','qualityPassed','rmseM', ...
        'errorX','errorY','yawErrorRad','queryPoints','fixedPoints','referenceYawChangeDeg','overlap','medianDistanceM'});
    save(fullfile(outputFolder,'raw_pairs.mat'),'pairs','queryPose','fixedPose','matchedPose','transforms');
    use=pairs.training & pairs.qualityPassed;
    [calibration,fit]=fitLidarTranslationCalibration(queryPose(use,:),fixedPose(use,:),matchedPose(use,:));
    calibration.identifier="mncav-front-lidar-reference-12-11-24-raw-planar-v1";
    pred=zeros(height(pairs),2);
    for k=1:height(pairs)
        a=queryPose(k,3);b=fixedPose(k,3);R=[cos(a)-cos(b),-sin(a)+sin(b);sin(a)-sin(b),cos(a)-cos(b)];
        pred(k,:)=(R*calibration.translation(1:2).').';
    end
    pairs.correctedResidualM=vecnorm([pairs.errorX,pairs.errorY]-pred,2,2);v=~pairs.training & pairs.qualityPassed;
    report=struct('calibration',calibration,'fit',fit,'validationPairs',nnz(v), ...
        'hasValidationEvidence',any(v),'deployed',false, ...
        'validationBeforeRmseM',sqrt(mean(pairs.errorX(v).^2+pairs.errorY(v).^2)), ...
        'validationAfterRmseM',sqrt(mean(pairs.correctedResidualM(v).^2)), ...
        'evaluationDriveUsed',false,'referencePoint',"recorded_INS_output_point", ...
        'limitations',"Diagnostic only; no acceptable validation pairs means an inconclusive calibration, not a validated offset");
    save(fullfile(outputFolder,'fit.mat'),'report','pairs','queryPose','fixedPose','matchedPose','transforms','-v7.3');
    writetable([pairs,array2table([queryPose,fixedPose,matchedPose], ...
        VariableNames={'qx','qy','qa','fx','fy','fa','mx','my','ma'})],fullfile(outputFolder,'relative_poses.csv'));
    fid=fopen(fullfile(outputFolder,'summary.json'),'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));disp(report);
end
