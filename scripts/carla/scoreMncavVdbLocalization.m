function report=scoreMncavVdbLocalization(estimateFolder,referenceFile,scoreFolder)
% scoreMncavVdbLocalization Score already-saved estimates in an isolated step.
% This function never runs the estimator or writes to its dataset/results.
    arguments
        estimateFolder (1,1) string
        referenceFile (1,1) string
        scoreFolder (1,1) string
    end
    setupVehicleLocalization();
    assert(string(java.io.File(char(estimateFolder)).getCanonicalPath())~= ...
        string(java.io.File(char(scoreFolder)).getCanonicalPath()), ...
        'VehicleLocalization:SeparateScoringRequired','Keep scores in a separate output directory.');
    if ~isfolder(scoreFolder),mkdir(scoreFolder);end
    reference=readtable(referenceFile,'TextType','string');
    file=fullfile(estimateFolder,'fused_trajectory.csv');
    assert(isfile(file),'VehicleLocalization:NoPredictions','No fused trajectory to score.');
    prediction=readtable(file);frames=prediction.frame;
    assert(all(abs(prediction.time-reference.lidar_stamp_sec(frames))<1e-7), ...
        'VehicleLocalization:ReferenceAlignment','Scoring timestamps differ from saved predictions.');
    truth=zeros(height(prediction),3);
    for k=1:height(prediction),truth(k,:)=localizationSupport.poseRowToPlanarPose(reference(frames(k),:));end
    error=prediction{:,{'x','y','yaw'}}-truth;error(:,3)=atan2(sin(error(:,3)),cos(error(:,3)));
    position=hypot(error(:,1),error(:,2));steady=prediction.time>=prediction.time(1)+2;
    row=struct('scenario',"fused",'frames',height(prediction),'startTime',prediction.time(1), ...
        'allRmseM',sqrt(mean(position.^2)),'allMaximumM',max(position), ...
        'after2sRmseM',sqrt(mean(position(steady).^2)),'after2sP95M',prctile(position(steady),95), ...
        'after2sMaximumM',max(position(steady)),'after2sYawRmseDeg',rad2deg(sqrt(mean(error(steady,3).^2))), ...
        'after2sWithin10cmPercent',100*mean(position(steady)<=.1));
    prediction.refX=truth(:,1);prediction.refY=truth(:,2);prediction.refYaw=truth(:,3);
    prediction.positionErrorM=position;prediction.yawErrorDeg=rad2deg(error(:,3));
    writetable(prediction,fullfile(scoreFolder,"fused_scores.csv"));
    report=struct('estimates',string(estimateFolder),'reference',string(referenceFile),'scores',row, ...
        'boundary',"Scoring reads immutable predictions. No estimate or measurement file is changed.");
    f=fopen(fullfile(scoreFolder,'summary.json'),'w');fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));fclose(f);
end
