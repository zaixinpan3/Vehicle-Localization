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
    files=dir(fullfile(estimateFolder,'*_trajectory.csv'));rows=struct([]);
    assert(~isempty(files),'VehicleLocalization:NoPredictions','No saved predictions to score.');
    for file=files.'
        prediction=readtable(fullfile(file.folder,file.name));frames=prediction.frame;
        assert(all(abs(prediction.time-reference.lidar_stamp_sec(frames))<1e-7), ...
            'VehicleLocalization:ReferenceAlignment','Scoring timestamps differ from saved predictions.');
        truth=zeros(height(prediction),3);
        for k=1:height(prediction),truth(k,:)=poseRowToPlanarPose(reference(frames(k),:));end
        error=prediction{:,{'x','y','yaw'}}-truth;error(:,3)=atan2(sin(error(:,3)),cos(error(:,3)));
        position=hypot(error(:,1),error(:,2));steady=prediction.time>=prediction.time(1)+2;
        scenario=erase(string(file.name),'_trajectory.csv');
        row=struct('scenario',scenario,'frames',height(prediction),'startTime',prediction.time(1), ...
            'allRmseM',sqrt(mean(position.^2)),'allMaximumM',max(position), ...
            'after2sRmseM',sqrt(mean(position(steady).^2)),'after2sP95M',prctile(position(steady),95), ...
            'after2sMaximumM',max(position(steady)),'after2sYawRmseDeg',rad2deg(sqrt(mean(error(steady,3).^2))), ...
            'after2sWithin10cmPercent',100*mean(position(steady)<=.1));
        prediction.refX=truth(:,1);prediction.refY=truth(:,2);prediction.refYaw=truth(:,3);
        prediction.positionErrorM=position;prediction.yawErrorDeg=rad2deg(error(:,3));
        writetable(prediction,fullfile(scoreFolder,scenario+"_scores.csv"));rows=[rows;row]; %#ok<AGROW>
    end
    report=struct('estimates',string(estimateFolder),'reference',string(referenceFile),'scores',rows, ...
        'boundary',"Scoring reads immutable predictions. No estimate or measurement file is changed.");
    f=fopen(fullfile(scoreFolder,'summary.json'),'w');fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));fclose(f);
end
