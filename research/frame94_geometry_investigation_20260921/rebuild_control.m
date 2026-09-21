% rebuild_control Rebuild a bounded map with unchanged labels and inference.
% Only the stored-LiDAR to reference-point translation differs between runs.
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
previous=pwd;cleanup=onCleanup(@()cd(previous));cd(root);setupVehicleLocalization();
out=fullfile(root,'output/frame94_geometry_investigation_20260921');dest=fileparts(mfilename('fullpath'));
study=load(fullfile(out,'geometry.mat'),'offset','baseline','summary');
loaded=load('output/mississippi_mapping_synchronized/feature_observations.mat','featureData');data=loaded.featureData;
loaded=load('output/mississippi_mapping_synchronized/probability_cloud_map.mat','probabilityCloudMap');mapCfg=loaded.probabilityCloudMap.canonicalMap.config;
matching=load('output/receiver_clock_20260921/coarse_pipeline/matching/report.mat');
call=matching.report.calls(94,:);reference=[call.referenceX call.referenceY call.referencePsi];
seed=[call.predictedX call.predictedY call.predictedPsi];
points=cell(1170*3,1);labels=points;frames=points;sources=points;shifts=points;i=0;
for f=1:1170
    [R,~]=poseRowToRigidTransform(data.framePoseTable(f,:));
    shift=(R*[study.offset;0]).';
    for c=1:3
        i=i+1;p=double(data.pointsByFeatureFrame{c,f});
        keep=vecnorm(p(:,1:2)-reference(1:2),2,2)<=65;ids=find(keep);p=p(keep,:);
        points{i}=p;shifts{i}=repmat(shift,size(p,1),1);
        labels{i}=repmat(data.featureNames(c),size(p,1),1);frames{i}=repmat(f,size(p,1),1);
        sources{i}=string(f)+":"+data.featureNames(c)+":"+string(ids);
    end
end
points=vertcat(points{:});shifts=vertcat(shifts{:});labels=vertcat(labels{:});frames=vertcat(frames{:});sources=vertcat(sources{:});
observations=struct('frameId',frames,'observationBlockId',frames-1,'sourceId',sources);
mapCfg.logEnabled=false;maps=cell(2,1);clouds=maps;results=cell(2,3);rows=cell(6,10);
row=0;mapSeconds=zeros(2,1);
for corrected=0:1
    calibration=lidarFrameCalibrationConfig();
    if corrected,calibration.translation=[study.offset.',0];calibration.identifier="diagnosticTranslationFitExcluding94";end
    currentCfg=mapCfg;currentCfg.frameCalibration=calibration;
    fprintf('Building profile %d with %d unchanged observations.\n',corrected,size(points,1));drawnow;
    timer=tic;maps{corrected+1}=buildTemporalStabilityGmmMap(points+corrected*shifts,labels,observations,currentCfg);
    mapSeconds(corrected+1)=toc(timer);clouds{corrected+1}=temporalMapToProbabilityCloud(maps{corrected+1});
    config=study.baseline.cfg;config.perception.frameCalibration=calibration;
    history=[];
    for f=92:94
        scan=loadPointCloudFrame('data/raw/MissisipiPointClouds.mat',f);
        [~,tilt]=poseRowToPlanarPose(data.framePoseTable(f,:));config.perception.coarseProbabilityCloud.projectionRotation=tilt;
        current=perceiveCoarseProbabilityCloud(scan,config.perception);
        dr=matching.report.deadReckoning(f,:);
        [source,history]=updateLocalizationSourceWindow(current,matching.report.calls.timeSeconds(f), ...
            [dr.x dr.y dr.psi],history,config.sourceWindow);
    end
    for variant=1:3
        initial=seed;moving=source;
        if variant==2,initial=reference;end
        if variant==3,moving=current;end
        r=registerSemanticProbabilityCloud(clouds{corrected+1},moving,initial,config.registration);
        results{corrected+1,variant}=r;d=r.poseXYTheta-reference;row=row+1;
        rows(row,:)={logical(corrected),variant,r.accepted,r.reason,norm(d(1:2)),d(1),d(2),rad2deg(d(3)),r.similarity,height(r.correspondences)};
    end
    save(fullfile(out,'map_control.mat'),'maps','clouds','results','rows','mapSeconds','points','labels','frames','observations','shifts','mapCfg','reference','seed','-v7.3');
    fprintf('Profile %d map completed in %.1f s; frame 94 error %.5f m.\n',corrected,mapSeconds(corrected+1),rows{row-2,5});drawnow;
end
tableResults=cell2table(rows,VariableNames={'offsetApplied','variant','accepted','reason','positionErrorM', ...
    'errorX','errorY','yawErrorDeg','similarity','matches'});
baselineDifference=max(abs(results{1,1}.poseXYTheta-study.baseline.results{1}.poseXYTheta));
assert(baselineDifference<1e-6,'Cropped reconstruction must reproduce the original frame-94 map match.');
writetable(tableResults,fullfile(dest,'rebuilt_map_controls.csv'));
summary=struct('sourceObservationCount',size(points,1),'roiRadiusM',65,'rebuildSeconds',mapSeconds.', ...
    'baselinePoseReproductionMaxAbs',baselineDifference,'offsetXYM',study.offset.', ...
    'sourceFrames',[92 93 94],'labelsChanged',false,'perceptionThresholdsChanged',false, ...
    'registrationConfigChanged',false,'mapInferenceConfigChanged',false, ...
    'limitation',"Diagnostic same-drive effective translation, fitted without frame 94; physical mounting and constant latency are not jointly calibrated.");
fid=fopen(fullfile(dest,'rebuilt_map_summary.json'),'w');assert(fid>=0);fileCleanup=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(summary,PrettyPrint=true));disp(tableResults);disp(summary);
