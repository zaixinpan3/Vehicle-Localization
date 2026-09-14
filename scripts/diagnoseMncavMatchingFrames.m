function report=diagnoseMncavMatchingFrames(outputFolder)
% diagnoseMncavMatchingFrames Re-solve selected scans with diagnostic seeds.
% Frozen crop and source cloud isolate local optimizer initialization. The
% reference seed is diagnostic only and is not a localization result.
    arguments
        outputFolder (1,1) string="output/mncav_error_diagnosis_20260914"
    end
    root=setupVehicleLocalization();if ~isfolder(outputFolder),mkdir(outputFolder);end
    calls=readtable(fullfile(root,'output','mncav_full_localization_20260914','matching','calls.csv'));
    loaded=load(fullfile(root,'output','mississippi_mapping_20260912','probability_cloud_map.mat'));
    cloud=registrationSupport.projectSemanticProbabilityCloud(temporalMapToProbabilityCloud(loaded.probabilityCloudMap),2);
    mapCfg=featureMapBuildConfig();pcfg=perceptionConfig("Mississippi");cfg=distributionRegistrationConfig();
    frames=[100,300,700,820,879,1090,1093,1098];rows={};details=cell(numel(frames),2);
    poses=readFramePoseTable(fullfile(root,'data',mapCfg.poseMatchCsvPath),frames);
    for k=1:numel(frames)
        id=frames(k);c=calls(calls.frame==id,:);pred=[c.predictedX,c.predictedY,c.predictedPsi];
        ref=[c.referenceX,c.referenceY,c.referencePsi];
        [~,pcfg.coarseProbabilityCloud.projectionRotation]=poseRowToPlanarPose(poses(k,:));
        frame=loadPointCloudFrame(fullfile(root,'data',mapCfg.pointCloudMatPath),id);
        moving=perceiveCoarseProbabilityCloud(frame,pcfg);local=crop(cloud,pred);
        for j=1:2
            seeds=[pred;ref];r=registerSemanticProbabilityCloud(local,moving,seeds(j,:),cfg);details{k,j}=r;
            d=r.poseXYTheta-ref;d(3)=atan2(sin(d(3)),cos(d(3)));
            classText=strjoin(r.classDiagnostics.semanticName+":"+string(r.classDiagnostics.matchedComponents)+ ...
                "/rank"+string(r.classDiagnostics.observableRank),";");
            rows{end+1}=struct('frame',id,'referenceSeed',j==2,'accepted',r.accepted, ...
                'reason',r.reason,'positionDiscrepancyM',norm(d(1:2)), ...
                'longitudinalDiscrepancyM',dot(d(1:2),[cos(ref(3)),sin(ref(3))]), ...
                'similarity',r.similarity,'minimumInformationEigenvalue',min(eig(r.information)), ...
                'maximumClassCorrection',max(r.classDiagnostics.observableCorrection), ...
                'classes',classText,'x',r.poseXYTheta(1),'y',r.poseXYTheta(2)); %#ok<AGROW>
        end
        fprintf('Frame %d diagnostic complete.\n',id);
    end
    report=struct2table(vertcat(rows{:}));writetable(report,fullfile(outputFolder,'matching_seeds.csv'));
    save(fullfile(outputFolder,'matching_diagnostics.mat'),'details','frames','report');disp(report);
end

function local=crop(cloud,pose)
    local=cloud;c=cloud.components;keep=sum((c.mean-pose(1:2)).^2,2)<=100^2;
    local.components.mean=c.mean(keep,:);local.components.covariance=c.covariance(:,:,keep);
    local.components.numComponents=nnz(keep);
    for field=["semanticName","mixtureWeight","repeatability","semanticProbability","occupancyProbability","supportAmplitude"]
        if isfield(c,field),local.components.(field)=c.(field)(keep,:);end
    end
end
