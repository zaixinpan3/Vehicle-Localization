% investigate_geometry Test inter-frame feature consistency and point origins.
% Translation fits are diagnostic effective offsets, not surveyed extrinsics.
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
previous=pwd;cleanup=onCleanup(@()cd(previous));cd(root);setupVehicleLocalization();
out=fullfile(root,'output/frame94_geometry_investigation_20260921');
if ~isfolder(out),mkdir(out);end
dest=fileparts(mfilename('fullpath'));
loadInput=load('output/mississippi_mapping_synchronized/feature_observations.mat','featureData');data=loadInput.featureData;
loadInput=load('output/frame94_fusion_diagnosis_20260921/diagnostic.mat');baseline=loadInput;
loadInput=load('output/saved_perception_inspva_20260915/experiment.mat','pcfg');pcfg=loadInput.pcfg;
cfg=baseline.cfg.registration;
trainingFrames=[60 65 70 75 80 85 90 100 105 110];
fixedChecks=[60 70 75 80 85 90 93 95 98 100 105 110 115];
allFrames=unique([trainingFrames,fixedChecks,94]);
clouds=cell(1170,1);poses=zeros(1170,3);
for f=allFrames
    poses(f,:)=poseRowToPlanarPose(data.framePoseTable(f,:));
    clouds{f}=buildSavedFeatureProbabilityCloud(data,f,poses(f,:),pcfg);
end
rows=cell(0,12);matches=cell(0,1);
for query=trainingFrames
    for fixed=trainingFrames
        if fixed>=query || query-fixed>25 || abs(poses(query,3)-poses(fixed,3))<.08,continue;end
        result=registerSemanticProbabilityCloud(globalCloud(clouds{fixed},poses(fixed,:)),clouds{query},poses(query,:),cfg);
        delta=result.poseXYTheta-poses(query,:);
        rows(end+1,:)={query,fixed,delta(1),delta(2),delta(3),result.accepted, ...
            result.similarity,result.matchedFraction,result.reason,poses(query,3),poses(fixed,3),height(result.correspondences)}; %#ok<SAGROW>
        matches{end+1,1}=result; %#ok<SAGROW>
    end
end
train=cell2table(rows,VariableNames={'query','fixed','errorX','errorY','yawErrorRad','accepted', ...
    'similarity','matchedFraction','reason','queryYaw','fixedYaw','pairs'});
% Predeclared geometric quality screen, with no position-error threshold.
use=train.accepted & abs(train.yawErrorRad)<deg2rad(1) & train.matchedFraction>=.2;
assert(nnz(use)>=8 && ~any(train.query==94|train.fixed==94));
A=zeros(2*nnz(use),2);b=zeros(2*nnz(use),1);selected=find(use);
for j=1:numel(selected)
    k=selected(j);A(2*j-1:2*j,:)=rotation(train.queryYaw(k))-rotation(train.fixedYaw(k));
    b(2*j-1:2*j)=[train.errorX(k);train.errorY(k)];
end
offset=A\b;
for iteration=1:20
    residual=reshape(A*offset-b,2,[]).';weights=min(1,.10./max(vecnorm(residual,2,2),eps));
    rootWeight=repelem(sqrt(weights),2);offset=(A.*rootWeight)\(b.*rootWeight);
end
train.usedForFit=use;train.predictedX=zeros(height(train),1);train.predictedY=train.predictedX;
for k=1:height(train)
    prediction=(rotation(train.queryYaw(k))-rotation(train.fixedYaw(k)))*offset;
    train.predictedX(k)=prediction(1);train.predictedY(k)=prediction(2);
end
train.residualM=hypot(train.errorX-train.predictedX,train.errorY-train.predictedY);
writetable(train,fullfile(dest,'translation_training_pairs.csv'));
checks=cell(numel(fixedChecks)*4,10);savedChecks=cell(size(checks,1),1);i=0;
for f=fixedChecks
    for sourceType=1:2
        query=clouds{94};if sourceType==2,query=baseline.source;end
        for calibrated=0:1
            q=query;target=globalCloud(clouds{f},poses(f,:));
            if calibrated
                % Exact planar offset experiment; source geometry and labels
                % are unchanged. Tilt transport is handled in the rebuild study.
                q.components.mean=q.components.mean+offset.';
                target.components.mean=target.components.mean+(rotation(poses(f,3))*offset).';
            end
            result=registerSemanticProbabilityCloud(target,q,poses(94,:),cfg);
            d=result.poseXYTheta-poses(94,:);i=i+1;savedChecks{i}=result;
            prediction=(rotation(poses(94,3))-rotation(poses(f,3)))*offset;
            checks(i,:)={f,sourceType,logical(calibrated),result.accepted,result.reason, ...
                norm(d(1:2)),d(1),d(2),rad2deg(d(3)),norm(prediction)};
        end
    end
end
checks=cell2table(checks,VariableNames={'fixedFrame','sourceType','offsetApplied','accepted','reason', ...
    'positionErrorM','errorX','errorY','yawErrorDeg','predictedUncorrectedErrorM'});
writetable(checks,fullfile(dest,'heldout_frame94_pairs.csv'));
summary=struct('trainingFrames',trainingFrames,'trainingPairs',height(train),'usedTrainingPairs',nnz(use), ...
    'query94ExcludedFromFit',true,'effectiveStoredToReferenceTranslationXYM',offset.', ...
    'fitSingularValues',svd(A).','unadjustedTrainingRmseM',sqrt(mean(sum(reshape(b,2,[]).^2,1))), ...
    'adjustedTrainingRmseM',sqrt(mean(train.residualM(use).^2)), ...
    'qualification',"Same-drive reference-assisted planar consistency fit with no query 94 pair in fitting; not surveyed extrinsics or independent-drive validation.");
writeJson(fullfile(dest,'translation_fit_summary.json'),summary);
save(fullfile(out,'geometry.mat'),'summary','train','checks','matches','savedChecks','offset','poses','clouds','baseline','-v7.3');
disp(summary);disp(checks(checks.sourceType==2,:));

function R=rotation(yaw)
    R=[cos(yaw),-sin(yaw);sin(yaw),cos(yaw)];
end
function cloud=globalCloud(cloud,pose)
    R=rotation(pose(3));cloud.components.mean=cloud.components.mean*R.'+pose(1:2);
    cloud.components.covariance=pagemtimes(pagemtimes(R,cloud.components.covariance),R.');
end
function writeJson(file,value)
    fid=fopen(file,'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(value,PrettyPrint=true));
end
