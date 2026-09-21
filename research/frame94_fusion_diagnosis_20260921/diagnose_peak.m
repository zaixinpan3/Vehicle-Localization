% diagnose_peak Reproduce and diagnose the corrected-clock fusion maximum.
% Oracle substitutions are offline attribution controls, not deployable fixes.
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
previousFolder=pwd; folderCleanup=onCleanup(@()cd(previousFolder)); cd(root);
setupVehicleLocalization();
destination=fullfile(root,'research/frame94_fusion_diagnosis_20260921');
output=fullfile(root,'output/frame94_fusion_diagnosis_20260921');
if ~isfolder(output),mkdir(output);end
base='output/receiver_clock_20260921/coarse_pipeline';
saved=load(fullfile(base,'observer/experiment.mat'));
matching=load(fullfile(base,'matching/report.mat'));
reference=saved.reference; estimate=saved.runs{1}.estimate;
errorXY=estimate.position-reference(:,1:2);
[maximumError,index]=max(vecnorm(errorXY,2,2));
time=estimate.time; [epochDifference,frameRow]=min(abs(matching.report.calls.timeSeconds-time(index)));
frame=matching.report.calls.frame(frameRow);assert(frame==94 && epochDifference<1e-9);
replayed=runFullLocalizationObserver(saved.data,saved.lateralDesign,saved.cfg,LateralInputs=saved.lateral);
reproduction=max(abs(replayed.z-estimate.z),[],'all'); assert(reproduction<1e-8);
diagnostics=estimate.diagnostics;

% Exact decomposition of the realized position update. Motion includes the
% realized velocity observer, heading, body inputs and implicit discretization.
% Terms are signed vectors with memory, not independent error variances.
n=numel(time);terms=zeros(n,2,4);terms(1,:,4)=errorXY(1,:);
for k=2:n
    dt=time(k)-time(k-1);
    information=diagnostics.gnssInformationAtObserverPoint(:,:,k);
    Kg=saved.cfg.gnss.positionGain*(information/(information+saved.cfg.gnss.gainInformationScale*eye(2)));
    information=saved.data.lidar.information(:,:,k);
    W=information/(information+saved.cfg.lidar.gainInformationScale*eye(3));
    Kl=saved.cfg.gains(1)*W(1:2,1:2);
    A=eye(2)+dt*(Kg+Kl); forcing=zeros(2,4);
    forcing(:,1)=dt*Kl*(saved.data.lidar.pose(k,1:2)-reference(k,1:2)).';
    forcing(:,2)=dt*Kg*(diagnostics.gnssPositionAtObserverPoint(k,:)-reference(k,1:2)).';
    forcing(:,3)=(dt*estimate.velocity(k,:)-(reference(k,1:2)-reference(k-1,1:2))).';
    for j=1:4,terms(k,:,j)=(A\(terms(k-1,:,j).'+forcing(:,j))).';end
end
decompositionResidual=max(vecnorm(sum(terms,3)-errorXY,2,2));assert(decompositionResidual<1e-7);
referenceVelocity=[gradient(reference(:,1),time),gradient(reference(:,2),time)];
referenceBody=[cos(reference(:,3)).*referenceVelocity(:,1)+sin(reference(:,3)).*referenceVelocity(:,2), ...
    -sin(reference(:,3)).*referenceVelocity(:,1)+cos(reference(:,3)).*referenceVelocity(:,2)];

% Hold the saved data, gains, information and common initial state fixed.
% Replace one input over frames 60:95 only. Reference values are diagnostics.
interval=60:95; variants=["reproduced_baseline","reference_lidar_xy_60_95", ...
    "reference_lidar_yaw_60_95","reference_lateral_velocity_60_95"];
controlRows=cell(numel(variants),5);controls=cell(numel(variants),1);
for j=1:numel(variants)
    data=saved.data; lateral=saved.lateral;
    if j==2,data.lidar.pose(interval,1:2)=reference(interval,1:2);end
    if j==3,data.lidar.pose(interval,3)=reference(interval,3);end
    if j==4
        % Isolate the lateral-velocity input. Retain the saved sideslip-rate
        % input; this is not a complete, physically reconstructed motion model.
        lateral.lateralVelocity(interval)=referenceBody(interval,2);
    end
    controls{j}=runFullLocalizationObserver(data,saved.lateralDesign,saved.cfg,LateralInputs=lateral);
    e=vecnorm(controls{j}.position-reference(:,1:2),2,2);
    controlRows(j,:)={variants(j),e(index),sqrt(mean(e.^2)),max(e), ...
        rad2deg(wrap(controls{j}.heading(index)-reference(index,3)))};
end
observerControls=cell2table(controlRows,VariableNames={'variant','frame94ErrorM','allFrameRmseM','allFrameMaximumM','frame94YawErrorDeg'});
writetable(observerControls,fullfile(destination,'observer_controls.csv'));

% Reconstruct exactly the current three-scan source. The saved dead-reckoning
% trajectory differs from wheel odometry only by a constant SE(2) transform;
% updateLocalizationSourceWindow uses invariant relative motion.
loaded=load('output/mississippi_mapping_synchronized/probability_cloud.mat','cloud');
fixed=registrationSupport.projectSemanticProbabilityCloud(loaded.cloud,2);
call=matching.report.calls(frameRow,:);
seed=[call.predictedX call.predictedY call.predictedPsi];
pose=[call.referenceX call.referenceY call.referencePsi];
fixed=subset(fixed,vecnorm(fixed.components.mean-seed(1:2),2,2)<=100);
cfg=matching.cfg; mapCfg=featureMapBuildConfig();
frameIndices=(frame-cfg.sourceWindow.maximumFrames+1):frame;
poses=readFramePoseTable(fullfile(root,'data',mapCfg.poseMatchCsvPath),frameIndices);
history=[];
for j=1:numel(frameIndices)
    f=frameIndices(j);input=loadPointCloudFrame(fullfile(root,'data',mapCfg.pointCloudMatPath),f);
    [~,tilt]=poseRowToPlanarPose(poses(j,:));cfg.perception.coarseProbabilityCloud.projectionRotation=tilt;
    current=perceiveCoarseProbabilityCloud(input,cfg.perception);
    row=matching.report.deadReckoning(f,:);
    [source,history,window]=updateLocalizationSourceWindow(current,matching.report.calls.timeSeconds(f), ...
        [row.x row.y row.psi],history,cfg.sourceWindow);
end
results=cell(16,1);names=["three_scan_recorded_seed","three_scan_reference_seed", ...
    "single_scan_recorded_seed","single_scan_reference_seed", ...
    "fine_single_recorded_seed","fine_single_reference_seed", ...
    "three_scan_without_curb","three_scan_without_pole","three_scan_without_sign", ...
    "three_scan_gnss_xy_seed","uniform_map_priors_recorded_seed", ...
    "uniform_map_priors_reference_seed","association_gate_1m", ...
    "association_gate_0p5m","reference_window_motion_recorded_seed", ...
    "reference_window_motion_reference_seed"];
results{1}=registerSemanticProbabilityCloud(fixed,source,seed,cfg.registration);
results{2}=registerSemanticProbabilityCloud(fixed,source,pose,cfg.registration);
results{3}=registerSemanticProbabilityCloud(fixed,current,seed,cfg.registration);
results{4}=registerSemanticProbabilityCloud(fixed,current,pose,cfg.registration);
registrationReproduction=max(abs(results{1}.poseXYTheta-[call.x call.y call.psi]));
assert(registrationReproduction<1e-7 && results{1}.accepted);
loaded=load('output/mississippi_mapping_synchronized/feature_observations.mat','featureData');
fineConfig=load('output/saved_perception_inspva_20260915/experiment.mat','pcfg');
fine=buildSavedFeatureProbabilityCloud(loaded.featureData,frame,pose,fineConfig.pcfg);
results{5}=registerSemanticProbabilityCloud(fixed,fine,seed,cfg.registration);
results{6}=registerSemanticProbabilityCloud(fixed,fine,pose,cfg.registration);
classes=["curb","pole","trafficSign"];
for j=1:3
    reduced=subset(source,source.components.semanticName~=classes(j));
    results{j+6}=registerSemanticProbabilityCloud(fixed,reduced,seed,cfg.registration);
end
gnssSeed=[diagnostics.gnssPositionAtObserverPoint(index,:),seed(3)];
results{10}=registerSemanticProbabilityCloud(fixed,source,gnssSeed,cfg.registration);
flat=fixed;flat.components.mixtureWeight(:)=1/flat.components.numComponents;
results{11}=registerSemanticProbabilityCloud(flat,source,seed,cfg.registration);
results{12}=registerSemanticProbabilityCloud(flat,source,pose,cfg.registration);
for j=1:2
    gated=cfg.registration;gated.geometric.maximumMatchDistance=1/j;
    results{12+j}=registerSemanticProbabilityCloud(fixed,source,seed,gated);
end
referenceHistory=[];
for j=1:numel(frameIndices)
    f=frameIndices(j);row=matching.report.calls(f,:);
    [referenceSource,referenceHistory]=updateLocalizationSourceWindow(history.clouds{j},row.timeSeconds, ...
        [row.referenceX,row.referenceY,row.referencePsi],referenceHistory,cfg.sourceWindow);
end
results{15}=registerSemanticProbabilityCloud(fixed,referenceSource,seed,cfg.registration);
results{16}=registerSemanticProbabilityCloud(fixed,referenceSource,pose,cfg.registration);
rows=cell(numel(results),9);
for j=1:numel(results)
    r=results{j};delta=r.poseXYTheta-pose;
    rows(j,:)={names(j),r.accepted,r.reason,norm(delta(1:2)),rad2deg(wrap(delta(3))), ...
        r.similarity,r.observableRank,r.iterations,height(r.correspondences)};
end
matchingControls=cell2table(rows,VariableNames={'variant','accepted','reason','positionErrorM', ...
    'yawErrorDeg','similarity','rank','iterations','matches'});
writetable(matchingControls,fullfile(destination,'matching_controls.csv'));
writetable(results{1}.classDiagnostics,fullfile(destination,'class_diagnostics.csv'));

% Describe the actual accepted associations. Point-to-line curb residuals
% deliberately ignore displacement along the target tangent.
r=results{1};pairs=r.correspondences;means=source.components.mean(pairs.source,:);
fmeans=fixed.components.mean(pairs.target,:);candidate=transform(means,r.poseXYTheta);
atReference=transform(means,pose);normal=zeros(height(pairs),2);
for j=1:height(pairs)
    [v,e]=eig(fixed.components.covariance(:,:,pairs.target(j)),'vector');[~,minor]=min(e);
    normal(j,:)=v(:,minor).';
end
physical=vecnorm(candidate-fmeans,2,2);physicalReference=vecnorm(atReference-fmeans,2,2);
line=pairs.semanticName=="curb";
physical(line)=abs(sum((candidate(line,:)-fmeans(line,:)).*normal(line,:),2));
physicalReference(line)=abs(sum((atReference(line,:)-fmeans(line,:)).*normal(line,:),2));
pairs.physicalResidualM=physical;pairs.referencePoseResidualSamePairsM=physicalReference;
pairs.sourceX=means(:,1);pairs.sourceY=means(:,2);
pairs.targetRelativeX=fmeans(:,1)-pose(1);pairs.targetRelativeY=fmeans(:,2)-pose(2);
writetable(pairs,fullfile(destination,'accepted_correspondences.csv'));
classRows=cell(3,5);
for j=1:3
    keep=pairs.semanticName==classes(j);
    classRows(j,:)={classes(j),nnz(keep),numel(unique(pairs.target(keep))), ...
        median(physical(keep)),median(physicalReference(keep))};
end
classResiduals=cell2table(classRows,VariableNames={'semanticName','sourcePairs','uniqueMapTargets', ...
    'medianResidualAtCandidateM','medianResidualAtReferenceSamePairsM'});
writetable(classResiduals,fullfile(destination,'class_residuals.csv'));

rotation=[cos(pose(3)),-sin(pose(3));sin(pose(3)),cos(pose(3))];
lidarError=[call.x call.y]-pose(1:2);gnssError=diagnostics.gnssPositionAtObserverPoint(index,:)-reference(index,1:2);
summary=struct('frame',frame,'timeSeconds',time(index),'fusionMaximumM',maximumError, ...
    'fusionErrorXY',errorXY(index,:),'fusionErrorForwardLeftM',errorXY(index,:)*rotation, ...
    'rawLidarErrorM',norm(lidarError),'rawLidarErrorForwardLeftM',lidarError*rotation, ...
    'gnssMeasurementErrorM',norm(gnssError), ...
    'gnssOnlyObserverErrorM',norm(saved.runs{3}.estimate.position(index,:)-reference(index,1:2)), ...
    'lidarOnlyObserverErrorM',norm(saved.runs{2}.estimate.position(index,:)-reference(index,1:2)), ...
    'signedErrorContributionsXYM',squeeze(terms(index,:,:)), ...
    'contributionOrder',["lidar_position","gnss_position","realized_motion","initial"], ...
    'speedMps',saved.data.highRate.longitudinalSpeed(index),'yawRateRadPerSecond',saved.data.highRate.yawRate(index), ...
    'lateralVelocityMps',saved.lateral.lateralVelocity(index), ...
    'referenceDerivedBodyVelocityMps',referenceBody(index,:), ...
    'observerVelocityMps',estimate.velocity(index,:),'referenceDerivedVelocityMps',referenceVelocity(index,:), ...
    'lidarLearnedLateralBiasMps',diagnostics.lidarVelocityBias(index), ...
    'positionCorrections',diagnostics.positionCorrection(index,:), ...
    'minimumInformationWeights',diagnostics.minimumWeights(index,:), ...
    'coarseWindow',window,'coarseInitialPoseErrorM',norm(seed(1:2)-pose(1:2)), ...
    'matchingSimilarity',call.similarity,'matchingAcceptanceThreshold',cfg.registration.minimumSimilarity, ...
    'scaledInformationEigenvalues',r.curvatureEigenvalues, ...
    'observerReproductionMaxAbs',reproduction,'registrationReproductionMaxAbs',registrationReproduction, ...
    'decompositionMaxResidualM',decompositionResidual, ...
    'clockModelId',matching.report.metadata.clockModelId, ...
    'scope',"Same-drive map and INS reference; reference substitutions are offline diagnostics only. Fine labels are saved data, not production localization input.");
biasStart=find(time<=time(index)-saved.cfg.bias.window,1,'last');
summary.biasWindowMinimumSpeedMps=min(saved.data.highRate.longitudinalSpeed(biasStart:index));
summary.biasWindowRequiredMinimumSpeedMps=saved.cfg.bias.minimumSpeed;
writeJson(fullfile(destination,'summary.json'),summary);
neighbor=matching.report.calls(60:110,:);
neighbor.fusedErrorM=vecnorm(errorXY(60:110,:),2,2);
neighbor.gnssOnlyErrorM=vecnorm(saved.runs{3}.estimate.position(60:110,:)-reference(60:110,1:2),2,2);
neighbor.gnssMeasurementErrorM=vecnorm(diagnostics.gnssPositionAtObserverPoint(60:110,:)-reference(60:110,1:2),2,2);
writetable(neighbor,fullfile(destination,'neighborhood.csv'));
save(fullfile(output,'diagnostic.mat'),'summary','observerControls','matchingControls','controls', ...
    'results','fixed','source','fine','current','terms','pairs','cfg','-v7.3');

fig=figure('Visible','off','Position',[0 0 1200 850]);layout=tiledlayout(fig,2,2);
ax=nexttile(layout);plot(ax,neighbor.frame,100*[neighbor.positionErrorM,neighbor.fusedErrorM, ...
    neighbor.gnssOnlyErrorM,neighbor.gnssMeasurementErrorM],'LineWidth',1.3);
xline(ax,frame,'--');grid(ax,'on');xlabel(ax,'Frame');ylabel(ax,'Position error (cm)');
legend(ax,'Raw LiDAR','Fusion','GNSS-only observer','GNSS measurement','Location','northwest');
title(ax,'Error buildup and abrupt matching recovery');
ax=nexttile(layout);plot(ax,60:110,100*squeeze(terms(60:110,1,1:3)),'LineWidth',1.3);
xline(ax,frame,'--');grid(ax,'on');xlabel(ax,'Frame');ylabel(ax,'Signed map-X contribution (cm)');
legend(ax,'LiDAR position','GNSS position','Motion','Location','northwest');title(ax,'Exact realized-update decomposition');
ax=nexttile(layout);plot(ax,60:110,[saved.lateral.lateralVelocity(60:110),referenceBody(60:110,2)],'LineWidth',1.3);
xline(ax,frame,'--');grid(ax,'on');xlabel(ax,'Frame');ylabel(ax,'Body lateral velocity (m/s)');
legend(ax,'Lateral observer','Reference finite difference','Location','best');title(ax,'Motion discrepancy during the turn');
ax=nexttile(layout);colors=lines(3);hold(ax,'on');
for j=1:3
    selected=pairs.semanticName==classes(j);
    q=(candidate(selected,:)-pose(1:2))*rotation;f=(fmeans(selected,:)-pose(1:2))*rotation;
    plot(ax,q(:,1),q(:,2),'.','Color',colors(j,:),'DisplayName',classes(j)+" source");
    plot(ax,f(:,1),f(:,2),'o','Color',colors(j,:),'MarkerSize',4,'HandleVisibility','off');
    plot(ax,[q(:,1),f(:,1)].',[q(:,2),f(:,2)].','-','Color',[colors(j,:) .25],'HandleVisibility','off');
end
axis(ax,'equal');grid(ax,'on');xlabel(ax,'Reference forward (m)');ylabel(ax,'Reference left (m)');
legend(ax,'Location','best');title(ax,'Accepted pairs: dots=source, circles=map');
title(layout,'Mississippi frame 94: diagnosis, unchanged production settings');
exportgraphics(fig,fullfile(output,'diagnosis.png'),'Resolution',150);close(fig);
disp(observerControls);disp(matchingControls);disp(summary);

function cloud=subset(cloud,keep)
    c=cloud.components;c.mean=c.mean(keep,:);c.covariance=c.covariance(:,:,keep);
    for field=["semanticName","mixtureWeight","repeatability","semanticProbability","occupancyProbability","supportAmplitude"]
        if isfield(c,field),c.(field)=c.(field)(keep,:);end
    end
    c.numComponents=nnz(keep);cloud.components=c;
end

function points=transform(points,pose)
    r=[cos(pose(3)),-sin(pose(3));sin(pose(3)),cos(pose(3))];points=points*r.'+pose(1:2);
end

function value=wrap(value)
    value=atan2(sin(value),cos(value));
end

function writeJson(file,value)
    fid=fopen(file,'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(value,PrettyPrint=true));
end
