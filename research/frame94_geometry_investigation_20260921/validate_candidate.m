% validate_candidate Check held-out queries and unchanged pillar selections.
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
previous=pwd;cleanup=onCleanup(@()cd(previous));cd(root);setupVehicleLocalization();
dest=fileparts(mfilename('fullpath'));out=fullfile(root,'output/frame94_geometry_investigation_20260921');
study=load(fullfile(out,'geometry.mat'),'offset','baseline','summary');
maps=load(fullfile(out,'map_control.mat'),'clouds');
loadInput=load('output/receiver_clock_20260921/coarse_pipeline/matching/report.mat');calls=loadInput.report.calls;motion=loadInput.report.deadReckoning;
data=load('output/mississippi_mapping_synchronized/feature_observations.mat','featureData');data=data.featureData;
queries=[76 87 91 94 97 103 112];assert(isempty(intersect(queries,study.summary.trainingFrames)));
rows=cell(0,9);selectionChecks=cell(0,5);savedResults=cell(numel(queries),3);
for iq=1:numel(queries)
    q=queries(iq);history={[],[],[]};reference=calls{q,{'referenceX','referenceY','referencePsi'}};
    seed=calls{q,{'predictedX','predictedY','predictedPsi'}};
    for f=q-2:q
        input=loadPointCloudFrame('data/raw/MissisipiPointClouds.mat',f);
        [~,tilt]=poseRowToPlanarPose(data.framePoseTable(f,:));cfg=study.baseline.cfg;
        cfg.perception.coarseProbabilityCloud.projectionRotation=tilt;
        base=perceiveCoarseProbabilityCloud(input,cfg.perception);
        calibration=cfg.perception.frameCalibration;calibration.translation=[study.offset.',0];
        calibration.identifier="diagnosticTranslationFitExcluding94";cfg.perception.frameCalibration=calibration;
        calibrated=perceiveCoarseProbabilityCloud(input,cfg.perception);
        countMatch=isequal(base.sourceSummary.selectedHitCount,calibrated.sourceSummary.selectedHitCount);
        cellMatch=isequal(base.sourceSummary.selectedSourceCellCount,calibrated.sourceSummary.selectedSourceCellCount);
        assert(countMatch && cellMatch,'Calibration must preserve classifier selections.');
        selectionChecks(end+1,:)={q,f,countMatch,cellMatch,calibrated.components.numComponents-base.components.numComponents}; %#ok<SAGROW>
        % Control the phase of the secondary Gaussian aggregation grid by
        % translating existing components instead of aggregating them anew.
        shifted=base;shift=(tilt*[study.offset;0]).';
        shifted.components.mean=shifted.components.mean+shift(1:2);
        shifted.components.meanXYZ=shifted.components.meanXYZ+shift;
        shifted.frameCalibration=calibration;
        sources={base,calibrated,shifted};
        for variant=1:3
            [matchingSources{variant},history{variant}]=updateLocalizationSourceWindow(sources{variant},calls.timeSeconds(f), ...
                motion{f,{'x','y','psi'}},history{variant},cfg.sourceWindow); %#ok<SAGROW>
        end
    end
    for variant=1:3
        mapIndex=1+(variant>1);
        r=registerSemanticProbabilityCloud(maps.clouds{mapIndex},matchingSources{variant},seed,cfg.registration);
        savedResults{iq,variant}=r;d=r.poseXYTheta-reference;
        reproduction=NaN;
        if variant==1,reproduction=max(abs(r.poseXYTheta-calls{q,{'x','y','psi'}}));end
        rows(end+1,:)={q,variant,r.accepted,r.reason,norm(d(1:2)),d(1),d(2),r.similarity,reproduction}; %#ok<SAGROW>
    end
end
results=cell2table(rows,VariableNames={'query','variant','accepted','reason','positionErrorM','errorX','errorY','similarity','baselineReproductionMaxAbs'});
selection=cell2table(selectionChecks,VariableNames={'query','sourceFrame','sameHitCounts','samePillarCounts','componentCountChange'});
writetable(results,fullfile(dest,'heldout_queries.csv'));writetable(selection,fullfile(dest,'perception_selection_checks.csv'));
save(fullfile(out,'heldout_queries.mat'),'results','selection','savedResults');disp(results);

% Trace one mapped traffic-sign distribution back to actual observations.
% A fixed 2.5 m gate around a selected accepted-map component is a diagnostic
% neighborhood, not a ground-truth object identity annotation.
full=load('output/mississippi_mapping_synchronized/probability_cloud_map.mat','probabilityCloudMap');
map=full.probabilityCloudMap.canonicalMap;original=temporalMapToProbabilityCloud(full.probabilityCloudMap);
pairTable=study.baseline.results{1}.correspondences;
target=125;center=study.baseline.fixed.components.mean(target,:);
[~,fullId]=ismember(center,original.components.mean,'rows');
label=original.components.semanticName(fullId);layer=map.layers(map.classLabels==label);
componentId=erase(original.components.componentId(fullId),label+":");component=layer.components(layer.componentIds==componentId);
tile=layer.tiles(all(vertcat(layer.tiles.ownerTile)==component.ownerTile,2));
% Stored block IDs start at zero in buildSlidingWindowMap.
blockFrames=str2double(tile.observationBlockIds)+1;counts=component.blockEffectiveCounts;
provenance=table(blockFrames,counts,VariableNames={'frame','effectiveCount'});
writetable(provenance,fullfile(dest,'example_component_frame_support.csv'));
selectedFrames=[70 80 90 94 100 110];colors=turbo(numel(selectedFrames));
fig=figure('Visible','off','Position',[0 0 1300 850]);layout=tiledlayout(fig,2,2);
for corrected=0:1
    ax=nexttile(layout,corrected+1);hold(ax,'on');
    for j=1:numel(selectedFrames)
        f=selectedFrames(j);p=data.pointsByFeatureFrame{3,f};keep=vecnorm(p(:,1:2)-center,2,2)<2.5;p=p(keep,:);
        [R,~]=poseRowToRigidTransform(data.framePoseTable(f,:));p=p+corrected*(R*[study.offset;0]).';
        plot(ax,p(:,1)-center(1),p(:,2)-center(2),'.','Color',colors(j,:),'DisplayName',"Frame "+f);
    end
    axis(ax,'equal');grid(ax,'on');xlabel(ax,'Map X relative to original component (m)');ylabel(ax,'Map Y relative to original component (m)');
    legend(ax,'Location','best');if corrected,title(ax,'Same observations with effective offset');else,title(ax,'Original zero-translation projection');end
end
ax=nexttile(layout,3);hold(ax,'on');
for variant=1:3
    r=results(results.variant==variant,:);plot(ax,r.query,100*r.positionErrorM,'o-','LineWidth',1.2);
end
grid(ax,'on');xlabel(ax,'Held-out query frame');ylabel(ax,'Coarse matching error (cm)');
legend(ax,'Original map','Offset: rebuilt cloud','Offset: fixed Gaussian partition','Location','best');
title(ax,'Same recorded seeds and matching parameters');
ax=nexttile(layout,4);bar(ax,[92.7991,13.9882,8.8015]);xticklabels(ax,{'Original 3 scans','Offset 3 scans','Offset 1 scan'});
ylabel(ax,'Frame 94 error (cm)');grid(ax,'on');title(ax,'Coordinate-transform control');
title(layout,'Frame 94: reference-point inconsistency during a turn');
exportgraphics(fig,fullfile(out,'geometry_diagnosis.png'),'Resolution',150);close(fig);
summary=struct('queries',queries,'queryFramesExcludedFromFit',true,'sourcePillarAndHitCountsIdentical',true, ...
    'maximumBaselineReproductionError',max(results.baselineReproductionMaxAbs,[],'omitnan'), ...
    'gaussianPartitionMayChange',any(selection.componentCountChange~=0), ...
    'exampleComponentId',original.components.componentId(fullId),'exampleComponentRepeatability',component.repeatability, ...
    'exampleComponentFirstSupportedFrame',min(blockFrames(counts>.5)), ...
    'exampleComponentLastSupportedFrame',max(blockFrames(counts>.5)), ...
    'qualification',"Held-out query frames within the same drive and local map, not independent-route validation. Existing map includes query observations.");
fid=fopen(fullfile(dest,'candidate_validation.json'),'w');assert(fid>=0);fileCleanup=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(summary,PrettyPrint=true));disp(summary);
