function runPerceptionThresholds()
% runPerceptionThresholds Audit lower learned ranks before promoting a profile.
    root=setupVehicleLocalization();addpath('research/pillar_fine_alignment_20260926');dest=fileparts(mfilename('fullpath'));
    base=load('output/pole_boundary_recovery_20260929/replay.mat','currentClouds');
    predictions=readtable('research/pole_boundary_recovery_20260929/expected.csv');
    oldMetrics=readtable('research/pole_boundary_recovery_20260929/deployed_pole_replay.csv');
    refs=load('output/pole_precision_20260927/mississippi.mat','references','frames');
    c=load('output/line_direction_matching_20260928/production/report.mat','report');calls=c.report.calls;
    o=load('output/line_direction_matching_20260928/sources.mat','motion');
    mc=featureMapBuildConfig();poses=readFramePoseTable(fullfile(root,'data',mc.poseMatchCsvPath),1:1170);
    wc=localizationSourceWindowConfig();summaries=table();
    for threshold=[.87 .85 .82]
        cfg=perceptionConfig('Mississippi');original=cfg.offGroundFeatures.pole.distributionValidation.minimumScore;
        cfg.offGroundFeatures.pole.distributionValidation.minimumScore=threshold;
        frames=unique([predictions.frame(predictions.score>=threshold & predictions.score<original);685;820;856]);
        currentClouds=base.currentClouds;metrics=oldMetrics;
        for k=frames.'
            raw=loadPointCloudFrame(fullfile(root,'data',mc.pointCloudMatPath),k);[~,tilt]=poseRowToPlanarPose(poses(k,:));cfg.coarseProbabilityCloud.projectionRotation=tilt;
            p=perceiveFrame(raw,cfg);currentClouds{k}=p.probabilityCloud;
            ids=p.candidates.pillarIndices{p.candidates.semanticNames=="pole"};m=measureFinePoleAlignment(raw,refs.references{k}.pointIndices,ids,p.candidates.geometry);m.frame=k;
            metrics(k,:)=struct2table(m);
        end
        history=[];sources=cell(1170,1);
        for k=1:1170,[sources{k},history]=updateLocalizationSourceWindow(currentClouds{k},calls.timeSeconds(k),o.motion(k,:),history,wc);end
        label="pole"+round(100*threshold);sourceFile=fullfile('output/iterative_matching_20260929',label+"_sources.mat");save(sourceFile,'sources','currentClouds','cfg','frames','-v7.3');
        writetable(metrics,fullfile(dest,label+"_perception.csv"));summary=replayCloudVariant(sourceFile,label,baselineMatchingConfig());
        summary.poleSelected=sum(metrics.candidatePillarCount);summary.poleEmpty=sum(metrics.extraPillarCount);
        summary.poleCovered=sum(metrics.coveredFinePointCount);summary.poleFalseFraction=summary.poleEmpty/summary.poleSelected;
        summary.poleCoverage=summary.poleCovered/sum(metrics.finePointCountInRoi);summaries=[summaries;summary]; %#ok<AGROW>
        writetable(summaries,fullfile(dest,'pole_threshold_screen.csv'));disp(summary);
    end
end
