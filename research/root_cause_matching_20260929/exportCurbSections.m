function exportCurbSections()
% exportCurbSections Inspect measured transverse height profiles and fine labels.
    root=setupVehicleLocalization();dest='output/root_cause_matching_20260929';cfg=rootCausePerceptionConfig();cfg.coarseProbabilityCloud.storeDiagnostics=true;
    mc=featureMapBuildConfig();frames=[178 854 1096];poses=readFramePoseTable(fullfile(root,'data',mc.poseMatchCsvPath),frames);
    for j=1:numel(frames)
        k=frames(j);raw=loadPointCloudFrame(fullfile(root,'data',mc.pointCloudMatPath),k);[~,tilt]=poseRowToPlanarPose(poses(j,:));cfg.coarseProbabilityCloud.projectionRotation=tilt;p=perceiveFrame(raw,cfg);
        context=p.diagnostics.groundPointContext;xyz=context.groundPoints*p.probabilityCloud.projectionRotation.'+p.probabilityCloud.projectionTranslation;
        for w=1:4
            a=load(sprintf('output/fine_matching_20260919/inputs_%d.mat',w),'frames','selectedIndices','cfg');at=find(a.frames==k);
            if ~isempty(at),ref=a.selectedIndices{at,string(a.cfg.featureNames)=="curb"};break;end
        end
        fine=ismember(context.groundOriginalPointIdx,ref);t=table(xyz(:,1),xyz(:,2),xyz(:,3),fine,VariableNames={'x','y','z','fineCurb'});
        writetable(t,fullfile(dest,"ground_section_"+k+".csv"));
    end
end
