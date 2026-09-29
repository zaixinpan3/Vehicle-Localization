function inspectCurbStep(selectedFrames)
% inspectCurbStep Check local geometry recovery against diagnostic fine centers.
    if nargin<1,selectedFrames=[178 806 854];end
    root=setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));cfg=rootCausePerceptionConfig();cfg.coarseProbabilityCloud.storeDiagnostics=true;
    mc=featureMapBuildConfig();frames=selectedFrames;poses=readFramePoseTable(fullfile(root,'data',mc.poseMatchCsvPath),frames);
    for j=1:numel(frames)
        k=frames(j);raw=loadPointCloudFrame(fullfile(root,'data',mc.pointCloudMatPath),k);[~,tilt]=poseRowToPlanarPose(poses(j,:));cfg.coarseProbabilityCloud.projectionRotation=tilt;p=perceiveFrame(raw,cfg);
        [~,t]=estimateCurbStepMoments(p.diagnostics.ground,p.diagnostics.groundPointContext,p.probabilityCloud.projectionRotation,p.probabilityCloud.projectionTranslation);
        t.stepDx=t.dx;t.stepDy=t.dy;original=readtable(fullfile(dest,"curb_positions_"+k+".csv"));a=innerjoin(original,t,Keys='pillar',RightVariables={'stepDx','stepDy','stepHeight','explainedFraction','residualStd','accepted'});
        a.before=abs(a.fineY-a.wholeY);a.after=abs(a.fineY-a.wholeY-a.stepDy.*a.accepted);
        writetable(a,fullfile(dest,"curb_step_"+k+".csv"));disp(k);disp(a);
    end
end
