function inspectFrameTiming()
% inspectFrameTiming Audit native point timing and stored map-pose consistency.
    root=setupVehicleLocalization();mc=featureMapBuildConfig();raw=loadPointCloudFrame(fullfile(root,'data',mc.pointCloudMatPath),806);disp(fieldnames(raw));
    a=load('output/mississippi_mapping_calibrated/feature_observations.mat','featureData');disp(a.featureData.framePoseTable(806,:));disp(a.featureData.frameCalibration);
    b=readFramePoseTable(fullfile(root,'data',mc.poseMatchCsvPath),806);disp(b);
    fn=fieldnames(raw);
    for k=1:numel(fn)
        x=raw.(fn{k});if isnumeric(x),fprintf('%s size=%s min=%.12g max=%.12g\n',fn{k},mat2str(size(x)),min(x,[],'all'),max(x,[],'all'));end
    end
end
