function runPointObjectMap()
% runPointObjectMap Refit point-landmark priors without extended-curve drift.
    setupVehicleLocalization();out='output/root_cause_matching_20260929';
    a=load('output/mississippi_mapping_calibrated/feature_observations.mat','featureData');featureData=a.featureData;
    selected=ismember(string(featureData.featureNames),["pole","trafficSign"]);featureData.featureNames=featureData.featureNames(selected);featureData.pointsByFeatureFrame=featureData.pointsByFeatureFrame(selected,:);
    cfg=featureMapBuildConfig();cfg.featureNames=string(featureData.featureNames);cfg.logEnabled=true;
    cfg.temporalMap.frameCalibration=featureData.frameCalibration;
    for k=1:numel(cfg.temporalMap.classParams)
        if ismember(cfg.temporalMap.classParams(k).classLabel,["pole","trafficSign"])
            cfg.temporalMap.classParams(k).elongatedAnisotropyThreshold=1e9;
        end
    end
    map=buildSlidingWindowMap(featureData,cfg);
    old=load('output/mississippi_mapping_calibrated/probability_cloud_map.mat');names=fieldnames(old);complete=old.(names{1});
    if isfield(complete,'canonicalMap'),canonical=complete.canonicalMap;else,canonical=complete;end
    for k=1:numel(map.canonicalMap.layers)
        at=string({canonical.layers.classLabel})==map.canonicalMap.layers(k).classLabel;canonical.layers(at)=map.canonicalMap.layers(k);
    end
    canonical.config=cfg.temporalMap;cloud=temporalMapToProbabilityCloud(canonical);file=fullfile(out,'pointObjectMap_cloud.mat');save(file,'cloud','canonical','cfg','-v7.3');
    rootCauseReplay('output/pole_boundary_recovery_20260929/replay.mat',"pointObjectMap",distributionRegistrationConfig(),file);
end
