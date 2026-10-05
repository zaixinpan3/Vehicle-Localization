function cfg=downtownReferenceReplayConfig()
% downtownReferenceReplayConfig Fixed protocol for independent-reference maps.
% Alternate covered acquisitions for mapping/query; keep original frame IDs.
% One reference pose initializes the first query; tracking has no reference.
% Moving-start records lack stationary tilt alignment: sensor tilt is disabled.
    cfg=struct('featureNames',["curb","pole","trafficSign","facade"], ...
        'initializationMode',"reference_pose_once",'motionStepSeconds',.01, ...
        'mapFrameParity',1,'queryFrameParity',0,'mapThinningResolutionM',.1, ...
        'registration',distributionRegistrationConfig(), ...
        'sourceWindow',localizationSourceWindowConfig(),'observer',fullObserverConfig());
    cfg.tilt=lidarImuTiltConfig();cfg.tilt.mode="disabled";
    cfg.referenceKind="pseudo_ground_truth";
    cfg.evaluation="Same-drive interleaved disjoint acquisitions; neighboring observations and reference errors remain correlated";
end
