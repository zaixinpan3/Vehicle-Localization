function cfg=downtownReferenceReplayConfig()
% downtownReferenceReplayConfig Fixed protocol for independent-reference maps.
% Alternate covered acquisitions for mapping/query; keep original frame IDs.
% The fixed local-origin prior is declared, not read from evaluation truth.
% Moving-start records lack stationary tilt alignment: sensor tilt is disabled.
    cfg=struct('featureNames',["curb","pole","trafficSign","facade"], ...
        'initialPose',[.5,-.4,deg2rad(2)],'motionStepSeconds',.01, ...
        'mapFrameParity',1,'queryFrameParity',0,'mapThinningResolutionM',.1, ...
        'registration',distributionRegistrationConfig(), ...
        'sourceWindow',localizationSourceWindowConfig(),'observer',fullObserverConfig());
    cfg.tilt=lidarImuTiltConfig();cfg.tilt.mode="disabled";
    cfg.referenceKind="pseudo_ground_truth";
    cfg.evaluation="Same-drive interleaved disjoint acquisitions; neighboring observations and reference errors remain correlated";
end
