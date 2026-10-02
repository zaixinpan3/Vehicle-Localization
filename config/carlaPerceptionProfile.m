function cfg=carlaPerceptionProfile(cfg)
% carlaPerceptionProfile: CARLA Town10HD values applied by perceptionConfig("Carla").
% The CARLA semantic LiDAR (CARLA 0.10.0, 64 channels, +14.2/-17.7 deg,
% 1024 columns, 10 Hz, 2.0 m above the road, 1.5 cm range noise added) has no
% intensity, so traffic signs are not requested. The profile starts from the
% Mississippi profile with three structural choices and then sets tuned values:
%   * the semantic precision forests are off: fitted on recorded Ouster
%     distributions, they rejected nearly all CARLA curb and facade proposals;
%   * the Mississippi pole distribution model is kept, with a lower operating
%     score (0.70 instead of 0.87);
%   * coarse facade evidence is the continuous vertical run of each pillar,
%     counted only for runs of at least 3.0 m that begin within 0.6 m of the
%     local ground, instead of raw return counts. This removes tree canopies,
%     fences and low clutter and keeps online facades consistent with the
%     mapped facades (offline planes of at least 2.5 m).
% Values were selected by seeded random search against CARLA semantic labels
% on 40 mapping-drive sweeps (curb: stage-2 sample 27; pole: stage-1 sample
% 131; coarse facade: stage-2 sample 57 with the span and ground-contact
% gate raised for map consistency; offline facade: sample 82) and checked
% on the held-out localization drives. Offline curb and pole keep the
% Mississippi offline values: their searches found no better operating point.
% Stable tree trunks are valid pole-like landmarks. The optional structure
% rejection experiment is disabled to preserve their localization support.
% See research/carla_town10_20261001/README.md.
    spacing=cfg.voxel.voxelSize(1);
    cfg.semanticPrecision.enabled=false;
    if abs(spacing-.6)<1e-12
        cfg.offGroundFeatures.pole.distributionValidation=pillarPoleDistributionConfig("mississippi");
        cfg.groundFeatures.curb=coarseCurb(cfg.groundFeatures.curb);
        cfg.offGroundFeatures.pole=coarsePole(cfg.offGroundFeatures.pole);
        cfg.offGroundFeatures=coarseFacade(cfg.offGroundFeatures);
    else
        [cfg.offGroundFeatures,cfg.fine]=offlineFacade(cfg.offGroundFeatures,cfg.fine);
        cfg.fine=offlinePoleStructure(cfg.fine);
    end
end

function c=coarseCurb(c)
% Curb energy on 0.6 m pillars. CARLA curbs are 10-15 cm smooth steps:
% every ground return counts, and the energy rests on the height step and
% on the within-pillar height spread of pillars that straddle the step.
    c.minPointsPerCell=1;
    c.heightStepMinMeters=0.065;c.heightStepTargetMeters=0.094;c.heightStepSigmaMeters=0.065;
    c.roughnessTargetMeters=0.048;c.roughnessSigmaMeters=0.017;
    c.residualSlopeTargetDeg=6.6;c.residualSlopeSigmaDeg=1.16;
    c.curvatureTarget=0.12;c.curvatureSigma=0.115;
    c.relativeHeightMinMeters=0.069;c.relativeHeightSaturatedMeters=0.19;
    c.weights.heightStep=0.43;c.weights.residualSlope=0.047;c.weights.curvature=0.097;
    c.weights.roughness=0.63;c.weights.relativeHeight=0.12;
    c.extractionEnergyThreshold=0.63;c.extractionBaseEnergyThreshold=0.26;
    c.extractionHeightStepMinMeters=0.016;c.extractionCenterEvidenceMin=0.49;
    c.linearityEnergyThreshold=0.24;
end

function p=coarsePole(p)
% Shaft proposal, continuous-support validation and distribution operating score.
    p.distributionValidation.minimumScore=0.70;
    p.shaft.minimumPoints=14;p.shaft.minimumHeight=1.27;p.shaft.minimumRobustHeight=1.24;
    p.shaft.maximumRadialRms=0.126;p.shaft.minimumScore=0.20;
    p.shaft.minimumRadialSignificance=2.1;p.shaft.minimumPeakContrast=1.15;
    p.validation.minimumSupportHeight=1.40;p.validation.minimumWindowFraction=0.78;
    p.validation.minimumIsolation=0.835;p.validation.maximumTilt=8.2;
    p.validation.maximumRms=0.138;p.validation.maximumStd=0.067;p.validation.minimumOwnerPoints=8;
end

function o=coarseFacade(o)
% Facade lines on 0.6 m pillars from continuous vertical support that
% reaches the ground. The 1.9 m span of the perception-only optimum gave
% 81-86 % precision but put facades on canopies the map does not contain.
    o.facadeContinuousSupportMinimumPoints=4;o.facadeContinuousSupportMinimumSpan=3.0;
    o.facadeMaximumBaseHeight=0.6;
    o.houghPeakThresholdRatio=0.17;o.maxHoughPeaks=28;
    o.facadeLineMinLengthVoxels=12;o.facadeLineFillGapVoxels=10;o.lineDistanceThresholdVoxels=0.61;
    o.maxLinesPerThetaCluster=14;o.maxThetaClusters=5;o.minAssignedPixels=6;
    o.supportGammaMin=0.34;o.supportQuantile=0.51;o.supportAbs=0.20;
    o.orientationToleranceDeg=26;o.facadeSupportRadiusCells=2;o.houghSuppressionSize=[8 8];
end

function fine=offlinePoleStructure(fine)
% Optional semantic-label experiment; disabled for localization by default.
% Enable with cfg.fine.poleStructureRejectionEnabled=true. This removes
% useful trunks as well as building edges, so use a separately rebuilt map.
% Remove tree trunks (canopy around the shaft top) and building edges
% (returns beside the shaft over its height) from the mapped poles, and
% clusters beyond 25 m, where building corners dominate the detections. On the
% held-out lap this raises pole/sign-post precision of the offline pole
% points from 78 to 98 % (Pole-label precision 43 to 54 %) and lowers
% instance recall within 30 m from 65 to 52 %. These label metrics do not
% measure geometric landmark validity or localization quality.
    fine.poleStructureRejectionEnabled=false;
    fine.poleClusterLinkMeters=0.5;fine.poleCanopyRadius=2.5;fine.poleCanopyHeight=3.0;
    fine.poleCanopyMinimumPoints=10;fine.poleCanopyMaximumSectors=5;fine.poleCanopyMinimumThickness=0.6;
    fine.poleMaximumAdjacentReturns=20;fine.poleMaximumRange=25;
end

function [o,fine]=offlineFacade(o,fine)
% Offline facade lines (copied into the fine structural branch) and plane
% validation: only planes at least 2.5 m tall are mapped as facades.
    o.houghPeakThresholdRatio=0.33;o.maxHoughPeaks=29;
    o.facadeLineMinLengthVoxels=17;o.facadeLineFillGapVoxels=17;
    o.maxLinesPerThetaCluster=10;o.maxThetaClusters=8;o.minAssignedPixels=14;
    o.supportGammaMin=0.68;o.supportQuantile=0.43;o.supportAbs=0.11;o.houghSuppressionSize=[22 22];
    fine.facadeMinimumPoints=15;fine.facadeMinimumHeight=2.48;fine.facadeMinimumLength=1.34;
    fine.facadeMaximumDistance=0.26;fine.facadeMaximumNormalAngleDegrees=26.8;
end
