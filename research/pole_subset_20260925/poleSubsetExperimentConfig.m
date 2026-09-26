function cfg=poleSubsetExperimentConfig(cfg)
% poleSubsetExperimentConfig: Enable the experimental 0.6 m subset detector.
% Recorded agreement remains below the migration target, so this experiment
% is explicit and does not replace the default perception configuration.
    assert(all(abs(cfg.voxel.voxelSize-[.6 .6])<1e-12), ...
        'perception:SubsetExperimentSpacing','This experiment requires the 0.6 m coarse lattice.');
    cfg.offGroundFeatures.pole.detector="subset";
    cfg.offGroundFeatures.pole.probabilityEvidence="subset";
end
