function results = sweepDensitySupport()
% sweepDensitySupport: Compare metric pole supports against frozen 0.3 m output.
% The input lattice and published labels stay at 0.6 m. Only the metric
% density-core support changes. The CSVs retain frame IDs so tuning frames
% (1:20:1170) and interleaved validation frames can be scored separately.
    root = setupVehicleLocalization();
    addpath(fullfile(root,'research','coarse_lattice_20260924'));
    settings = [0.15 0.60 0.35 0.35; ...
        0.18 0.60 0.55 0.35; 0.20 0.60 0.55 0.40; ...
        0.20 0.60 0.45 0.40; 0.20 0.60 0.45 0.50; ...
        0.22 0.60 0.55 0.45; 0.25 0.60 0.55 0.50];
    frames = 1:10:1170;
    rows = struct([]);
    for k = 1:size(settings,1)
        label = sprintf('support%02d',k);
        captureCoarseSequence(label,frames,root,@(cfg) setSupport(cfg,settings(k,:)));
        summary = compareCoarseSequences(label,'baseline',frames,root);
        row = struct('label',string(label),'coreRadius',settings(k,1), ...
            'isolationRadius',settings(k,2),'minimumCoreFraction',settings(k,3), ...
            'minimumCoreIsolation',settings(k,4),'precision',summary.pole.precision, ...
            'recall',summary.pole.recall,'f1',summary.pole.f1, ...
            'f1Tol',summary.pole.f1Tol);
        rows = [rows; row]; %#ok<AGROW>
    end
    results = struct2table(rows);
    writetable(results,fullfile(root,'research','coarse_lattice_continuation_20260925','support_sweep.csv'));
    disp(results);
end

function cfg = setSupport(cfg,setting)
    % Freeze the takeover detector so future default changes cannot silently
    % change the interpretation of this support-radius experiment.
    cfg.offGroundFeatures.pole.minimumCorePoints = 0;
    cfg.offGroundFeatures.pole.minimumPointScore = 0.60;
    cfg.offGroundFeatures.pole.maximumLineScore = 0.90;
    cfg.offGroundFeatures.pole.coreRadius = setting(1);
    cfg.offGroundFeatures.pole.isolationRadius = setting(2);
    cfg.offGroundFeatures.pole.minimumCoreFraction = setting(3);
    cfg.offGroundFeatures.pole.minimumCoreIsolation = setting(4);
end
