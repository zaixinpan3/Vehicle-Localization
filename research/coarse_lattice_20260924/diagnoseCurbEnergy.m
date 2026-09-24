function diagnoseCurbEnergy(frameIndex, window, root)
% diagnoseCurbEnergy: Render curb energy maps of both lattices inside an XY window.
    if nargin < 3, root = '/home/zai/Downloads/ResearchProjects/vehicleLocalization'; end
    out = fullfile(root, 'output', 'coarse_lattice_20260924', 'frames');
    frame = loadPointCloudFrame(fullfile(root, 'data', 'raw', 'MissisipiPointClouds.mat'), frameIndex);
    cfgs = {perceptionConfig("Mississippi", "offline"), perceptionConfig()};
    names = ["heightStepMeters", "roughnessMeters", "residualSlope", "curvature", "totalBase", "linearity", "total", "extractedMask"];
    fig = figure('Visible', 'off', 'Position', [0 0 2400 700]);
    for k = 1:2
        cfg = cfgs{k}; cfg.coarseProbabilityCloud.storeDiagnostics = true;
        p = perceiveFrame(frame, cfg); g = p.diagnostics.ground; m = g.energyMaps;
        x = g.cellOrigin(1) + ((1:size(m.total, 2)) - 0.5) * g.cellSize(1);
        y = g.cellOrigin(2) + ((1:size(m.total, 1)) - 0.5) * g.cellSize(2);
        for j = 1:numel(names)
            subplot(2, numel(names) + 1, (k - 1) * (numel(names) + 1) + j);
            v = double(m.(names(j))); v(~isfinite(v)) = 0;
            imagesc(x, y, v); axis xy equal; xlim(window(1:2)); ylim(window(3:4));
            if contains(names(j), "Meters"), clim([0 0.15]); elseif names(j) == "extractedMask", clim([0 1]); else, clim([0 1]); end
            title(sprintf('%s %.1f m', names(j), g.cellSize(1)), 'Interpreter', 'none'); colorbar;
        end
        subplot(2, numel(names) + 1, k * (numel(names) + 1));
        imagesc(x, y, double(g.curbCellMask) + 0.5 * double(g.roadCellMask)); axis xy equal; xlim(window(1:2)); ylim(window(3:4));
        title(sprintf('curb+road %.1f m', g.cellSize(1))); colorbar;
    end
    png = fullfile(out, sprintf('frame%04d_energy.png', frameIndex));
    exportgraphics(fig, png, 'Resolution', 90); close(fig); fprintf('Saved %s\n', png);
end
