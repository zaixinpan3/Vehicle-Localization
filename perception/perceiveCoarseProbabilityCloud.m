function [probabilityCloud, diagnostics] = perceiveCoarseProbabilityCloud(frame, cfg)
% perceiveCoarseProbabilityCloud: Run the fast pillar-only perception path
% and return a sparse semantic cloud with XYZ statistics and an XY marginal. Curb and marking
% decisions are made on ground cells, pole decisions on vertical columns,
% and no point-level semantic refinement or dense semantic volume is built.
%
% Input:
%   frame: organized point-cloud struct with x, y, z, and optional scalar
%       return channels
%   cfg: optional struct from perceptionConfig
%
% Output:
%   probabilityCloud: sparse 2D semantic NDT probability cloud
%   diagnostics: optional coarse cell maps used for regression analysis
    if nargin < 2 || isempty(cfg)
        cfg = perceptionConfig();
    end
    if ~isfield(cfg, "coarseProbabilityCloud") || ...
            ~isstruct(cfg.coarseProbabilityCloud)
        cfg.coarseProbabilityCloud = coarseSemanticProbabilityCloudConfig();
    end
    cfg.executionMode = "coarseProbabilityCloud";
    cfg.coarseProbabilityCloud.storeDiagnostics = nargout > 1;

    result = perceiveFrame(frame, cfg);
    probabilityCloud = result.probabilityCloud;
    probabilityCloud.sourceSummary.frame = result.sourceSummary;
    diagnostics = struct();
    if nargout > 1 && isfield(result, "diagnostics")
        diagnostics = result.diagnostics;
    end
end
