function enabled = isFacadeDetectionEnabled(cfg)
% isFacadeDetectionEnabled: Resolve whether facade detection
% should run for the current scenario from the off-ground processing
% configuration.
%
% Input:
%   cfg: off-ground processing configuration struct with optional
%       facadeDetectionEnabled field
%
% Output:
%   enabled: logical scalar true when facade Hough/refinement should run
    enabled = false;
    if isfield(cfg, "facadeDetectionEnabled") && ~isempty(cfg.facadeDetectionEnabled)
        candidate = cfg.facadeDetectionEnabled;
        if (islogical(candidate) || isnumeric(candidate)) && isscalar(candidate)
            enabled = logical(candidate);
        end
    end
end
