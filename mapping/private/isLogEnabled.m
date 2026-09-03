function tf = isLogEnabled(cfg)
% isLogEnabled: Resolve the command-window diagnostic log switch
% from the test configuration, defaulting to false when the field is absent.
%
% Input:
%   cfg: test configuration struct
%
% Output:
%   tf: logical scalar indicating whether step logs are enabled
    tf = false;
    if isstruct(cfg) && isfield(cfg, "stepLogsEnabled") && isscalar(cfg.stepLogsEnabled)
        tf = logical(cfg.stepLogsEnabled);
    end
end
