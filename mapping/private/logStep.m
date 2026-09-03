function logStep(cfg, logKey, message, varargin)
% logStep: Print one structured command-window diagnostic line for the
% map-building smoke test when cfg.stepLogsEnabled is true. The log is intended
% for agent-side inspection of algorithm flow and intermediate counts without
% writing persistent files.
%
% Input:
%   cfg: test configuration struct with optional stepLogsEnabled field
%   logKey: string scalar diagnostic key
%   message: sprintf-compatible message format
%   varargin: values consumed by the message format
%
% Output:
%   none
    if ~isLogEnabled(cfg)
        return;
    end
    fprintf("[map-test][%s] %s\n", char(string(logKey)), sprintf(char(string(message)), varargin{:}));
end
