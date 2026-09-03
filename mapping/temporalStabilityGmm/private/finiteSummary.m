function [minValue, medianValue, maxValue] = finiteSummary(values)
% finiteSummary: Compute min, median, and max over finite numeric values
% for compact map-builder diagnostic logging.
%
% Input:
%   values: numeric vector or array
%
% Output:
%   minValue: scalar minimum finite value or NaN
%   medianValue: scalar median finite value or NaN
%   maxValue: scalar maximum finite value or NaN
    values = double(values(:));
    values = values(isfinite(values));
    if isempty(values)
        minValue = NaN;
        medianValue = NaN;
        maxValue = NaN;
        return;
    end
    minValue = min(values);
    medianValue = median(values);
    maxValue = max(values);
end
