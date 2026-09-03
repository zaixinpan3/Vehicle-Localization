function values = clamp01(values)
% clamp01: Clamp numeric values into the unit interval [0, 1] while
% converting non-finite entries to zero.
%
% Input:
%   values: numeric scalar, vector, or array
%
% Output:
%   values: numeric array with finite values in [0, 1]
    values = double(values);
    values(~isfinite(values)) = 0;
    values = min(max(values, 0), 1);
end
