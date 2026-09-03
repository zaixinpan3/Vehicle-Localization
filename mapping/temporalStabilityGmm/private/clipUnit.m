function value = clipUnit(value)
% clipUnit: Validate numeric support values against the closed unit
% interval, allowing only harmless round-off at the boundary.
%
% Input:
%   value: numeric support value or array
%
% Output:
%   value: numeric support value or array after boundary round-off correction
    if any(~isfinite(value), "all")
        error("buildTemporalStabilityGmmMap:InvalidUnitValue", ...
            "Support and reliability quantities must be finite.");
    end
    lowerMask = value < 0;
    upperMask = value > 1;
    if any(value(lowerMask) < -1.0e-12, "all") || any(value(upperMask) > 1 + 1.0e-12, "all")
        error("buildTemporalStabilityGmmMap:UnitValueOutOfRange", ...
            "Support and reliability quantities must lie in [0, 1].");
    end
    value(lowerMask) = 0;
    value(upperMask) = 1;
end
