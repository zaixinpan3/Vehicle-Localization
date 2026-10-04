function value = latticeTunedValue(spacing, fineValue, coarseValue)
% latticeTunedValue: Select the parameter tuned for one pillar lattice.
% Perception stages are tuned on exactly two lattices: 0.3 m pillars
% (including Downtown coarse) and 0.6 m pillars (see pillarGridConfig). Cell-count
% and density parameters cannot be rescaled by a formula, so each config
% states both tuned values side by side and this helper picks one.
    assert(isscalar(spacing) && isfinite(spacing), "perception:UnsupportedPillarSpacing", ...
        "Pillar spacing must be a finite scalar.");
    if abs(spacing - 0.3) < 1e-9
        value = fineValue;
    elseif abs(spacing - 0.6) < 1e-9
        value = coarseValue;
    else
        error("perception:UnsupportedPillarSpacing", ...
            "Perception parameters are tuned for 0.3 m and 0.6 m pillars, not %.3g m.", spacing);
    end
end
