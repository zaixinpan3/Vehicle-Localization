function [A, C] = evaluateLateralModel(model, rho)
% evaluateLateralModel: Evaluate the LPV lateral model at one scheduling
% point. The two components of rho are treated as independent coordinates,
% because the polytope vertices used by the synthesis do not all lie on the
% physical curve rho = [Vx; 1/Vx].
%
% Input:
%   model: struct from lateralBicycleModel
%   rho: [2 x 1] scheduling point [rho1; rho2]
%
% Output:
%   A: [2 x 2] state matrix at rho
%   C: [2 x 2] output matrix at rho
    rho = double(rho(:));
    assert(numel(rho) == 2 && all(isfinite(rho)), "rho must be a finite [2 x 1] scheduling point.");
    A = model.A0 + (rho(1) .* model.A1) + (rho(2) .* model.A2);
    C = model.C0 + (rho(2) .* model.C2);
end
