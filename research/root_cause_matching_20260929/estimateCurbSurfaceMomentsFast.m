function [moments,details]=estimateCurbSurfaceMomentsFast(ground,context,rotation,translation)
% estimateCurbSurfaceMomentsFast Reproduce the deployed batched surface model.
    [moments,details]=estimateCurbBoundaryGeometry(ground,context,rotation,translation);
end
