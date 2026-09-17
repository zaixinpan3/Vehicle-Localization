function [position,information]=correctGnssOutputPoint(position,information,yaw,alignment)
% correctGnssOutputPoint Express a receiver position at the observer point.
% bodyOffset is receiver minus observer point, in forward/left coordinates.
% An empirical output-point calibration is not a surveyed installation lever.
% Its covariance and declared heading uncertainty are conservative additions;
% unreported correlations with the receiver solution remain unknown.
    offset=alignment.bodyOffset(:);C=alignment.bodyCovariance;
    assert(numel(offset)==2 && all(isfinite(offset)) && isequal(size(C),[2,2]) && ...
        all(isfinite(C),'all') && norm(C-C.','fro')<1e-12 && min(eig(C))>=0 && ...
        isscalar(alignment.headingStdRad) && isfinite(alignment.headingStdRad) && alignment.headingStdRad>=0, ...
        'VehicleLocalization:InvalidGnssAlignment','Invalid output-point calibration.');
    if all(offset==0) && all(C==0,'all'),return;end
    R=[cos(yaw),-sin(yaw);sin(yaw),cos(yaw)];J=[0,-1;1,0];
    position=position-(R*offset).';
    covariance=information\eye(2)+R*C*R.'+alignment.headingStdRad^2*(R*J*offset)*(R*J*offset).';
    information=covariance\eye(2);information=(information+information.')/2;
end
