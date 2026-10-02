function measurement = buildLidarLineMeasurement(points,mapPoints,normals,pose,weights)
% buildLidarLineMeasurement Frozen planar point-to-line residuals at prediction.
% Rows are deskewed vehicle-frame scan XY, associated map XY, and map normals.
% Extrinsics must already be applied. Weights include uncertainty and robust
% reliability, with a vector for independent residuals or full PSD precision
% for correlated ones. Associations, map, calibration and tilt are external
% assumptions; this routine does not estimate nuisance variables or covariance.
    arguments
        points (:,2) double {mustBeReal,mustBeFinite}
        mapPoints (:,2) double {mustBeReal,mustBeFinite}
        normals (:,2) double {mustBeReal,mustBeFinite}
        pose (3,1) double {mustBeReal,mustBeFinite}
        weights double {mustBeReal,mustBeFinite}
    end
    assert(size(points,1)==size(mapPoints,1) && isequal(size(points),size(normals)) ...
        && ~isempty(points) && all(abs(vecnorm(normals,2,2)-1)<1e-8), ...
        'VehicleLocalization:InvalidLidarGeometry','Require aligned points and unit map normals.');
    R=[cos(pose(3)),-sin(pose(3));sin(pose(3)),cos(pose(3))];
    transformed=points*R.'+pose(1:2).';
    derivative=points*(R*[0,-1;1,0]).';
    measurement=struct('residual',sum(normals.*(transformed-mapPoints),2), ...
        'jacobian',[normals,sum(normals.*derivative,2)],'weights',weights, ...
        'linearizationPose',pose);
end
