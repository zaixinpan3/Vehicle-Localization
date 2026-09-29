function cfg=landmarkViewMapConfig()
% landmarkViewMapConfig Offline point-landmark view distributions for XY matching.
% Kernel distances are map meters; headings are radians. Coverage scales
% influence continuously, independently of the normalized conditional mean.
% Scatter and support are engineering geometry measures, not pose covariance
% or calibrated detection probabilities. Runtime uses stored map statistics
% and the predicted viewing origin, without online fine labels/reference poses.
    cfg=struct('pointClasses',["pole","trafficSign"], ...
        'mergeRadius',1.5,'maximumAssignmentDistance',1.5,'minimumObservationPoints',3, ...
        'bandwidth',5,'coverageScale',5,'varianceFloor',.0025, ...
        'maximumHeadingDifference',deg2rad(30));
end
