function cfg=localizationSourceWindowConfig()
% localizationSourceWindowConfig Causal coarse-distribution matching support.
% Five scans span about 0.4 s at 10 Hz. Motion comes from independent
% wheel/gyro odometry, never from previous map-registration corrections.
% Only distributions confirmed in at least two distinct scans are emitted.
% Association gates describe local odometry-aligned feature compatibility.
% Covariance Bhattacharyya distance tests shape before merging. Every earlier
% acquisition must agree; the pooled scatter cannot hide a conflicting shape.
% The variance floor is spatial regularization, not center/odometry noise.
    cfg=struct('maximumFrames',5,'maximumAgeSeconds',.45, ...
        'minimumDetectionFrames',2,'maximumAssociationDistance',.75, ...
        'maximumStandardizedDistance',3,'associationNoiseStandardDeviation',.10, ...
        'maximumShapeDistance',.5,'shapeVarianceFloor',.0004);
end
