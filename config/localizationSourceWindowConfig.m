function cfg=localizationSourceWindowConfig()
% localizationSourceWindowConfig Causal coarse-distribution matching support.
% Five scans span about 0.4 s at 10 Hz. Motion comes from independent
% wheel/gyro odometry, never from previous map-registration corrections.
% Only distributions confirmed in at least two distinct scans are emitted.
% Association gates describe local odometry-aligned feature compatibility.
    cfg=struct('maximumFrames',5,'maximumAgeSeconds',.45, ...
        'minimumDetectionFrames',2,'maximumAssociationDistance',.75, ...
        'maximumStandardizedDistance',3,'associationNoiseStandardDeviation',.10);
end
