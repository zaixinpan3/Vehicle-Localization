function cfg=localizationSourceWindowConfig()
% localizationSourceWindowConfig Causal coarse-distribution matching support.
% Three scans span about 0.2 s at 10 Hz. Motion comes from independent
% wheel/gyro odometry, never from previous map-registration corrections.
% Only distributions confirmed in at least two distinct scans are emitted.
% Association gates describe local odometry-aligned feature compatibility.
    cfg=struct('maximumFrames',3,'maximumAgeSeconds',.25, ...
        'minimumDetectionFrames',2,'maximumAssociationDistance',.75, ...
        'maximumStandardizedDistance',3,'associationNoiseStandardDeviation',.10);
end
