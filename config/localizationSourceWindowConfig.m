function cfg=localizationSourceWindowConfig()
% localizationSourceWindowConfig Causal coarse-distribution matching support.
% Three scans span about 0.2 s at 10 Hz. Motion comes from independent
% wheel/gyro odometry, never from previous map-registration corrections.
    cfg=struct('maximumFrames',3,'maximumAgeSeconds',.25);
end
