function cfg=fullObserverConfig()
% fullObserverConfig Synchronous frame-rate GNSS/LiDAR and motion observer.
% Source alignment is offline bracket interpolation with declared wait.
% Measurement channels have zero perception delay; no pose transport occurs.
% GNSS corrects position only. The heading is the integrated gyro, corrected
% by the full Lyapunov-matched LiDAR channel, including translation-yaw terms.
    cfg=motionAidedObserverConfig();
    cfg.kind="full-synchronous-route-a-v5";
    cfg.timing="synchronous";
    cfg.synchronization=struct('gnssMaximumBracket',.15,'motionMaximumBracket',.025);
    cfg.gnss=struct('positionGain',1,'gainInformationScale',16, ...
        'minimumPositionWeight',.25,'maximumAge',.2);
    % Generic input already uses the observer point. Dataset adapters may
    % supply a separately calibrated receiver-minus-observer offset.
    cfg.gnss.outputPoint=struct('bodyOffset',[0;0],'bodyCovariance',zeros(2), ...
        'headingStdRad',0,'identifier',"coincident-output-points");
    cfg.lidar.maximumAge=.2;
    % Continuous seven-state ISS design domain, before numerical integration.
    % Bounds concern true speed/acceleration and the filtered LiDAR matrix S.
    % They are assumptions, not clipping limits or artificial information.
    cfg.iss=struct('maximumSpeed',15,'maximumAcceleration',2, ...
        'minimumPositionStrength',.25,'minimumHeadingStrength',.97, ...
        'maximumPositionHeadingCoupling',.06,'decayRate',.01, ...
        'solver',"sedumi",'tolerance',1e-8);
    cfg.initialHeading=0;
end
