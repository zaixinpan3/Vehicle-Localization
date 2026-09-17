function cfg=fullObserverConfig()
% fullObserverConfig Synchronous frame-rate GNSS/LiDAR and motion observer.
% Source alignment is offline bracket interpolation with declared wait.
% Measurement channels have zero perception delay; no pose transport occurs.
    cfg=motionAidedObserverConfig();
    cfg.kind="full-synchronous-observer-v2";
    cfg.timing="synchronous";
    cfg.synchronization=struct('gnssMaximumBracket',.15,'motionMaximumBracket',.025);
    cfg.gnss=struct('positionGain',1,'headingGain',.5, ...
        'gainInformationScale',16,'minimumPositionWeight',.25, ...
        'maximumAge',.2,'courseWindow',2,'maximumCourseGap',.25, ...
        'minimumCourseDisplacement',2,'minimumSpeed',1, ...
        'headingErrorLimit',pi/3);
    % Generic input already uses the observer point. Dataset adapters may
    % supply a separately calibrated receiver-minus-observer offset.
    cfg.gnss.outputPoint=struct('bodyOffset',[0;0],'bodyCovariance',zeros(2), ...
        'headingStdRad',0,'identifier',"coincident-output-points");
    cfg.lidar.maximumAge=.2;
    cfg.bias=struct('enabled',true,'window',2,'maximumGap',.25, ...
        'timeConstant',4,'maximumMagnitude',.8,'minimumSpeed',5);
    cfg.initialHeading=0;
end
