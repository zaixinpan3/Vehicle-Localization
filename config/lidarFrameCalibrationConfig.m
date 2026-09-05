function cfg = lidarFrameCalibrationConfig()
% lidarFrameCalibrationConfig: Increment from stored point coordinates to body.
% Apply p_body = rotation*p_stored + translation before the recorded attitude.
% Identity is the default. Dataset estimates must be selected explicitly and
% used for BOTH offline map construction and the online probability cloud.
    cfg=struct('rotation',eye(3),'translation',[0 0 0],'identifier',"identity");
end
