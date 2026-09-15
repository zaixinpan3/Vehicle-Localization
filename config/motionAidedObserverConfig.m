function cfg=motionAidedObserverConfig()
% motionAidedObserverConfig Zero-delay seven-state MnCAV motion-aided design.
% Physical gains [position, velocity, acceleration, heading] have units 1/s.
% Selected on t<=60 s of the recorded design drive; see the validation report.
% Acceleration inputs must be compensated inertial body-frame acceleration.
    cfg=struct('kind',"motion-aided-seven-state-v1", ...
        'gains',[4,4,12,4],'maximumTrackAngleRate',.4, ...
        'maximumIntegrationStep',.005,'initialState',[]);
    cfg.lidar=struct('poseScales',[1;1;1],'gainInformationScale',.001, ...
        'minimumPoseWeight',.25);
end
