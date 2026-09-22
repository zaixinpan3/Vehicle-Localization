function report=run_experiment(outputFolder,window)
% run_experiment Replay the stable coarse source with unchanged map and motion.
    arguments
        outputFolder (1,1) string="output/temporal_perception_20260922/matching"
        window (1,1) struct=localizationSourceWindowConfig()
    end
    root=setupVehicleLocalization();
    folder=fullfile(root,outputFolder);
    [prepared,~,~]=prepareMncavObserverReplay('output/mncav_wheel_only_20260916/sensors', ...
        'output/mncav_interface_audit_20260916/vehicle_parameters.json',table(),0,IncludeOdom=false);
    saved=load('output/mncav_inspva_observer_20260915/experiment.mat','lateralDesign');
    lateral=runLateralVelocityObserver(prepared.highRate,saved.lateralDesign,saved.lateralDesign.cfg);
    h=prepared.highRate;
    motion=struct('time',h.time,'longitudinalSpeed',h.longitudinalSpeed, ...
        'lateralVelocity',lateral.lateralVelocity,'yawRate',h.yawRate, ...
        'longitudinalVelocitySource',"four_wheel",'clockModelId',prepared.clockModelId);
    report=replayMississippiLocalization('output/mississippi_mapping_calibrated/probability_cloud.mat', ...
        'output/mncav_wheel_only_20260916/sensors',folder,"recursive",[],MotionInputs=motion,SourceWindowConfig=window);
end
