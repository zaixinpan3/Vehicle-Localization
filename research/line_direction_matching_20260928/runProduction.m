function report=runProduction()
% runProduction Fresh full-route replay with deployed line-direction factors.
    root=setupVehicleLocalization();external=fullfile(fileparts(root),'RobustVehicleLocalization','external');
    addpath(genpath(fullfile(external,'YALMIP')));addpath(genpath(fullfile(external,'sedumi')));
    out='output/line_direction_matching_20260928/production';
    assert(~isfile(fullfile(out,'report.mat')),'Refuse to overwrite a completed production replay.');
    set(groot,'defaultFigureVisible','off');mapcfg=featureMapBuildConfig();sensors='output/mncav_wheel_only_20260916/sensors';
    [prepared,~,~]=prepareMncavObserverReplay(sensors,"",table(),0,IncludeOdom=false);
    design=designLateralObserverGains(lateralObserverConfig());
    lateral=runLateralVelocityObserver(prepared.highRate,design,lateralObserverConfig());h=prepared.highRate;
    motion=struct('time',h.time,'longitudinalSpeed',h.longitudinalSpeed,'lateralVelocity',lateral.lateralVelocity, ...
        'yawRate',h.yawRate,'longitudinalVelocitySource',"four_wheel",'clockModelId',prepared.clockModelId);
    report=replayMississippiLocalization(mapcfg.probabilityCloudPath,sensors,out,"recursive",[],MotionInputs=motion);
    fprintf('PRODUCTION_REPLAY_COMPLETED\n');
end
