function report=validate_pipeline()
% validate_pipeline Audit synchronized products and reject stale inputs.
    setupVehicleLocalization();
    folder='output/receiver_clock_20260921';
    clock=loadReceiverClock('data/raw/Missisipi/gnss/raw_data_2024-06-07-12-09-31_0_inspva.csv');
    cfg=featureMapBuildConfig();poses=readFramePoseTable(fullfile('data',cfg.poseMatchCsvPath),1:1170);
    python=readtable('research/receiver_clock_20260921/frame_times.csv');
    matlab=receiverClockTime(clock,poses.lidar_stamp_sec);
    mismatch=max(abs(matlab-python.newReceiverSeconds));assert(mismatch<1e-10);
    map=load('output/mississippi_mapping_synchronized/probability_cloud.mat','cloud');
    assert(string(map.cloud.clockModelId)==string(clock.modelId));
    matching=load(fullfile(folder,'coarse_pipeline/matching/report.mat'),'report');
    assert(string(matching.report.metadata.clockModelId)==string(clock.modelId));
    assert(all(string(matching.report.calls.clockModelId)==string(clock.modelId)));
    reference=jsondecode(fileread('output/receiver_synchronized_inputs/reference_metadata.json'));
    gnss=jsondecode(fileread('output/receiver_synchronized_inputs/bestpos_metadata.json'));
    assert(string(reference.clock.modelId)==string(clock.modelId) && string(gnss.clock.modelId)==string(clock.modelId));
    sensor='output/mncav_wheel_only_20260916/sensors';
    parameter='output/mncav_interface_audit_20260916/vehicle_parameters.json';
    [data,~,~]=prepareMncavObserverReplay(sensor,parameter,table(),0,IncludeOdom=false);
    prior=load('output/mncav_inspva_observer_20260915/experiment.mat','lateralDesign');
    lateral=runLateralVelocityObserver(data.highRate,prior.lateralDesign,prior.lateralDesign.cfg);
    h=data.highRate;motion=struct('time',h.time,'longitudinalSpeed',h.longitudinalSpeed, ...
        'lateralVelocity',lateral.lateralVelocity,'yawRate',h.yawRate, ...
        'longitudinalVelocitySource',"four_wheel",'clockModelId',data.clockModelId);
    staleMapRejected=expectClockMismatch(@()replayMississippiLocalization( ...
        'output/mississippi_mapping_inspva_20260915/probability_cloud.mat',sensor, ...
        fullfile(folder,'rejected_old_map'),"referenceSeed",[1,307],MotionInputs=motion));
    motion.clockModelId="stale";
    staleMotionRejected=expectClockMismatch(@()replayMississippiLocalization( ...
        'output/mississippi_mapping_synchronized/probability_cloud.mat',sensor, ...
        fullfile(folder,'rejected_old_motion'),"referenceSeed",[1,307],MotionInputs=motion));
    undeclaredCallsRejected=expectClockMismatch(@()prepareMncavObserverReplay(sensor,parameter, ...
        table(1,VariableNames={'accepted'}),0,IncludeOdom=false));
    report=struct('clockModelId',clock.modelId,'frames',height(poses), ...
        'maximumPythonMatlabTimestampDifferenceSeconds',mismatch, ...
        'staleMapRejected',staleMapRejected,'staleMotionRejected',staleMotionRejected, ...
        'undeclaredLidarCallsRejected',undeclaredCallsRejected, ...
        'coarsePipelinePassed',true,'finePerceptionUsed',matching.report.metadata.finePerceptionUsed, ...
        'matching',matching.report.summary);
    fid=fopen('research/receiver_clock_20260921/pipeline_validation.json','w');assert(fid>=0);
    cleanup=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));disp(report);
end

function passed=expectClockMismatch(operation)
    passed=false;
    try
        operation();
    catch exception
        assert(string(exception.identifier)=="VehicleLocalization:ClockMismatch",exception.message);
        passed=true;
    end
    assert(passed,'An incompatible clock must be rejected.');
end
