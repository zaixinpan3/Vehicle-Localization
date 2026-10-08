function tilt=prepareMississippiLidarTilt(queryRosTime,sensorFolder,options)
% prepareMississippiLidarTilt Read only raw Ouster IMU and measured wheel data.
% The first scan may precede wheel availability: identity is then explicitly
% marked invalid, until stationary alignment can begin. No future alignment
% is backfilled into earlier scans. Reference pose files are never opened.
    arguments
        queryRosTime (:,1) double
        sensorFolder (1,1) string="output/mncav_wheel_only_20260916/sensors"
        options.ImuFile (1,1) string="output/mississippi_lidar_imu/front_imu.csv"
        options.Configuration (1,1) struct=lidarImuTiltConfig()
    end
    cfg=options.Configuration;n=numel(queryRosTime);
    assert(ismember(string(cfg.mode),["imu","disabled"]),'VehicleLocalization:InvalidTiltMode', ...
        'Only sensor-derived IMU tilt or explicitly disabled tilt is allowed.');
    if string(cfg.mode)=="disabled"
        tilt=struct('rotation',repmat(eye(3),1,1,n),'time',queryRosTime,'angles',zeros(n,2), ...
            'valid',false(n,1),'aligned',false(n,1),'ageSeconds',nan(n,1),'sourceIndex',zeros(n,1), ...
            'source',"disabled",'schemaVersion',1,'referencePoseUsed',false,'causal',true,'configuration',cfg);
        return;
    end
    assert(isfile(options.ImuFile),'VehicleLocalization:MissingLidarImu', ...
        'Export raw front-LiDAR IMU first with scripts/extractOusterImuFromBag.py. Reference attitude is not a fallback.');
    [folder,name]=fileparts(options.ImuFile);
    metadata=jsondecode(fileread(fullfile(folder,name+".json")));
    assert(metadata.schemaVersion==1 && string(metadata.source)=="front-ouster-raw-imu" && ...
        ~metadata.referencePoseUsed,'VehicleLocalization:InvalidTiltSource','Require raw sensor IMU provenance.');
    root=fileparts(fileparts(mfilename('fullpath')));
    assert(strcmpi(metadata.csvSha256,fileHash(options.ImuFile)) && ...
        strcmpi(metadata.mountSha256,fileHash(fullfile(root,'config','mississippiLidarMount.json'))), ...
        'VehicleLocalization:StaleLidarImu','IMU export or fixed mount changed; regenerate the raw export.');
    s=readtable(options.ImuFile);w=readtable(fullfile(sensorFolder,'wheel_speed_report.csv'));
    wc=wheelSpeedObserverConfig();
    imu=struct('arrivalTime',s.arrivalTime,'deviceTime',s.deviceTime, ...
        'specificForce',[s.specificX,s.specificY,s.specificZ], ...
        'angularVelocity',[s.gyroX,s.gyroY,s.gyroZ]);
    wheel=struct('arrivalTime',w.bag_time_sec, ...
        'speed',mean([w.rear_left,w.rear_right].*wc.effectiveRadius(3:4),2));
    tilt=lidarCalibrationSupport.estimateLidarImuTilt(imu,wheel,queryRosTime,cfg);
    tilt.inputMetadata=metadata;
    tilt.wheelInput=struct('file',fullfile(sensorFolder,'wheel_speed_report.csv'), ...
        'sha256',fileHash(fullfile(sensorFolder,'wheel_speed_report.csv')), ...
        'effectiveRearRadiusM',wc.effectiveRadius(3:4),'availability',"bag arrival time");
end

function hash=fileHash(path)
    f=fopen(path,'rb');assert(f>=0);cleanup=onCleanup(@()fclose(f));bytes=fread(f,Inf,'*uint8');
    digest=java.security.MessageDigest.getInstance('SHA-256');digest.update(bytes);
    hash=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2).',1,[]));
end
