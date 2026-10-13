function [report,estimate,trial]=runKittiOverlapTrial(datasetFolder,outputFolder)
% runKittiOverlapTrial Known-map transfer trial on a KITTI Raw sync drive.
% Odd zero-based scans build a fixed map using OXTS reference poses; even
% scans are queries. Runtime accepts LiDAR, OXTS body inertial outputs and one
% initial pose/tilt only. No navigation velocity, later pose, GNSS correction,
% steering or wheel-speed measurement enters the observer. This tests the
% global branch, not the vehicle-specific lateral observer or KITTI odometry.
% Numerical outputs stay in memory; only small summaries and figures persist.
% Optional estimate/trial outputs support paired in-memory validation; the
% trial reference is scoring data and never crosses the estimator boundary.
    setupVehicleLocalization();if ~isfolder(outputFolder),mkdir(outputFolder);end
    packetFiles=dir(fullfile(datasetFolder,'oxts','data','*.txt'));
    assert(numel(packetFiles)>=4,'VehicleLocalization:MissingKittiData','Require KITTI Raw OXTS and LiDAR files.');
    packets=zeros(numel(packetFiles),30);ids=zeros(numel(packetFiles),1);
    for k=1:numel(packetFiles)
        ids(k)=str2double(erase(packetFiles(k).name,'.txt'));
        value=readmatrix(fullfile(packetFiles(k).folder,packetFiles(k).name));
        assert(numel(value)==30,'VehicleLocalization:InvalidKittiPacket','Require the 30-field OXTS format.');packets(k,:)=value(:).';
    end
    [ids,order]=sort(ids);packets=packets(order,:);
    stamps=string(readlines(fullfile(datasetFolder,'oxts','timestamps.txt')));stamps(stamps=="")=[];
    times=datetime(stamps,'InputFormat','yyyy-MM-dd HH:mm:ss.SSSSSSSSS');t=seconds(times-times(1));t=t(ids+1);
    lidarStamps=string(readlines(fullfile(datasetFolder,'velodyne_points','timestamps.txt')));lidarStamps(lidarStamps=="")=[];
    lidarTimes=datetime(lidarStamps,'InputFormat','yyyy-MM-dd HH:mm:ss.SSSSSSSSS');
    packetOffset=seconds(times(ids+1)-lidarTimes(ids+1));
    calibration=readCalibration(fullfile(fileparts(datasetFolder),'calib_imu_to_velo.txt'));
    imuFromVelo=inv(calibration);reference=referencePoses(packets);
    training=find(mod(ids,2)==1);query=find(mod(ids,2)==0);names=["curb","pole","facade"];
    p=perceptionConfig("Downtown","offline");p.featureNames=names;p.frameCalibration=lidarFrameCalibrationConfig();
    mapCfg=featureMapBuildConfig();mapCfg.frameCalibration=p.frameCalibration;mapCfg.featureNames=names;
    mapCfg.temporalMap.frameCalibration=p.frameCalibration;mapCfg.logEnabled=false;
    observations=struct('featureNames',names,'frameIndices',ids(training).', ...
        'frameCalibration',p.frameCalibration,'pointsByFeatureFrame',{cell(numel(names),numel(training))});
    timer=tic;counts=zeros(numel(training),numel(names));
    for j=1:numel(training)
        k=training(j);xyz=readScan(datasetFolder,ids(k),imuFromVelo);
        % Mapping may use all reference attitude; query attitude is reconstructed
        % separately from gyros after a single initial tilt below.
        Rtilt=tiltRotation(packets(k,4),packets(k,5));level=xyz*Rtilt.';
        perception=perceiveFrame(asFrame(level),p);R=rotation(reference(k,3));
        globalXYZ=[level(:,1:2)*R.'+reference(k,1:2),level(:,3)+reference(k,4)];
        for c=1:numel(names)
            points=globalXYZ(logical(perception.featureMasks.(names(c))),:);counts(j,c)=size(points,1);
            if ~isempty(points)
                resolution=mapCfg.temporalMap.defaultParams.representativeResolution;
                [~,keep]=unique(floor(points(:,1:2)/resolution),'rows','stable');points=points(keep,:);
            end
            observations.pointsByFeatureFrame{c,j}=points;
        end
        if mod(j,20)==0 || j==numel(training),fprintf('KITTI mapping %d/%d\n',j,numel(training));end
    end
    map=registrationSupport.projectSemanticProbabilityCloud(temporalMapToProbabilityCloud(buildSlidingWindowMap(observations,mapCfg)),2);
    mappingSeconds=toc(timer);clear observations;
    % Only these named inertial columns cross into runtime preparation.
    sensors=struct('time',t(query)-t(query(1)),'bodyAcceleration',packets(query,12:14), ...
        'bodyAngularRate',packets(query,18:20));
    initial=struct('pose',reference(query(1),1:3)+[.5,-.4,deg2rad(2)], ...
        'tilt',packets(query(1),4:5),'velocity',[0,0]);
    [h,l,tilt]=motionInputs(sensors,initial.tilt);
    p=perceptionConfig("Downtown");p.featureNames=names;p.frameCalibration=lidarFrameCalibrationConfig();
    sources=cell(numel(query),1);timer=tic;
    for j=1:numel(query)
        xyz=readScan(datasetFolder,ids(query(j)),imuFromVelo);
        sources{j}=perceiveCoarseProbabilityCloud(asFrame(xyz*tiltRotation(tilt(j,1),tilt(j,2)).'),p);
        if mod(j,20)==0 || j==numel(query),fprintf('KITTI query perception %d/%d\n',j,numel(query));end
    end
    perceptionSeconds=toc(timer);
    % Re-synthesize the kinematic global observer for explicitly declared
    % transfer-trial assumptions. Design revisions used this development drive;
    % no navigation reference series enters prediction or innovations.
    cfg=mncavFullObserverConfig(LoadDesign=false);cfg.kind="kitti-global-overlap-transfer-trial";
    cfg.invariantObserver.velocityAvailable=false;
    cfg.invariantObserver.operatingPoints=["lidar","weakLidar"];
    cfg.gnss.outputPoint=struct('bodyOffset',[0;0],'bodyCovariance',zeros(2), ...
        'headingStdRad',0,'identifier',"coincident-output-points");
    cfg.invariantObserver.noise=struct('lidarPositionM',.1,'lidarHeadingRad',.01, ...
        'gnssPositionM',1,'velocityMps',1,'accelerationMps2',1,'gyroRadps',.01, ...
        'jerkMps3',cfg.iss.maximumAcceleration/2);
    cfg.invariantObserver.provenance.noise="Declared transfer scales; jerk maximumAcceleration / 2 s, no velocity output, weak LiDAR without GNSS. Revised during this development trial; no independent holdout claim";
    cfg.invariantObserver.provenance.theta="Fixed theta 4 inherited from the current route; no KITTI sweep";
    design=invariantObserverSupport.synthesize(cfg);cfg.invariantDesign=design;
    cfg.gainSource="Fresh LMI synthesis for declared KITTI transfer trial";
    cfg.initialState=[initial.pose(1);0;0;initial.pose(2);0;0;initial.pose(3)];
    data=struct('highRate',h,'lidarOverlap',struct('map',map,'sources',{sources}, ...
        'valid',true(numel(query),1),'config',overlapGradientConfig()));
    timer=tic;estimate=runFullLocalizationObserver(data,struct(),cfg,LateralInputs=l);observerSeconds=toc(timer);
    trial=struct('data',data,'lateral',l,'config',cfg,'reference',reference(query,1:3));
    poisoned=data;poisoned.highRate.longitudinalSpeed(:)=1e8;poisonedLateral=l;poisonedLateral.lateralVelocity(:)=-1e8;
    isolated=runFullLocalizationObserver(poisoned,struct(),cfg,LateralInputs=poisonedLateral);
    velocityIsolationDifference=max(abs(isolated.z-estimate.z),[],'all');
    assert(velocityIsolationDifference==0,'VehicleLocalization:KittiVelocityLeakage','Withdrawn velocity payload changed a state.');
    errors=estimate.pose-reference(query,1:3);errors(:,3)=atan2(sin(errors(:,3)),cos(errors(:,3)));
    distance=hypot(errors(:,1),errors(:,2));tracking=sensors.time>=2;
    metrics=table(["all";"after_2s"],[numel(query);nnz(tracking)], ...
        [sqrt(mean(distance.^2));sqrt(mean(distance(tracking).^2))], ...
        [median(distance);median(distance(tracking))], ...
        [prctile(distance,95);prctile(distance(tracking),95)], ...
        [max(distance);max(distance(tracking))], ...
        rad2deg([sqrt(mean(errors(:,3).^2));sqrt(mean(errors(tracking,3).^2))]), ...
        'VariableNames',{'window','frames','position_rmse_m','position_median_m','position_p95_m','position_max_m','heading_rmse_deg'});
    writetable(metrics,fullfile(outputFolder,'metrics.csv'));
    report=struct('datasetFolder',string(datasetFolder),'trainingFrames',ids(training).', ...
        'queryFrames',ids(query).','trainingQueryOverlap',numel(intersect(ids(training),ids(query))), ...
        'mappingSeconds',mappingSeconds,'perceptionSeconds',perceptionSeconds,'observerSeconds',observerSeconds, ...
        'mapComponents',map.components.numComponents,'mappingFeatureCounts',sum(counts,1), ...
        'featureNames',names,'reference',"KITTI OXTS GPS/INS at the IMU origin; not independent absolute truth", ...
        'runtimeInputs',"LiDAR and OXTS ax/ay/az, wx/wy/wz; one initial pose and tilt; initial velocity zero", ...
        'navigationVelocityUsed',false,'referenceUsedAfterInitialization',false,'wheelSpeedUsed',false,'steeringUsed',false, ...
        'bodyVelocityValid',estimate.diagnostics.packetCounts.bodyVelocityValid, ...
        'withdrawnVelocityStateDifference',velocityIsolationDifference, ...
        'lidarAvailable',nnz(estimate.diagnostics.lidarOverlap.available),'singleFrameSources',true, ...
        'deskew',"Disabled for map and queries; motion distortion remains", ...
        'clock',"Synchronized packet/frame indices on the native OXTS timestamps; queries at approximately 5 Hz", ...
        'packetMinusLidarTimestampRangeS',[min(packetOffset),max(packetOffset)], ...
        'perceptionProfile',"Unmodified Downtown geometry thresholds with curb, pole and facade; no intensity or KITTI labels", ...
        'lateralObserverUsed',false,'courseRateApproximation',"Internal estimated rate with gyro-output innovation; sideslip-angle derivative assumed zero", ...
        'intrinsicPredictionUsesImu',false,'developmentTrial',true, ...
        'initialization',initial,'gainInputs',invariantObserverSupport.inputs(cfg), ...
        'gainDesign',cfg.invariantDesign,'gainVerification',design.verification,'metrics',table2struct(metrics), ...
        'actualOverlapOperatorLmiVerified',false,'sampledSystemCertified',false,'officialOdometryBenchmark',false);
    f=fopen(fullfile(outputFolder,'summary.json'),'w');fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));fclose(f);
    fig=figure('Visible','off','Color','white','Position',[100,100,900,700]);tiledlayout(2,1,TileSpacing="compact");nexttile;
    plot(reference(query,1),reference(query,2),'k-',estimate.position(:,1),estimate.position(:,2),'b-');axis equal;axis tight;grid on;
    legend('OXTS reference','Overlap observer');xlabel('East (m)');ylabel('North (m)');
    nexttile;plot(sensors.time,distance);grid on;xlabel('Elapsed time (s)');ylabel('Position difference (m)');
    set(findall(fig,'Type','axes'),'Color','white','XColor','black','YColor','black','GridColor',[.7,.7,.7]);
    set(findall(fig,'Type','text'),'Color','black');set(findall(fig,'Type','legend'),'Color','white','TextColor','black');
    exportgraphics(fig,fullfile(outputFolder,'trajectory.png'),BackgroundColor="white");close(fig);disp(metrics);
end

function T=readCalibration(path)
    lines=string(readlines(path));r=sscanf(extractAfter(lines(startsWith(lines,'R:')),'R:'),'%f');
    t=sscanf(extractAfter(lines(startsWith(lines,'T:')),'T:'),'%f');T=[reshape(r,3,3).',t;0,0,0,1];
end

function ref=referencePoses(p)
    scale=cosd(p(1,1));radius=6378137;
    xy=[scale*radius*deg2rad(p(:,2)),scale*radius*log(tan(deg2rad(90+p(:,1))/2))];
    ref=[xy-xy(1,:),p(:,6),p(:,3)-p(1,3)];
end

function xyz=readScan(folder,id,T)
    path=fullfile(folder,'velodyne_points','data',sprintf('%010d.bin',id));f=fopen(path,'rb');
    assert(f>=0,'VehicleLocalization:MissingKittiScan','Missing scan %s.',path);cleanup=onCleanup(@()fclose(f));
    raw=fread(f,[4,Inf],'single=>double').';xyz=raw(:,1:3)*T(1:3,1:3).'+T(1:3,4).';
end

function frame=asFrame(xyz)
    frame=struct('x',xyz(:,1),'y',xyz(:,2),'z',xyz(:,3));
end

function R=rotation(yaw)
    R=[cos(yaw),-sin(yaw);sin(yaw),cos(yaw)];
end

function R=tiltRotation(roll,pitch)
    Rx=[1,0,0;0,cos(roll),-sin(roll);0,sin(roll),cos(roll)];
    Ry=[cos(pitch),0,sin(pitch);0,1,0;-sin(pitch),0,cos(pitch)];R=Ry*Rx;
end

function [h,l,tilt]=motionInputs(sensors,initialTilt)
% This boundary contains no navigation pose, orientation or velocity series.
    t=sensors.time;n=numel(t);tilt=zeros(n,2);tilt(1,:)=initialTilt;rate=zeros(n,1);acc=zeros(n,2);
    for k=1:n
        if k>1
            w=(sensors.bodyAngularRate(k-1,:)+sensors.bodyAngularRate(k,:))/2;phi=tilt(k-1,1);theta=tilt(k-1,2);
            derivative=[w(1)+tan(theta)*(sin(phi)*w(2)+cos(phi)*w(3)),cos(phi)*w(2)-sin(phi)*w(3)];
            tilt(k,:)=tilt(k-1,:)+(t(k)-t(k-1))*derivative;
        end
        phi=tilt(k,1);theta=tilt(k,2);w=sensors.bodyAngularRate(k,:);
        rate(k)=(sin(phi)*w(2)+cos(phi)*w(3))/cos(theta);
        a=tiltRotation(phi,theta)*sensors.bodyAcceleration(k,:).';acc(k,:)=a(1:2).';
    end
    zero=zeros(n,1);h=struct('time',t,'longitudinalSpeed',nan(n,1),'bodyVelocityValid',false(n,1), ...
        'longitudinalAcceleration',acc(:,1),'lateralAcceleration',acc(:,2),'yawRate',rate);
    l=struct('time',t,'lateralVelocity',nan(n,1),'sideSlipAngleRate',zero);
end
