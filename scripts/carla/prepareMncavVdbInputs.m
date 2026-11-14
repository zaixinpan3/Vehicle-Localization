function inputs=prepareMncavVdbInputs(datasetFolder,interfaceFile)
% prepareMncavVdbInputs Causal wheel/IMU processing without a truth channel.
% Initialize tilt from stationary specific force in [2,3) seconds, then
% integrate measured body rates. Synthetic encoders have zero delivery lag.
% The stationary interval is excluded from localization evaluation.
    s=readtable(fullfile(datasetFolder,'sensors.csv'));s=s(s.time>=2,:);
    interface=jsondecode(fileread(interfaceFile));t=s.time;n=height(s);
    nominalWheel=wheelSpeedObserverConfig();
    assert(isfield(interface,'schemaVersion') && interface.schemaVersion==2 && ...
        string(interface.source)=="independent-mncav-wheel-calibration" && ...
        isequal(reshape(interface.effectiveRadiusM,1,4),nominalWheel.effectiveRadius), ...
        'VehicleLocalization:OracleWheelCalibration','Require the independent current MnCAV wheel-radius calibration, not plant states.');
    stat=s.time<3;f=mean([s.specificX(stat),s.specificY(stat),s.specificZ(stat)],1);
    roll=atan2(f(2),f(3));pitch=atan2(-f(1),hypot(f(2),f(3)));
    tilt=zeros(3,3,n);tiltRate=zeros(3,3,n);acc=zeros(n,3);yawRate=zeros(n,1);angles=zeros(n,2);
    for k=1:n
        if k>1 && t(k)>=3
            dt=t(k)-t(k-1);p=s.gyroX(k-1);q=s.gyroY(k-1);r=s.gyroZ(k-1);
            dr=p+sin(roll)*tan(pitch)*q+cos(roll)*tan(pitch)*r;
            dp=cos(roll)*q-sin(roll)*r;
            roll=roll+dt*dr;pitch=pitch+dt*dp;
        end
        cr=cos(roll);sr=sin(roll);cp=cos(pitch);
        [R,Rdot]=localizationSupport.levelingRotation(roll,pitch,[s.gyroX(k);s.gyroY(k);s.gyroZ(k)]);
        if t(k)<3,Rdot(:)=0;end % The stationary alignment holds the tilt state.
        tilt(:,:,k)=R;tiltRate(:,:,k)=Rdot;angles(k,:)=[roll,pitch];
        acc(k,:)=[s.specificX(k),s.specificY(k),s.specificZ(k)]+([0,0,-9.81]*R);
        yawRate(k)=(sr*s.gyroY(k)+cr*s.gyroZ(k))/cp;
    end
    h=struct('time',t,'steeringAngle',s.roadWheelAngle,'yawRate',s.gyroZ, ...
        'longitudinalAcceleration',acc(:,1),'lateralAcceleration',acc(:,2));
    h.wheels=struct('time',t,'angularVelocity',[s.omegaFL,s.omegaFR,s.omegaRL,s.omegaRR]);
    wc=nominalWheel;
    wc.lagCompensation=zeros(1,4);wc.rearCgDistance=0; % Synthetic IMU is at the CG.
    wheel=localizationSupport.estimateWheelLongitudinalSpeed(h,wc);
    assert(all(wheel.valid),'VehicleLocalization:MissingVdbWheel','Wheel input became unavailable.');
    h.longitudinalSpeed=wheel.longitudinalSpeed;
    lc=lateralObserverConfig();design=designLateralObserverGains(lc);
    % The filter state supplies pitch-rate dynamics for vertical point transport.
    angular=[s.gyroX,s.gyroY,s.gyroZ];
    [filtered,alpha]=localizationSupport.filterSignalWithRate(t,angular,.1);
    velocityMap=zeros(n,6);velocityMapRate=zeros(n,6);
    for k=1:n
        R=tilt(:,:,k);Rdot=tiltRate(:,:,k);
        vertical=lc.outputPoint.forwardOffsetM*filtered(k,2);
        verticalRate=lc.outputPoint.forwardOffsetM*alpha(k,2);
        velocityMap(k,:)=[R(1,1),R(1,2),R(2,1),R(2,2),R(1,3)*vertical,R(2,3)*vertical];
        velocityMapRate(k,:)=[Rdot(1,1),Rdot(1,2),Rdot(2,1),Rdot(2,2), ...
            Rdot(1,3)*vertical+R(1,3)*verticalRate,Rdot(2,3)*vertical+R(2,3)*verticalRate];
    end
    lateral=runLateralVelocityObserver(h,design,lc, ...
        struct('value',velocityMap,'rate',velocityMapRate));
    % Yaw acceleration comes from the lateral observer; roll/pitch rates use
    % the bounded-bandwidth filter dynamics, never differences of gyro samples.
    alpha(:,3)=lateral.yawRateRate;
    angular(:,3)=lateral.yawRate;
    lever=repmat([-lc.outputPoint.forwardOffsetM,0,0],n,1);
    aout=acc+cross(alpha,lever,2)+cross(angular,cross(angular,lever,2),2);
    % Match the horizontal map axes after tilt compensation of the LiDAR.
    ao=zeros(n,3);
    for k=1:n
        ao(k,:)=(tilt(:,:,k)*aout(k,:).').';
    end
    globalMotion=struct('time',t,'longitudinalSpeed',lateral.longitudinalSpeed, ...
        'longitudinalAcceleration',ao(:,1),'lateralAcceleration',ao(:,2),'yawRate',yawRate);
    inputs=struct('highRate',globalMotion,'lateral',lateral,'lateralDesign',design,'wheel',wheel, ...
        'tilt',tilt,'tiltRate',tiltRate,'angles',angles,'cgMotion',h,'sensorTable',s,'referenceUsed',false, ...
        'initialization',"Stationary IMU alignment over [2,3) seconds; pose initialization follows in the localization runner");
    inputs.signature=mncavVdbInputSignature(datasetFolder,interfaceFile);
end
