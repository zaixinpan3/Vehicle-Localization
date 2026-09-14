function estimate = runImprovedVehicleObserver(sensorData,lateralDesign,observerDesign,cfg,options)
% runImprovedVehicleObserver Integrate the continuous GNSS or delayed LiDAR HGO.
% sensorData.highRate contains the numerical input grid. The selected source
% is sensorData.gnss or sensorData.lidar, with evaluate(t) returning a 2-vector
% position or a struct with pose (3-vector, lifted yaw) and information (3x3).
% A source can instead explicitly declare representation="piecewiseLinear"
% and supply time plus position/pose/information on the highRate.time grid.
% Such arrays define an offline continuous signal reconstruction.
% LiDAR evaluates y_L(t)=C*z(t-delay)+noise at CURRENT time t; do not shift
% it again in the runner. source.delay must equal cfg.measurement.fixedLidarDelay.
% InitialHistory(t) supplies a continuous seven-state estimate on [t0-delay,t0]
% matching cfg.observer.initialState at t0. It is required for positive delay.
% LateralInputs can bypass the independent lateral observer on the input grid.
% RK4 with method of steps and cubic Hermite history approximates the continuous
% equations; it does not introduce measurement events or state corrections.
    arguments
        sensorData (1,1) struct
        lateralDesign (1,1) struct
        observerDesign (1,1) struct
        cfg (1,1) struct = improvedObserverConfig()
        options.LateralInputs (1,1) struct = struct()
        options.InitialHistory = []
    end
    verification=verifyImprovedObserverDesign(observerDesign,cfg);
    assert(verification.certified,'VehicleLocalization:CertificateMismatch', ...
        'The supplied matrices fail the continuous ISS certificate.');
    validateRuntimeConfig(cfg);
    high=normalizeHighRate(sensorData);
    if isempty(fieldnames(options.LateralInputs))
        lateral=runLateralVelocityObserver(high,lateralDesign,lateralDesign.cfg);
        lateralSource="lateralObserver";
    else
        lateral=validateLateralInputs(options.LateralInputs,high.time);
        lateralSource="provided";
    end
    source=normalizeSource(sensorData,cfg,high.time);
    delay=0;if cfg.mode=="lidar",delay=cfg.measurement.fixedLidarDelay;end
    initialTime=high.time(1);
    state=initialState(cfg,source,high,lateral);
    if delay>0
        assert(isa(options.InitialHistory,'function_handle'), ...
            'VehicleLocalization:InitialHistoryRequired','Positive LiDAR delay requires InitialHistory(t).');
        endpoint=historyValue(options.InitialHistory,initialTime);
        assert(norm(endpoint-state)<=1e-10*max(1,norm(state)), ...
            'VehicleLocalization:InitialHistoryMismatch','InitialHistory(t0) must match the initial state.');
        historyValue(options.InitialHistory,initialTime-delay);
    end
    data=buildImprovedObserverCertificateData(cfg);
    observerDesign.poseGain=data.T*observerDesign.K;
    observerDesign.auxiliaryGain=data.T*observerDesign.N/cfg.observer.theta^3;
    observerDesign.C=data.C;
    history=struct('time',zeros(0,1),'state',zeros(7,0),'derivative',zeros(7,0));
    n=numel(high.time);states=zeros(n,7);innovations=zeros(n,3);auxiliary=zeros(n,4);
    weights=zeros(3,3,n);delayedStates=NaN(n,7);rateOutside=false(n,1);
    maximumHistory=0;steps=0;anyRateOutside=false;
    for k=1:n
        time=high.time(k);
        if k>1
            left=high.time(k-1);span=time-left;
            maximumStep=cfg.measurement.maximumIntegrationStep;
            if delay>0,maximumStep=min(maximumStep,delay);end
            pieces=max(1,ceil(span/maximumStep-1e-10));
            cuts=linspace(left,time,pieces+1);
            if delay>0
                firstBreak=floor((left-initialTime)/delay)+1;
                lastBreak=floor((time-initialTime)/delay);
                breaks=initialTime+(firstBreak:lastBreak)*delay;
                cuts=sort([cuts,breaks(breaks>left+1e-12 & breaks<time-1e-12)]);
                cuts=cuts([true,diff(cuts)>1e-12]);
            end
            for piece=1:numel(cuts)-1
                a=cuts(piece);b=cuts(piece+1);step=b-a;
                sample1=inputAt(high,lateral,k-1,(a-left)/span);
                sample2=inputAt(high,lateral,k-1,((a+b)/2-left)/span);
                sample3=inputAt(high,lateral,k-1,(b-left)/span);
                anyRateOutside=anyRateOutside || max(abs([sample1.yawRate+sample1.sideSlipAngleRate, ...
                    sample2.yawRate+sample2.sideSlipAngleRate,sample3.yawRate+sample3.sideSlipAngleRate])) ...
                    >cfg.operating.maximumTrackAngleRate+1e-12;
                past1=delayedValue(a,delay,state,history,options.InitialHistory,initialTime);
                [k1,~]=derivative(a,state,past1,sample1,source,observerDesign,cfg);
                middle=state+step*k1/2;
                past2=delayedValue((a+b)/2,delay,middle,history,options.InitialHistory,initialTime);
                k2=derivative((a+b)/2,middle,past2,sample2,source,observerDesign,cfg);
                middle=state+step*k2/2;
                past3=delayedValue((a+b)/2,delay,middle,history,options.InitialHistory,initialTime);
                k3=derivative((a+b)/2,middle,past3,sample2,source,observerDesign,cfg);
                endpoint=state+step*k3;
                past4=delayedValue(b,delay,endpoint,history,options.InitialHistory,initialTime);
                k4=derivative(b,endpoint,past4,sample3,source,observerDesign,cfg);
                next=state+step*(k1+2*k2+2*k3+k4)/6;
                assert(all(isfinite(next)),'VehicleLocalization:NonfiniteObserver', ...
                    'Continuous integration produced a nonfinite state at %.9g.',b);
                if delay>0
                    kend=derivative(b,next,past4,sample3,source,observerDesign,cfg);
                    if isempty(history.time)
                        history.time=a;history.state=state;history.derivative=k1;
                    end
                    history.time(end+1,1)=b;history.state(:,end+1)=next;
                    history.derivative(:,end+1)=kend;
                    % Retain one bracket before the next delayed query.
                    first=find(history.time<=b-delay,1,'last');
                    if ~isempty(first) && first>1
                        history.time=history.time(first:end);
                        history.state=history.state(:,first:end);
                        history.derivative=history.derivative(:,first:end);
                    end
                    maximumHistory=max(maximumHistory,numel(history.time));
                end
                state=next;steps=steps+1;
            end
        end
        sample=inputAt(high,lateral,min(k,n-1),double(k==n));
        past=delayedValue(time,delay,state,history,options.InitialHistory,initialTime);
        [~,details]=derivative(time,state,past,sample,source,observerDesign,cfg);
        states(k,:)=state.';innovations(k,1:numel(details.pose))=details.pose.';
        auxiliary(k,:)=details.auxiliary.';weights(:,:,k)=details.weight;
        delayedStates(k,:)=past.';
        rateOutside(k)=abs(sample.yawRate+sample.sideSlipAngleRate)>cfg.operating.maximumTrackAngleRate+1e-12;
    end
    wrapped=atan2(sin(states(:,7)),cos(states(:,7)));
    speed=hypot(high.longitudinalSpeed,lateral.lateralVelocity);
    conditional=~(anyRateOutside || any(rateOutside));
    if cfg.mode=="gnss",conditional=conditional && all(speed>=cfg.gnss.minimumSpeed);end
    conditions=struct('coefficientBoundsSatisfied',conditional, ...
        'matrixCertificateVerified',verification.certified, ...
        'conditionalCertificateApplicable',conditional, ...
        'trueStateBoundsVerified',false,'headingChartVerified',false, ...
        'upstreamDisturbanceBoundsVerified',false,'numericalErrorBoundVerified',false, ...
        'allTheoremHypothesesVerified',false, ...
        'informationAuditScope',"Function outputs checked at evaluation times; continuous bounds remain a signal hypothesis.", ...
        'gnssHeadingAdmissionVerified',false,'unconditionalStabilityClaimed',false);
    estimate=struct('time',high.time,'z',states,'onlineZ',states, ...
        'pose',[states(:,[1,4]),wrapped],'position',states(:,[1,4]), ...
        'velocity',states(:,[2,5]),'acceleration',states(:,[3,6]), ...
        'heading',wrapped,'headingUnwrapped',states(:,7), ...
        'speed',hypot(states(:,2),states(:,5)),'lateral',lateral, ...
        'sideSlipAngle',lateral.sideSlipAngle,'sideSlipAngleRate',lateral.sideSlipAngleRate, ...
        'trackAngleRate',high.yawRate+lateral.sideSlipAngleRate, ...
        'innovations',struct('pose',innovations,'invariant',auxiliary), ...
        'measurements',struct('highRate',high,'source',source));
    estimate.observer=struct('kind',"continuous-mo-hgo-v1",'mode',cfg.mode, ...
        'theta',cfg.observer.theta,'certificateVerified',verification.certified, ...
        'certified',false,'verification',verification, ...
        'scope',"Continuous ODE/DDE; the numerical trajectory is not an unconditional ISS certificate.");
    estimate.diagnostics=struct('integrationStepCount',steps, ...
        'maximumHistoryNodes',maximumHistory,'historyDuration',delay, ...
        'stateHistoryRecomputed',false,'lateralInputSource',lateralSource, ...
        'measurementRepresentation',source.representation,'poseWeight',weights, ...
        'delayedState',delayedStates,'outsideTrackRateEnvelope',rateOutside, ...
        'anyStageOutsideTrackRateEnvelope',anyRateOutside, ...
        'invariantExtensionActive',any(abs(states(:,[2,5]))>cfg.operating.maximumSpeed,2) ...
            |any(abs(states(:,[3,6]))>cfg.operating.maximumAcceleration,2), ...
        'certificateConditions',conditions);
end

function [value,details]=derivative(time,state,past,sample,source,design,cfg)
    channels=evaluateImprovedObserverChannels(state,sample,cfg.operating);
    auxiliary=channels.invariantInnovation;
    measurement=sourceValue(source,time,cfg);
    if cfg.mode=="gnss"
        pose=measurement.position-state([1,4]);weight=zeros(3);
        % Theorem G uses raw estimated velocity for its triangular yaw row.
        angle=state(7)+sample.sideSlipAngle;
        auxiliary(4)=-(state(5)*cos(angle)-state(2)*sin(angle));
        correction=design.poseGain*pose;
    else
        pose=measurement.pose./cfg.lidar.poseScales(:)-design.C*past;
        weight=measurement.weight;correction=design.poseGain*weight*pose;
    end
    value=channels.modelDerivative+correction+design.auxiliaryGain*auxiliary;
    details=struct('pose',pose,'auxiliary',auxiliary,'weight',weight);
end

function state=delayedValue(time,delay,current,history,initial,initialTime)
    if delay==0,state=current;return;end
    query=time-delay;
    if query<=initialTime+1e-12
        state=historyValue(initial,min(query,initialTime));return;
    end
    assert(~isempty(history.time) && query>=history.time(1)-1e-10 && query<=history.time(end)+1e-10, ...
        'VehicleLocalization:HistoryUnavailable','The method of steps needs an existing history bracket.');
    if query>=history.time(end)-1e-12,state=history.state(:,end);return;end
    index=find(history.time<=query,1,'last');
    if isempty(index),index=1;end
    step=history.time(index+1)-history.time(index);u=(query-history.time(index))/step;
    state=(2*u^3-3*u^2+1)*history.state(:,index)+(u^3-2*u^2+u)*step*history.derivative(:,index) ...
        +(-2*u^3+3*u^2)*history.state(:,index+1)+(u^3-u^2)*step*history.derivative(:,index+1);
end

function state=historyValue(provider,time)
    state=provider(time);
    assert(isnumeric(state) && isreal(state) && numel(state)==7 && all(isfinite(state)), ...
        'VehicleLocalization:InvalidInitialHistory','InitialHistory must return seven finite real states.');
    state=double(state(:));
end

function high=normalizeHighRate(data)
    assert(isfield(data,'highRate') && isstruct(data.highRate) && isfield(data.highRate,'time'), ...
        'VehicleLocalization:InvalidInputs','A highRate input grid is required.');
    high=data.highRate;n=numel(high.time);
    assert(n>=2,'VehicleLocalization:InvalidInputs','At least two input times are required.');
    for name=["time","steeringAngle","longitudinalSpeed","longitudinalAcceleration","lateralAcceleration","yawRate"]
        assert(isfield(high,name) && isnumeric(high.(name)) && isreal(high.(name)) ...
            && isvector(high.(name)) && numel(high.(name))==n && all(isfinite(high.(name))), ...
            'VehicleLocalization:InvalidInputs','highRate.%s must be a finite real aligned vector.',name);
        high.(name)=double(high.(name)(:));
    end
    assert(all(diff(high.time)>0) && all(high.longitudinalSpeed>=0), ...
        'VehicleLocalization:InvalidInputs','Times must increase and longitudinal speed must be nonnegative.');
end

function lateral=validateLateralInputs(lateral,time)
    for name=["time","lateralVelocity","sideSlipAngle","sideSlipAngleRate"]
        assert(isfield(lateral,name) && isnumeric(lateral.(name)) && isreal(lateral.(name)) ...
            && isvector(lateral.(name)) && numel(lateral.(name))==numel(time) ...
            && all(isfinite(lateral.(name))), ...
            'VehicleLocalization:InvalidLateralInputs','LateralInputs.%s must be finite and aligned.',name);
        lateral.(name)=double(lateral.(name)(:));
    end
    assert(isequal(lateral.time,time),'VehicleLocalization:InvalidLateralInputs', ...
        'LateralInputs.time must equal the high-rate timestamps.');
end

function sample=inputAt(high,lateral,index,alpha)
    sample=struct();
    for name=["longitudinalSpeed","longitudinalAcceleration","lateralAcceleration","yawRate"]
        sample.(name)=(1-alpha)*high.(name)(index)+alpha*high.(name)(index+1);
    end
    for name=["lateralVelocity","sideSlipAngle","sideSlipAngleRate"]
        sample.(name)=(1-alpha)*lateral.(name)(index)+alpha*lateral.(name)(index+1);
    end
end

function source=normalizeSource(data,cfg,time)
    assert(isfield(data,cfg.mode) && isstruct(data.(cfg.mode)), ...
        'VehicleLocalization:ContinuousMeasurementRequired','Provide sensorData.%s as a continuous signal.',cfg.mode);
    source=data.(cfg.mode);
    assert(~any(isfield(source,{'timestamp','arrivalTime','arrivalTimeIsPlaceholder'})), ...
        'VehicleLocalization:EventInputUnsupported','Use a continuous output on the evaluation clock, without arrival metadata.');
    if cfg.mode=="gnss"
        other="lidar";
    else
        other="gnss";
        assert(isfield(source,'delay') && isscalar(source.delay) && isfinite(source.delay) ...
            && source.delay==cfg.measurement.fixedLidarDelay, ...
            'VehicleLocalization:FixedLidarDelayMismatch','Declare the same fixed delay on the LiDAR signal and configuration.');
        assert(isfield(source,'headingConvention') && string(source.headingConvention)=="unwrapped", ...
            'VehicleLocalization:HeadingLiftRequired','Continuous LiDAR pose must use an explicitly lifted yaw.');
    end
    assert(~isfield(data,'gps') && (~isfield(data,other) || isempty(fieldnames(data.(other)))), ...
        'VehicleLocalization:MixedMeasurementModes','Provide exactly the selected continuous measurement mode.');
    if isfield(source,'evaluate')
        assert(isa(source.evaluate,'function_handle'),'VehicleLocalization:InvalidContinuousSignal', ...
            'source.evaluate must be a continuous-time function handle.');
        source.representation="function";
    else
        assert(isfield(source,'representation') && string(source.representation)=="piecewiseLinear" ...
            && isfield(source,'time') && isequal(source.time(:),time), ...
            'VehicleLocalization:InvalidContinuousSignal','Arrays must declare piecewiseLinear reconstruction on the input grid.');
        if cfg.mode=="gnss",field="position";width=2;else,field="pose";width=3;end
        assert(isfield(source,field) && isnumeric(source.(field)) && isreal(source.(field)) ...
            && isequal(size(source.(field)),[numel(time),width]) && all(isfinite(source.(field)),'all'), ...
            'VehicleLocalization:InvalidContinuousSignal','Continuous pose samples must be finite and aligned.');
        if cfg.mode=="lidar"
            assert(isfield(source,'information') && isequal(size(source.information),[3,3,numel(time)]), ...
                'VehicleLocalization:InvalidContinuousSignal','LiDAR requires aligned 3-by-3 information matrices.');
            for k=1:numel(time)
                [~,~,info]=computeLidarInformationWeights(source.information(:,:,k),cfg);
                assert(info.qualified,'VehicleLocalization:InsufficientLidarInformation', ...
                    'Every LiDAR direction must satisfy the declared information bound.');
            end
        end
    end
    sourceValue(source,time(1),cfg);
end

function measurement=sourceValue(source,time,cfg)
    if source.representation=="function"
        raw=source.evaluate(time);
        if cfg.mode=="gnss",measurement=struct('position',raw);else,measurement=raw;end
    else
        index=find(source.time<=time,1,'last');index=min(index,numel(source.time)-1);
        alpha=(time-source.time(index))/(source.time(index+1)-source.time(index));
        if cfg.mode=="gnss",field="position";else,field="pose";end
        measurement.(field)=((1-alpha)*source.(field)(index,:)+alpha*source.(field)(index+1,:)).';
        if cfg.mode=="lidar"
            measurement.information=(1-alpha)*source.information(:,:,index)+alpha*source.information(:,:,index+1);
        end
    end
    if cfg.mode=="gnss",field="position";width=2;else,field="pose";width=3;end
    assert(isstruct(measurement) && isfield(measurement,field) && isnumeric(measurement.(field)) ...
        && isreal(measurement.(field)) && numel(measurement.(field))==width && all(isfinite(measurement.(field))), ...
        'VehicleLocalization:InvalidContinuousSignal','The selected measurement is missing or nonfinite.');
    measurement.(field)=double(measurement.(field)(:));
    if cfg.mode=="lidar"
        assert(isfield(measurement,'information'),'VehicleLocalization:InsufficientLidarInformation', ...
            'Continuous LiDAR requires full pose information.');
        [~,~,info,measurement.weight]=computeLidarInformationWeights(measurement.information,cfg);
        assert(info.qualified,'VehicleLocalization:InsufficientLidarInformation', ...
            'The LiDAR signal left its uniformly informative sector.');
    end
end

function state=initialState(cfg,source,high,lateral)
    if ~isempty(cfg.observer.initialState)
        state=cfg.observer.initialState;
        assert(isnumeric(state) && isreal(state) && numel(state)==7 && all(isfinite(state)), ...
            'VehicleLocalization:InvalidInitialState','Initial state must contain seven finite real values.');
        state=double(state(:));return;
    end
    assert(cfg.mode=="gnss",'VehicleLocalization:InitialStateRequired', ...
        'Delayed LiDAR requires an explicit current initial state and matching history.');
    measured=sourceValue(source,high.time(1),cfg);yaw=cfg.observer.initialHeading;
    assert(isscalar(yaw) && isreal(yaw) && isfinite(yaw), ...
        'VehicleLocalization:InvalidInitialState','Initial heading must be finite.');
    rotation=[cos(yaw),-sin(yaw);sin(yaw),cos(yaw)];
    velocity=rotation*[high.longitudinalSpeed(1);lateral.lateralVelocity(1)];
    acceleration=rotation*[high.longitudinalAcceleration(1);high.lateralAcceleration(1)];
    state=[measured.position(1);velocity(1);acceleration(1);measured.position(2);velocity(2);acceleration(2);yaw];
end

function validateRuntimeConfig(cfg)
    step=cfg.measurement.maximumIntegrationStep;
    assert(isscalar(step) && isreal(step) && isfinite(step) && step>0, ...
        'VehicleLocalization:InvalidConfiguration','maximumIntegrationStep must be positive.');
    allowedFields={'fixedLidarDelay','maximumIntegrationStep'};
    assert(all(ismember(fieldnames(cfg.measurement),allowedFields)), ...
        'VehicleLocalization:InvalidConfiguration','Unsupported continuous measurement configuration field.');
end
