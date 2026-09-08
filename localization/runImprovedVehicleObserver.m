function estimate = runImprovedVehicleObserver(sensorData,lateralDesign,observerDesign,cfg)
% runImprovedVehicleObserver Fuse fixed-delay LiDAR without state replay.
% A bounded buffer contains only affine nominal-flow maps derived from q/r.
% Each delayed pose residual and its gain are transported to the current state.
% Integration proceeds forward once; every returned state is causal and final.
    arguments
        sensorData (1,1) struct
        lateralDesign (1,1) struct
        observerDesign (1,1) struct
        cfg (1,1) struct = improvedObserverConfig()
    end
    validateObserverDesign(observerDesign,cfg);
    scaling=diag(cfg.observer.theta.^cfg.observer.scalingExponents(:));
    observerDesign.poseGain=scaling*observerDesign.K;
    observerDesign.invariantPhysicalGain=scaling*observerDesign.N/cfg.observer.theta^3;
    [highRate,gps,lidar]=normalizeSensorData(sensorData,cfg);
    lateral=runLateralVelocityObserver(highRate,lateralDesign,lateralDesign.cfg);
    state=buildInitialState(highRate,lateral,gps,lidar,cfg);
    count=numel(highRate.time);states=zeros(count,7);trace=cell(count,1);
    history=cell(0,1);active=struct('gpsIndex',0,'lidarIndex',0, ...
        'gpsFlow',eye(8),'lidarFlow',eye(8));
    gps.accepted=false(numel(gps.timestamp),1);gps.rejected=gps.accepted;
    lidar.accepted=false(numel(lidar.timestamp),1);lidar.rejected=lidar.accepted;
    gps.rejectionReason=repmat("",numel(gps.timestamp),1);
    lidar.rejectionReason=repmat("",numel(lidar.timestamp),1);
    boundaries=unique([gps.arrivalTime;gps.arrivalTime+cfg.measurement.gpsMaximumAge; ...
        lidar.arrivalTime;lidar.arrivalTime+cfg.measurement.lidarMaximumAge; ...
        lidar.arrivalTime+cfg.measurement.maximumPoseInterval]);
    integrationSteps=0;maximumHistorySegments=0;
    intervals=zeros(0,2);sectorMinimum=zeros(0,1);
    for sampleIdx=1:count
        now=highRate.time(sampleIdx);
        if sampleIdx>1
            left=highRate.time(sampleIdx-1);right=now;
            pieces=max(1,ceil((right-left)/cfg.measurement.maximumIntegrationStep-1e-10));
            cuts=unique([linspace(left,right,pieces+1).';boundaries(boundaries>left & boundaries<right)]);
            for part=1:numel(cuts)-1
                t0=cuts(part);t1=cuts(part+1);
                [gps,lidar,active]=receiveEvents(gps,lidar,active,history,t0,cfg);
                fusion=activeFusion(t0,gps,lidar,active,cfg);
                if fusion.lidarActive
                    intervals(end+1,:)=[t0,t1]; %#ok<AGROW>
                    sectorMinimum(end+1,1)=min(eig(fusion.normalizedWeight)); %#ok<AGROW>
                end
                [state,flow]=forwardStep(state,t0,t1,sampleIdx-1, ...
                    highRate,lateral,active,fusion,observerDesign,cfg);
                if fusion.gpsActive,active.gpsFlow=flow.map*active.gpsFlow;end
                if fusion.lidarActive,active.lidarFlow=flow.map*active.lidarFlow;end
                history{end+1,1}=flow; %#ok<AGROW>
                cutoff=t1-cfg.measurement.inputHistoryDuration;
                while ~isempty(history) && history{1}.right<cutoff
                    history(1)=[];
                end
                maximumHistorySegments=max(maximumHistorySegments,numel(history));
                integrationSteps=integrationSteps+1;
            end
        end
        [gps,lidar,active]=receiveEvents(gps,lidar,active,history,now,cfg);
        fusion=activeFusion(now,gps,lidar,active,cfg);
        sample=measurementSample(highRate,lateral,min(sampleIdx,count-1),double(sampleIdx==count),cfg);
        [~,~,details]=currentDerivative(state,sample,eye(8),active,fusion,observerDesign,cfg);
        details.accepted=nnz(gps.accepted)+nnz(lidar.accepted);
        details.rejected=nnz(gps.rejected)+nnz(lidar.rejected);
        details.fusion=fusion;trace{sampleIdx}=details;
        states(sampleIdx,:)=state.';
    end
    estimate=assembleEstimate(states,trace,highRate,lateral,gps,lidar,observerDesign,cfg);
    estimate.diagnostics.integrationStepCount=integrationSteps;
    estimate.diagnostics.maximumInputHistorySegments=maximumHistorySegments;
    estimate.diagnostics.inputHistoryDuration=cfg.measurement.inputHistoryDuration;
    estimate.diagnostics.stateHistoryRecomputed=false;
    estimate.diagnostics.certificateConditions=transportConditions( ...
        highRate,gps,lidar,intervals,sectorMinimum,cfg);
end

function [state,flow]=forwardStep(state,left,right,index,high,lateral,active,fusion,design,cfg)
% forwardStep Advance the estimate and input-only affine flow with the same RK4.
    dt=right-left;start=high.time(index);span=high.time(index+1)-start;
    first=measurementSample(high,lateral,index,(left-start)/span,cfg);
    middle=measurementSample(high,lateral,index,((left+right)/2-start)/span,cfg);
    last=measurementSample(high,lateral,index,(right-start)/span,cfg);
    [k1,A1]=currentDerivative(state,first,eye(8),active,fusion,design,cfg);
    P2=eye(8)+dt*A1/2;
    [k2,A2]=currentDerivative(state+dt*k1/2,middle,P2,active,fusion,design,cfg);
    V2=A2*P2;P3=eye(8)+dt*V2/2;
    [k3,A3]=currentDerivative(state+dt*k2/2,middle,P3,active,fusion,design,cfg);
    V3=A3*P3;P4=eye(8)+dt*V3;
    [k4,A4]=currentDerivative(state+dt*k3,last,P4,active,fusion,design,cfg);
    V4=A4*P4;
    state=state+dt*(k1+2*k2+2*k3+k4)/6;
    % Keep a continuous yaw lift internally. Only the public angle is wrapped.
    assert(all(isfinite(state)),'VehicleLocalization:NonfiniteObserver', ...
        'The forward observer produced a nonfinite state at %.9g seconds.',right);
    flow=struct('left',left,'right',right,'map',eye(8)+dt*(A1+2*V2+2*V3+V4)/6, ...
        'slopes',cat(3,A1,V2,V3,V4));
end

function [derivative,A,details]=currentDerivative(state,sample,stepFlow,active,fusion,design,cfg)
    channels=evaluateImprovedObserverChannels(state,sample,cfg.operating);
    A=[channels.modelMatrix,channels.modelInput;zeros(1,8)];
    lidarResidual=zeros(3,1);gpsResidual=zeros(3,1);
    lidarGain=zeros(7,3);gpsGain=zeros(7,3);
    if fusion.lidarActive
        M=stepFlow*active.lidarFlow;
        past=M\[state;1];
        lidarResidual=fusion.lidarPose-past([1,4,7]);
        lidarResidual(3)=wrapAngleToPi(lidarResidual(3));
        lidarGain=M(1:7,1:7)*design.poseGain*fusion.lidarWeight;
    end
    if any(fusion.gpsWeight,'all')
        M=stepFlow*active.gpsFlow;
        past=M\[state;1];
        gpsResidual=[fusion.gpsPose-past([1,4]);0];
        gpsGain=M(1:7,1:7)*design.poseGain*fusion.gpsWeight;
    end
    derivative=channels.modelDerivative+lidarGain*lidarResidual+gpsGain*gpsResidual ...
        +design.invariantPhysicalGain*channels.invariantInnovation;
    if nargout>2
        base=lidarResidual;if ~fusion.lidarActive,base=gpsResidual;end
        details=struct('base',base,'lidarResidual',lidarResidual,'gpsResidual',gpsResidual, ...
            'invariant',channels.invariantInnovation,'sensitivity',channels.motionHeadingSensitivity, ...
            'trackRate',channels.trackAngleRate,'lidarGain',lidarGain,'gpsGain',gpsGain);
    end
end

function [gps,lidar,active]=receiveEvents(gps,lidar,active,history,time,cfg)
    [gps,active.gpsIndex,active.gpsFlow]=receiveStream( ...
        gps,active.gpsIndex,active.gpsFlow,history,time,cfg);
    [lidar,active.lidarIndex,active.lidarFlow]=receiveStream( ...
        lidar,active.lidarIndex,active.lidarFlow,history,time,cfg);
end

function [events,index,flow]=receiveStream(events,index,flow,history,time,cfg)
% receiveStream A late pose changes the current correction, never stored states.
    pending=find(~events.accepted & ~events.rejected & events.arrivalTime<=time);
    for k=pending(:).'
        reason="";
        if ~events.qualified(k)
            reason="invalidMeasurement";
        elseif time-events.timestamp(k)>cfg.measurement.inputHistoryDuration
            reason="inputHistoryExpired";
        elseif index>0 && events.timestamp(k)<events.timestamp(index)
            reason="supersededAcquisition";
        else
            [transport,available]=inputTransport(history,events.timestamp(k),time);
            if ~available,reason="inputHistoryUnavailable";end
        end
        if strlength(reason)>0
            events.rejected(k)=true;events.rejectionReason(k)=reason;
        else
            events.accepted(k)=true;events.incorporationTime(k)=time;
            index=k;flow=transport;
        end
    end
end

function [transport,available]=inputTransport(history,timestamp,time)
% inputTransport Compose stored nominal maps; no observer update is reexecuted.
    transport=eye(8);available=timestamp==time;
    if available,return;end
    if isempty(history) || timestamp<history{1}.left || timestamp>time,return;end
    available=true;
    for k=1:numel(history)
        segment=history{k};
        if segment.right<=timestamp,continue;end
        map=segment.map;
        if timestamp>segment.left
            u=(timestamp-segment.left)/(segment.right-segment.left);
            % Third-order RK4 dense extension, exact for constant acceleration.
            b=[u-1.5*u^2+2*u^3/3,u^2-2*u^3/3,u^2-2*u^3/3,-u^2/2+2*u^3/3];
            prefix=eye(8)+(segment.right-segment.left)*sum( ...
                segment.slopes.*reshape(b,1,1,4),3);
            map=map/prefix;
        end
        transport=map*transport;
    end
end

function fusion=activeFusion(time,gps,lidar,active,cfg)
    fusion=struct('lidarActive',false,'gpsActive',false,'lidarWeight',zeros(3), ...
        'gpsWeight',zeros(3),'normalizedWeight',zeros(3),'lidarPose',zeros(3,1), ...
        'gpsPose',zeros(2,1),'gpsAge',NaN,'lidarAge',NaN);
    g=active.gpsIndex;l=active.lidarIndex;recent=false;
    if g>0
        fusion.gpsActive=time-gps.incorporationTime(g)<cfg.measurement.gpsMaximumAge-1e-12;
        fusion.gpsPose=gps.pose(g,1:2).';
        if fusion.gpsActive,fusion.gpsAge=time-gps.timestamp(g);end
    end
    if l>0
        age=time-lidar.incorporationTime(l);
        recent=age<cfg.measurement.maximumPoseInterval-1e-12;
        fusion.lidarActive=age<cfg.measurement.lidarMaximumAge-1e-12;
        if fusion.lidarActive
            fusion.lidarAge=time-lidar.timestamp(l);fusion.lidarPose=lidar.pose(l,:).';
            fusion.lidarWeight=lidar.poseWeight(:,:,l);
            fusion.normalizedWeight=lidar.normalizedWeight(:,:,l);
            if fusion.gpsActive
                fusion.lidarWeight=lidar.gpsFusedLidarWeight(:,:,l);
                fusion.gpsWeight=lidar.gpsFusedGpsWeight(:,:,l);
                fusion.normalizedWeight=lidar.gpsFusedNormalizedWeight(:,:,l);
            end
        end
    end
    if ~recent && fusion.gpsActive
        fusion.gpsWeight=gps.poseWeight;fusion.normalizedWeight=gps.normalizedWeight;
    end
end

function estimate=assembleEstimate(states,trace,high,lateral,gps,lidar,design,cfg)
    n=numel(high.time);public=states;public(:,7)=wrapAngleToPi(public(:,7));
    base=zeros(n,3);lr=zeros(n,3);gr=zeros(n,3);intrinsic=zeros(n,4);
    sensitivity=zeros(n,1);track=zeros(n,1);accepted=zeros(n,1);rejected=zeros(n,1);
    WL=zeros(3,3,n);WG=WL;W=WL;LL=zeros(7,3,n);LG=LL;
    gpsAge=NaN(n,1);lidarAge=NaN(n,1);
    for k=1:n
        item=trace{k};fusion=item.fusion;
        base(k,:)=item.base.';lr(k,:)=item.lidarResidual.';gr(k,:)=item.gpsResidual.';
        intrinsic(k,:)=item.invariant.';sensitivity(k)=item.sensitivity;track(k)=item.trackRate;
        accepted(k)=item.accepted;rejected(k)=item.rejected;
        WL(:,:,k)=fusion.lidarWeight;WG(:,:,k)=fusion.gpsWeight;W(:,:,k)=fusion.normalizedWeight;
        LL(:,:,k)=item.lidarGain;LG(:,:,k)=item.gpsGain;
        gpsAge(k)=fusion.gpsAge;lidarAge(k)=fusion.lidarAge;
    end
    rawRate=high.yawRate+lateral.sideSlipAngleRate;
    outsideV=any(abs(public(:,[2,5]))>cfg.operating.maximumSpeed,2);
    outsideA=any(abs(public(:,[3,6]))>cfg.operating.maximumAcceleration,2);
    diagnostics=struct('translationWeight',WL(1:2,1:2,:),'headingWeight',reshape(WL(3,3,:),[],1), ...
        'lidarPoseWeight',WL,'gpsPoseWeight',WG,'totalNormalizedPoseWeight',W, ...
        'lidarPoseGain',LL,'gpsPoseGain',LG,'informationTimeBasis',"causal delivery-time corrections", ...
        'motionHeadingSensitivity',sensitivity,'gpsAge',gpsAge,'lidarAge',lidarAge, ...
        'rawTrackAngleRate',rawRate,'outsideTrackRateEnvelope',abs(rawRate)>cfg.operating.maximumTrackAngleRate, ...
        'estimatedVelocityOutsideEnvelope',outsideV,'estimatedAccelerationOutsideEnvelope',outsideA, ...
        'acceptedEventCount',accepted,'rejectedEventCount',rejected,'acceptedGps',gps.accepted, ...
        'acceptedLidar',lidar.accepted,'invariantExtensionActive',outsideV|outsideA);
    estimate=struct('time',high.time,'z',public,'onlineZ',public,'pose',public(:,[1,4,7]), ...
        'position',public(:,[1,4]),'velocity',public(:,[2,5]),'acceleration',public(:,[3,6]), ...
        'heading',public(:,7),'speed',hypot(public(:,2),public(:,5)), ...
        'sideSlipAngle',lateral.sideSlipAngle,'sideSlipAngleRate',lateral.sideSlipAngleRate, ...
        'trackAngleRate',track,'lateral',lateral, ...
        'measurements',struct('highRate',high,'gps',gps,'lidar',lidar), ...
        'innovations',struct('base',base,'lidarPosition',lr(:,1:2),'lidar',lr,'gps',gr,'invariant',intrinsic), ...
        'diagnostics',diagnostics);
    estimate.observer=struct('kind',"fixed-delay-transport-v1",'P',design.P,'K',design.K,'N',design.N, ...
        'theta',cfg.observer.theta,'sigma',cfg.observer.sigma,'certificateVerified',false,'certified',false, ...
        'referenceCertificateVerified',design.certified,'referenceVerification',design.verification, ...
        'verification',struct('certified',false,'scope',"fixed-delay transport inequalities not verified"), ...
        'scope',"input-flow transport with fixed LiDAR delay; historical current-pose timer certificate is not applicable");
end

function conditions=transportConditions(high,gps,lidar,intervals,minimumWeights,cfg)
    stamps=sort(lidar.timestamp(lidar.accepted));gaps=diff(stamps);
    if isempty(gaps),largest=Inf;else,largest=max(gaps);end
    values=zeros(nnz(lidar.accepted),1);selected=find(lidar.accepted);
    for k=1:numel(selected),values(k)=min(eig(lidar.normalizedWeight(:,:,selected(k))));end
    referenceSector=~isempty(minimumWeights) && all(minimumWeights>=cfg.lidar.certificateMinimumPoseWeight-1e-9);
    short=gaps<cfg.measurement.minimumPoseInterval-1e-9;long=gaps>cfg.measurement.maximumPoseInterval+1e-9;
    la=lidar.incorporationTime(lidar.accepted)-lidar.timestamp(lidar.accepted);
    ga=gps.incorporationTime(gps.accepted)-gps.timestamp(gps.accepted);
    conditions=struct('fixedDelayMatchesConfiguration',true, ...
        'configuredFixedDelaySeconds',cfg.measurement.fixedLidarDelay, ...
        'lidarIncorporationTime',lidar.incorporationTime,'gpsIncorporationTime',gps.incorporationTime, ...
        'lidarAssimilationDelaySeconds',la,'gpsAssimilationDelaySeconds',ga, ...
        'lidarArrivalToProcessingWaitSeconds',lidar.incorporationTime(lidar.accepted)-lidar.arrivalTime(lidar.accepted), ...
        'gpsArrivalToProcessingWaitSeconds',gps.incorporationTime(gps.accepted)-gps.arrivalTime(gps.accepted), ...
        'maximumObservedAssimilationDelaySeconds',max([0;la;ga]), ...
        'configuredCausalLagBoundSeconds',cfg.measurement.fixedLidarDelay, ...
        'maximumProcessingGridIntervalSeconds',max(diff(high.time)), ...
        'assimilationTimeBasis',"event-driven sensor clock; wall-clock execution unbudgeted", ...
        'allAcceptedDeliveryTimesProvided',all(lidar.deliveryTimeProvided(lidar.accepted)) && all(gps.deliveryTimeProvided(gps.accepted)), ...
        'lidarDeliveryTimesAssumedFromFixedDelay',~lidar.deliveryTimeProvided, ...
        'minimumLidarWeightEigenvalues',values,'weightSectorViolationCount',nnz(values<cfg.lidar.certificateMinimumPoseWeight-1e-9), ...
        'lidarInformationWithinReferenceSector',~isempty(values) && all(values>=cfg.lidar.certificateMinimumPoseWeight-1e-9), ...
        'posePulseInformationIntervals',intervals,'minimumCombinedWeightEigenvalues',minimumWeights, ...
        'combinedWeightSectorViolationCount',nnz(minimumWeights<cfg.lidar.certificateMinimumPoseWeight-1e-9), ...
        'informationWithinReferenceSector',referenceSector,'maximumQualifiedPoseGapSeconds',largest, ...
        'qualifiedPoseIntervals',gaps,'shortIntervalCount',nnz(short),'longIntervalCount',nnz(long), ...
        'timingWithinReferenceSchedule',~isempty(gaps) && ~any(short|long), ...
        'informationWithinCertificate',false,'timingWithinCertificate',false,'certificateApplicable',false, ...
        'informationAuditScope',"source-weight sector on actual delivery pulses; transport Jacobians require a new certificate", ...
        'poseErrorBoundValidated',logical(cfg.lidar.errorBoundValidated), ...
        'upstreamDisturbanceBoundValidated',false,'numericalErrorBoundValidated',false,'unconditionalStabilityClaimed',false);
end

function sample = measurementSample(highRate, lateralEstimate, intervalIdx, alpha, cfg)
% measurementSample Interpolate the cascade inputs inside one HGO step.
    interpolation = string(cfg.measurement.inputInterpolation);
    sample = struct();
    sample.longitudinalSpeed = interpolateValue(highRate.longitudinalSpeed, intervalIdx, alpha, interpolation);
    sample.longitudinalAcceleration = interpolateValue( ...
        highRate.longitudinalAcceleration, intervalIdx, alpha, interpolation);
    sample.lateralAcceleration = interpolateValue(highRate.lateralAcceleration, intervalIdx, alpha, interpolation);
    sample.yawRate = interpolateValue(highRate.yawRate, intervalIdx, alpha, interpolation);
    sample.lateralVelocity = interpolateValue(lateralEstimate.lateralVelocity, intervalIdx, alpha, interpolation);
    sample.sideSlipAngle = interpolateAngle(lateralEstimate.sideSlipAngle, intervalIdx, alpha, interpolation);
    rawSideSlipRate = interpolateValue(lateralEstimate.sideSlipAngleRate, intervalIdx, alpha, interpolation);
    rawTrackRate = sample.yawRate + rawSideSlipRate;
    if logical(cfg.observer.clampTrackAngleRateToDesignEnvelope)
        maximumRate = double(cfg.operating.maximumTrackAngleRate);
        usedTrackRate = min(max(rawTrackRate, -maximumRate), maximumRate);
        sample.sideSlipAngleRate = usedTrackRate - sample.yawRate;
    else
        sample.sideSlipAngleRate = rawSideSlipRate;
    end
end

function eventIdx = latestEligibleEvent(events, accepted, queryTime)
% latestEligibleEvent Select the most recent accepted physical timestamp.
    eligible = accepted & events.timestamp <= queryTime;
    if ~any(eligible)
        eventIdx = [];
        return;
    end
    latestTimestamp = max(events.timestamp(eligible));
    eventIdx = find(eligible & events.timestamp == latestTimestamp, 1, "last");
end

function initialState = buildInitialState(highRate, lateralEstimate, gps, lidar, cfg)
% buildInitialState Build [X,Vx,Ax,Y,Vy,Ay,heading] without zero-speed division.
    if ~isempty(cfg.observer.initialState)
        initialState = double(cfg.observer.initialState(:));
        assert(numel(initialState) == 7 && all(isfinite(initialState)), ...
            "cfg.observer.initialState must be empty or a finite [7 x 1] vector.");
        initialState(7) = wrapAngleToPi(initialState(7));
        return;
    end

    initialTime = highRate.time(1);
    initialGps = gps.qualified & gps.arrivalTime <= initialTime & gps.timestamp == initialTime;
    initialLidar = lidar.qualified & lidar.arrivalTime <= initialTime & lidar.timestamp == initialTime;
    position = double(cfg.observer.fallbackPosition(:));
    heading = double(cfg.observer.fallbackHeading);
    gpsIdx = latestEligibleEvent(gps, initialGps, initialTime);
    lidarIdx = latestEligibleEvent(lidar, initialLidar, initialTime);
    if any(initialGps)
        position = gps.pose(gpsIdx, 1:2).';
    end
    if ~isempty(lidarIdx)
        if lidar.informationDiagnostics{lidarIdx}.rank == 3
            % Preserve full-pose initialization and GPS position precedence.
            if isempty(gpsIdx), position = lidar.pose(lidarIdx, 1:2).'; end
            heading = lidar.pose(lidarIdx, 3);
        else
            % An arbitrary representative in a LiDAR nullspace is not an
            % initial measurement. Apply its bounded directional weight to
            % the fallback/GPS prior, on the same local yaw branch.
            residual = [lidar.pose(lidarIdx, 1:2).'-position; ...
                wrapAngleToPi(lidar.pose(lidarIdx, 3)-heading)];
            correction = lidar.poseWeight(:,:,lidarIdx)*residual;
            if isempty(gpsIdx), position = position+correction(1:2); end
            heading = heading+correction(3);
        end
    end

    bodyVelocity = [highRate.longitudinalSpeed(1); lateralEstimate.lateralVelocity(1)];
    bodyAcceleration = [highRate.longitudinalAcceleration(1); highRate.lateralAcceleration(1)];
    rotation = [cos(heading), -sin(heading); sin(heading), cos(heading)];
    globalVelocity = rotation * bodyVelocity;
    globalAcceleration = rotation * bodyAcceleration;
    initialState = [position(1); globalVelocity(1); globalAcceleration(1); ...
        position(2); globalVelocity(2); globalAcceleration(2); wrapAngleToPi(heading)];
end

function [highRate, gps, lidar] = normalizeSensorData(sensorData, cfg)
% normalizeSensorData Validate the common-rate stream and asynchronous events.
    assert(isfield(sensorData, "highRate") && isstruct(sensorData.highRate), ...
        "sensorData.highRate is required.");
    highRate = normalizeHighRate(sensorData.highRate);
    if isfield(sensorData, "gps")
        gps = normalizePoseEvents(sensorData.gps, false, 0);
    else
        gps = emptyEvents(false);
    end
    if isfield(sensorData, "lidar")
        lidar = normalizePoseEvents(sensorData.lidar, true, cfg.measurement.fixedLidarDelay);
    else
        lidar = emptyEvents(true);
    end
    gps.qualified = all(isfinite(gps.pose(:,1:2)),2);
    gps.incorporationTime = NaN(numel(gps.timestamp),1);
    lidar.incorporationTime = NaN(numel(lidar.timestamp),1);
    lidar.qualified = false(numel(lidar.timestamp),1);
    lidar.headingWeight = zeros(numel(lidar.timestamp),1);
    lidar.poseWeight=zeros(3,3,numel(lidar.timestamp));
    lidar.normalizedWeight=lidar.poseWeight;
    lidar.gpsFusedLidarWeight=lidar.poseWeight;lidar.gpsFusedGpsWeight=lidar.poseWeight;
    lidar.gpsFusedNormalizedWeight=lidar.poseWeight;
    [~,gps.poseWeight,gps.normalizedWeight]=fusePoseInformationWeights(zeros(3),true,cfg);
    lidar.informationDiagnostics = cell(numel(lidar.timestamp),1);
    for eventIdx=1:numel(lidar.timestamp)
        [~,weight,info,W] = computeLidarInformationWeights(lidar.information(:,:,eventIdx),cfg);
        lidar.qualified(eventIdx) = info.qualified && all(isfinite(lidar.pose(eventIdx,:)));
        lidar.headingWeight(eventIdx) = weight;
        lidar.poseWeight(:,:,eventIdx)=W;lidar.normalizedWeight(:,:,eventIdx)=info.normalizedWeight;
        lidar.gpsFusedLidarWeight(:,:,eventIdx)=info.gpsFusedLidarWeight;
        lidar.gpsFusedGpsWeight(:,:,eventIdx)=info.gpsFusedGpsWeight;
        lidar.gpsFusedNormalizedWeight(:,:,eventIdx)=info.gpsFusedNormalizedWeight;
        lidar.informationDiagnostics{eventIdx} = info;
    end
    tolerance = double(cfg.measurement.timestampTolerance);
    assert(all(gps.arrivalTime + tolerance >= gps.timestamp), ...
        "GPS arrival times must not precede their physical timestamps.");
    assert(all(lidar.arrivalTime + tolerance >= lidar.timestamp), ...
        "Lidar arrival times must not precede their physical timestamps.");
end

function highRate = normalizeHighRate(highRate)
% normalizeHighRate Return finite aligned column vectors with nonnegative speed.
    requiredFields = ["time", "steeringAngle", "longitudinalSpeed", ...
        "longitudinalAcceleration", "lateralAcceleration", "yawRate"];
    sampleCount = numel(highRate.time);
    assert(sampleCount >= 2, "The high-rate stream must contain at least two samples.");
    for fieldName = requiredFields
        assert(isfield(highRate, fieldName), "sensorData.highRate.%s is required.", fieldName);
        values = double(highRate.(fieldName)(:));
        assert(numel(values) == sampleCount && all(isfinite(values)), ...
            "sensorData.highRate.%s must be a finite aligned series.", fieldName);
        highRate.(fieldName) = values;
    end
    if isfield(highRate, "dynamicValid")
        dynamicValid = highRate.dynamicValid(:);
        assert((islogical(dynamicValid) || isnumeric(dynamicValid)) && ...
            numel(dynamicValid) == sampleCount && all(isfinite(double(dynamicValid))), ...
            "sensorData.highRate.dynamicValid must be a finite aligned logical series.");
        highRate.dynamicValid = logical(dynamicValid);
    end
    assert(all(diff(highRate.time) > 0.0), "sensorData.highRate.time must be strictly increasing.");
    assert(all(highRate.longitudinalSpeed >= 0.0), ...
        "sensorData.highRate.longitudinalSpeed must be nonnegative.");
end

function events = normalizePoseEvents(rawEvents, includeInformation, fixedDelay)
% normalizePoseEvents Normalize timestamped GPS or lidar event structures.
    if isempty(rawEvents) || (isstruct(rawEvents) && isempty(fieldnames(rawEvents)))
        events = emptyEvents(includeInformation);
        return;
    end
    assert(isstruct(rawEvents), "GPS and lidar event streams must be structs.");
    requiredFields = ["timestamp", "pose"];
    for fieldName = requiredFields
        assert(isfield(rawEvents, fieldName), "Event stream is missing %s.", fieldName);
    end
    timestamp = double(rawEvents.timestamp(:));
    eventCount = numel(timestamp);
    pose = double(rawEvents.pose);
    if size(pose, 1) ~= eventCount && size(pose, 2) == eventCount
        pose = pose.';
    end
    minimumWidth = 2 + double(includeInformation);
    assert(size(pose, 1) == eventCount && size(pose, 2) >= minimumWidth, ...
        "Event pose must have one row per timestamp and at least %d columns.", minimumWidth);
    pose = pose(:, 1:minimumWidth);
    if ~includeInformation
        pose(:, 3) = NaN;
    end
    if isfield(rawEvents, "arrivalTime")
        arrivalTime = double(rawEvents.arrivalTime(:));
    else
        arrivalTime = timestamp;
    end
    assert(numel(arrivalTime) == eventCount && all(isfinite(timestamp)) && all(isfinite(arrivalTime)), ...
        "Event timestamps and arrival times must be finite aligned vectors.");

    events = struct();
    events.timestamp = timestamp;
    events.arrivalTime = arrivalTime;
    events.deliveryTimeProvided = repmat(isfield(rawEvents,"arrivalTime"),eventCount,1);
    if isfield(rawEvents,"arrivalTimeIsPlaceholder")
        placeholder=logical(rawEvents.arrivalTimeIsPlaceholder(:));
        assert(isscalar(placeholder) || numel(placeholder)==eventCount, ...
            'Delivery placeholder flags must be scalar or aligned.');
        events.deliveryTimeProvided=events.deliveryTimeProvided & ~(placeholder & arrivalTime==timestamp);
    end
    if includeInformation
        assumed=~events.deliveryTimeProvided;
        events.arrivalTime(assumed)=timestamp(assumed)+fixedDelay;
        assert(all(abs(events.arrivalTime-timestamp-fixedDelay)<1e-8), ...
            'VehicleLocalization:FixedLidarDelayMismatch', ...
            'LiDAR delivery must equal acquisition plus cfg.measurement.fixedLidarDelay.');
    end
    events.pose = pose;
    if includeInformation
        events.information = normalizeInformation(rawEvents, eventCount);
    else
        events.information = NaN(3, 3, eventCount);
    end
    [~, order] = sortrows([events.arrivalTime, events.timestamp]);
    events.timestamp = events.timestamp(order);
    events.arrivalTime = events.arrivalTime(order);
    events.deliveryTimeProvided = events.deliveryTimeProvided(order);
    events.pose = events.pose(order, :);
    events.information = events.information(:, :, order);
end

function information = normalizeInformation(rawEvents, eventCount)
% normalizeInformation Accept [3 x 3 x N], [N x 9], or missing Hessians.
    information = NaN(3, 3, eventCount);
    if ~isfield(rawEvents, "information") || isempty(rawEvents.information)
        return;
    end
    rawInformation = double(rawEvents.information);
    if isequal(size(rawInformation), [3, 3]) && eventCount == 1
        information(:, :, 1) = rawInformation;
    elseif ndims(rawInformation) == 3 && isequal(size(rawInformation, 1), 3) && ...
            isequal(size(rawInformation, 2), 3) && size(rawInformation, 3) == eventCount
        information = rawInformation;
    elseif isequal(size(rawInformation), [eventCount, 9])
        for eventIdx = 1:eventCount
            information(:, :, eventIdx) = reshape(rawInformation(eventIdx, :), 3, 3);
        end
    else
        error("Lidar information must be [3 x 3 x N] or [N x 9].");
    end
end

function events = emptyEvents(includeInformation)
% emptyEvents Return a correctly shaped empty asynchronous stream.
    poseWidth = 2 + double(includeInformation);
    events = struct("timestamp", zeros(0, 1), "arrivalTime", zeros(0, 1), ...
        "pose", zeros(0, poseWidth), "information", NaN(3, 3, 0), ...
        "deliveryTimeProvided",false(0,1));
end

function value = interpolateValue(values, intervalIdx, alpha, interpolation)
% interpolateValue Apply linear interpolation or zero-order hold.
    nextIdx = min(intervalIdx + 1, numel(values));
    if lower(strtrim(string(interpolation))) == "zoh"
        value = values(intervalIdx);
    else
        value = (1.0 - alpha) .* values(intervalIdx) + alpha .* values(nextIdx);
    end
end

function angle = interpolateAngle(values, intervalIdx, alpha, interpolation)
% interpolateAngle Interpolate through the shortest wrapped angular increment.
    nextIdx = min(intervalIdx + 1, numel(values));
    if lower(strtrim(string(interpolation))) == "zoh"
        angle = values(intervalIdx);
    else
        increment = wrapAngleToPi(values(nextIdx) - values(intervalIdx));
        angle = wrapAngleToPi(values(intervalIdx) + alpha .* increment);
    end
end

function validateObserverDesign(design, cfg)
% validateObserverDesign Check gain provenance, not a transported-delay theorem.
    assert(isfield(cfg.measurement,'inputHistoryDuration') ...
        && isfinite(cfg.measurement.inputHistoryDuration) ...
        && isfinite(cfg.measurement.fixedLidarDelay) && cfg.measurement.fixedLidarDelay>=0 ...
        && cfg.measurement.inputHistoryDuration>=cfg.measurement.fixedLidarDelay+cfg.measurement.maximumIntegrationStep, ...
        'VehicleLocalization:InputHistoryTooShort','Input history must cover the fixed delay and one integration step.');
    assert(isfield(design,'kind') && string(design.kind)=="aperiodic-anisotropic-pose-v2", ...
        'VehicleLocalization:CertificateMismatch','Runtime requires the verified reference gain artifact.');
    assert(design.certified && design.verification.certified, ...
        'VehicleLocalization:CertificateMismatch','The pose design is not verified.');
    assert(isfield(design.verification,'verifiedModel'), ...
        'VehicleLocalization:CertificateMismatch','The certificate lacks its verified model snapshot.');
    verified = design.verification.verifiedModel;
    assert(isequal(design.K,verified.K) && isequal(design.N,verified.N) ...
        && isequal(design.timer.P,verified.P) ...
        && isequal(design.timer.knots,verified.knots) ...
        && design.timer.rate==verified.rate && design.timer.alpha==verified.alpha ...
        && isequal(design.timer.multipliers,verified.multipliers) ...
        && isequal(cfg.lidar.poseScales(:),verified.poseScales(:)) ...
        && isequal(design.timer.poseScales(:),verified.poseScales(:)) ...
        && design.timer.onTime==verified.onTime && design.timer.tMin==verified.tMin ...
        && design.timer.tMax==verified.tMax ...
        && cfg.observer.theta==verified.theta ...
        && design.theta==verified.theta && design.timer.theta==verified.theta ...
        && isequal(cfg.operating,verified.operating) ...
        && isequal(cfg.observer.scalingExponents(:),verified.scalingExponents(:)), ...
        'VehicleLocalization:CertificateMismatch','Runtime matrices or envelope differ from verification.');
    expected = [verified.theta; verified.theta; verified.onTime; verified.onTime; ...
        verified.tMin; verified.tMax; verified.alpha];
    actual = [cfg.observer.theta; cfg.observer.sigma; cfg.measurement.gpsMaximumAge; ...
        cfg.measurement.lidarMaximumAge; cfg.measurement.minimumPoseInterval; ...
        cfg.measurement.maximumPoseInterval; cfg.lidar.certificateMinimumPoseWeight];
    assert(all(abs(expected-actual)<1e-12) ...
        && max(abs(design.N-cfg.observer.invariantGain),[],'all')<1e-12, ...
        'VehicleLocalization:CertificateMismatch','Runtime gains or pulse timing differ from verification.');
    assert(string(cfg.observer.integrationMethod)=="rk4" ...
        && cfg.measurement.maximumIntegrationStep>0 ...
        && cfg.measurement.maximumIntegrationStep<=.01 ...
        && cfg.measurement.timestampTolerance==0, ...
        'VehicleLocalization:CertificateMismatch','Use event-split RK4 with no future timestamp allowance.');
end

function wrappedAngle = wrapAngleToPi(angle)
% wrapAngleToPi Wrap radians to [-pi, pi).
    wrappedAngle = mod(angle + pi, 2.0 .* pi) - pi;
end
