function estimate=integrate(data,cfg,lateral,L)
% Shared time integration for the map observer and synthetic innovation tests.
    h=data.highRate;t=h.time(:);n=numel(t);
    assert(n>=2 && all(isfinite(t)) && all(diff(t)>0), ...
        'VehicleLocalization:InvalidSyncClock','Require an increasing frame clock.');
    assert(isfield(lateral,'time') && isequal(lateral.time(:),t), ...
        'VehicleLocalization:AlignedLateralRequired','Supply lateral estimates on the frame clock.');
    assert(all(isfield(lateral,{'lateralVelocity'})), ...
        'VehicleLocalization:AlignedLateralRequired', ...
        'Supply the upstream lateral velocity.');
    reportedLateral=lateral;
    if isfield(lateral,'minimumDesignSpeedMps')
        lateral=globalObserverMotionMeasurements(lateral,lateral.measurements.longitudinalSpeed,lateral.measurements.yawRate);
        low=lateral.lowSpeedMeasurement;
        h.longitudinalSpeed(low)=lateral.longitudinalSpeed(low);
    end
    [upstreamRate,rateValid]=observerInternal.upstreamYawRate(lateral);
    velocityValid=motionValidity(h,'bodyVelocityValid',n) ...
        & motionValidity(lateral,'velocityValid',n);
    accelerationValid=motionValidity(h,'bodyAccelerationValid',n);
    fields=["longitudinalSpeed","longitudinalAcceleration","lateralAcceleration"];
    for name=fields
        valid=true(n,1);
        if name=="longitudinalSpeed",valid=velocityValid;end
        if any(name==["longitudinalAcceleration","lateralAcceleration"]),valid=accelerationValid;end
        assert(numel(h.(name))==n && all(isfinite(h.(name)(valid))), ...
            'VehicleLocalization:InvalidFullInput','Invalid aligned motion.');
        h.(name)=h.(name)(:);h.(name)(~valid)=0;
    end
    for name="lateralVelocity"
        valid=velocityValid;
        assert(numel(lateral.(name))==n && all(isfinite(lateral.(name)(valid))), ...
            'VehicleLocalization:InvalidFullInput','Invalid aligned lateral output.');
        lateral.(name)=lateral.(name)(:);lateral.(name)(~valid)=0;
    end
    G=gnssPositionSource(data,t);
    heading=localizationSupport.headingMeasurements(data,t,cfg);
    poseValid=G.valid & heading.valid;
    overlapDiagnostics=struct('evaluated',false(n,1),'available',false(n,1),'similarity',nan(n,1), ...
        'minimumInformation',nan(n,1),'seconds',nan(n,1),'gradient',nan(n,3), ...
        'channels',{cell(n,1)});
    alignment=struct('bodyOffset',[0;0],'bodyCovariance',zeros(2),'headingStdRad',0);
    if isfield(cfg.gnss,'outputPoint'),alignment=cfg.gnss.outputPoint;end
    recoveredHeading=nan(n,1);
    if any(G.valid & ~heading.valid & velocityValid & rateValid)
        if ~isfield(cfg.gnss,'positionStdM')
            defaults=mncavFullObserverConfig();cfg.gnss.positionStdM=defaults.gnss.positionStdM;
        end
        twist=[h.longitudinalSpeed,lateral.lateralVelocity,upstreamRate];
        recoveredHeading=observerInternal.displacementHeading(t,G,heading.valid, ...
            twist,velocityValid & rateValid,alignment.bodyOffset,cfg.gnss.positionStdM);
    end
    angularObservation=heading.heading;
    recovered=isfinite(recoveredHeading);
    angularObservation(recovered)=recoveredHeading(recovered);
    angularValid=heading.valid | recovered;
    [continuous,cfg]=localizationSupport.designFullObserverGains(cfg);
    lidarInjection=continuous.lidarMatched;
    stateCount=8;
    poseJacobian=zeros(3,stateCount);poseJacobian(1,1)=1;poseJacobian(2,4)=1;poseJacobian(3,7)=1;
    linearOutput=zeros(4,stateCount);
    linearOutput(1:3,:)=poseJacobian;
    linearOutput(4,8)=1;
    headingKinematics=zeros(stateCount);headingKinematics(7,8)=1;
    design=struct('kind',"nominal-fusion-implicit",'continuousDesign',continuous, ...
        'lidarMatched',lidarInjection,'lidarJumpCertificate',"Implicit locally affine fixed-metric contraction", ...
        'designRoute',continuous.designRoute, ...
        'sampledSystemCertified',false);
    x=cfg.initialState;
    assert(ismember(numel(x),[7,stateCount]) && all(isfinite(x)),'VehicleLocalization:FullInitializationRequired', ...
        'Supply an explicit common initial state.');x=x(:);
    if numel(x)==7,x(8)=0;end
    z=zeros(n,stateCount);correction=zeros(n,4);
    yawCorrection=zeros(n,1);gnssPosition=nan(n,2);
    headingInnovation=nan(n,1);
    rateInnovation=nan(n,1);
    lidarStrength=nan(n,1);maximumRate=0;
    lidarAudits=cell(n,1);poseWeights=zeros(3,3,n);preLidarStates=zeros(n,stateCount);
    intrinsicStates=zeros(n,stateCount);invariantAudits=cell(n,1);
    informationBounds=nan(n,3);
    lidarPoseSupport=struct('rank',zeros(n,1),'minimumStrength',zeros(n,1), ...
        'fullPoseSupported',false(n,1),'headingStrengthAfterPosition',zeros(n,1), ...
        'headingSupported',false(n,1),'sectorQualified',false(n,1), ...
        'correctionApplied',false(n,1),'sourceValid',false(n,1));
    for k=1:n
        dt=0;if k>1,dt=t(k)-t(k-1);end
        speed=h.longitudinalSpeed(k);vy=lateral.lateralVelocity(k);
        rate=x(8);
        maximumRate=max(maximumRate,abs(rate));
        x=localizationSupport.predict(x,dt,cfg);
        intrinsicStates(k,:)=x.';
        observation=linearOutput*x;
        antennaYaw=x(7);
        if angularValid(k)
            antennaYaw=angularObservation(k);
            headingInnovation(k)=atan2(sin(antennaYaw-x(7)),cos(antennaYaw-x(7)));
            observation(3)=x(7)+headingInnovation(k);
        end
        if G.valid(k)
            gnssPosition(k,:)=localizationSupport.correctGnssOutputPoint(G.values(k,:),[],antennaYaw,alignment);
            observation(1:2)=gnssPosition(k,:).';
        end
        if rateValid(k)
            observation(4)=upstreamRate(k);rateInnovation(k)=upstreamRate(k)-x(8);
        end
        rows=[G.valid(k),G.valid(k),angularValid(k),rateValid(k)];
        Kg=continuous.linearOutputGain(:,rows);Cg=linearOutput(rows,:);observation=observation(rows);
        % The empty packet contributes a zero right-hand side to the same solve.
        x=x+(eye(8)+dt*(Kg*Cg-headingKinematics))\(dt*Kg*(observation-Cg*x));
        gnssRate=Kg([1,4],:)*(observation-Cg*x);
        packet=localizationSupport.measurements(speed,vy,h.longitudinalAcceleration(k), ...
            h.lateralAcceleration(k), ...
            velocityValid(k),accelerationValid(k),cfg);
        [x,invariantAudits{k}]=localizationSupport.correct(x,packet,dt,continuous,cfg);
        preLidarStates(k,:)=x.';
        if L.valid(k)
            measurement=L.evaluate(k,x([1,4,7]));
            assert(~isfield(measurement,'pose') && (isfield(measurement,'gradient') || isfield(measurement,'residual')), ...
                'VehicleLocalization:ExternalLidarPose','LiDAR feedback must be evaluated at the prediction.');
            if isfield(measurement,'similarity') && isfield(measurement,'channels')
                overlapDiagnostics.evaluated(k)=true;overlapDiagnostics.available(k)=measurement.available;
                overlapDiagnostics.similarity(k)=measurement.similarity;overlapDiagnostics.seconds(k)=measurement.seconds;
                overlapDiagnostics.minimumInformation(k)=measurement.minimumInformation;
                overlapDiagnostics.gradient(k,:)=measurement.gradient.';
                overlapDiagnostics.channels{k}=measurement.channels;
                L.valid(k)=measurement.available;
            end
        end
        if L.valid(k)
            [delta,audit]=localizationSupport.computeLidarMatchedCorrection(measurement,x([1,4,7]),poseJacobian, ...
                lidarInjection,cfg,StepSize=dt,Discretization="implicit");
            assert(audit.jumpNonexpansive,'VehicleLocalization:LidarJumpCertificateFailed', ...
                'The numerical LiDAR jump failed its fixed-metric energy check.');
            x=x+delta;lidarAudits{k}=audit;poseWeights(:,:,k)=audit.filter.S;
            S=audit.filter.S;
            informationBounds(k,:)=[min(eig(S(1:2,1:2))),S(3,3),norm(S(1:2,3))];
            support=localizationSupport.lidarPoseSupport(audit.filter,cfg);
            for name=string(fieldnames(support)).'
                lidarPoseSupport.(name)(k)=support.(name);
            end
            lidarPoseSupport.correctionApplied(k)=support.rank>0 && dt>0;
            rateCorrection=audit.continuousCorrection;
            if dt>0,rateCorrection=delta/dt;end
            correction(k,3:4)=rateCorrection([1,4]).';yawCorrection(k)=rateCorrection(7);
            lidarStrength(k)=min(audit.filter.strengths);
        end
        assert(all(isfinite(x)),'VehicleLocalization:NonfiniteObserver','Synchronous update became nonfinite.');
        correction(k,1:2)=gnssRate.';
        z(k,:)=x.';
    end
    lidarPoseSupport.sourceValid=L.valid;
    estimate=struct('time',t,'z',z(:,1:7),'pose',[z(:,[1,4]),wrap(z(:,7))], ...
        'position',z(:,[1,4]),'velocity',z(:,[2,5]),'acceleration',z(:,[3,6]), ...
        'heading',wrap(z(:,7)),'headingUnwrapped',z(:,7),'lateral',reportedLateral,'observer',design);
    measuredSpeed=hypot(h.longitudinalSpeed(:),lateral.lateralVelocity(:));
    speedInterval=[0,cfg.iss.maximumSpeed];
    speedInDomain=velocityValid & measuredSpeed>=speedInterval(1) & measuredSpeed<=speedInterval(2);
    measuredInnovation=headingInnovation;measuredInnovation(~heading.valid)=NaN;
    estimate.diagnostics=struct('gnssPoseValid',poseValid,'lidarValid',logical(L.valid(:)),'positionCorrection',correction, ...
        'gnssPositionAtObserverPoint',gnssPosition, ...
        'yawCorrection',yawCorrection, ...
        'lidarMatched',{lidarAudits},'poseWeight',poseWeights,'preLidarState',preLidarStates, ...
        'lidarInformationBounds',informationBounds, ...
        'independentHeadingValid',heading.valid, ...
        'independentHeadingInnovation',measuredInnovation,'recoveredHeading',recoveredHeading, ...
        'observedMotionEnvelopeSatisfied', ...
            all(velocityValid) && all(accelerationValid) ...
            && all(speedInDomain) ...
            && all(hypot(h.longitudinalAcceleration(:),h.lateralAcceleration(:))<=cfg.iss.maximumAcceleration) ...
            && maximumRate<=cfg.maximumBodyYawRate, ...
        'issSpeedInterval',speedInterval,'observedSpeedWithinIssDomain',speedInDomain, ...
        'zeroSpeedCertified',false,'minimumLidarStrength',lidarStrength,'gnssPositionValid',G.valid, ...
        'maximumBodyYawRate',maximumRate, ...
        'rateEnvelopeSatisfied',maximumRate<=cfg.maximumBodyYawRate, ...
        'packetCounts',struct('gnssValid',nnz(G.valid | heading.valid),'gnssInvalid',nnz(~(G.valid | heading.valid)), ...
        'lidarValid',nnz(L.valid),'lidarInvalid',nnz(~L.valid), ...
        'bodyVelocityValid',nnz(velocityValid),'bodyAccelerationValid',nnz(accelerationValid)), ...
        'localizationUpdates',n-1, ...
        'nominalOutputRateHz',1/median(diff(t)),'referenceUsed',false, ...
        'lidarChannel',L.name, ...
        'lidarOverlap',overlapDiagnostics, ...
        'lidarPoseSupport',lidarPoseSupport, ...
        'motionOutputValidity',struct('bodyVelocity',velocityValid, ...
            'bodyAcceleration',velocityValid & accelerationValid), ...
        'allTheoremHypothesesVerified',false,'sampledSystemCertified',false, ...
        'scope',"Sampled implementation of the common continuous-time design. The certificate assumes a compact operating set and a lower bound on total pose feedback. Frame diagnostics report measured motion and surrogate LiDAR information. Synchronization is offline.");
    estimate.internalState=z;estimate.yawRate=z(:,8);
    estimate.diagnostics.intrinsicPrediction=intrinsicStates;
    estimate.diagnostics.invariantOutputs=invariantAudits;
    estimate.diagnostics.invariantOutputNames=["speedSquared","velocityAccelerationDot", ...
        "velocityAccelerationCross","bodyLateralVelocity"];
    estimate.diagnostics.intrinsicPredictionUsesImu=false;
    estimate.diagnostics.yawRateFeedbackSource="upstreamMotionPacket";
    estimate.diagnostics.upstreamYawRateValid=rateValid;
    estimate.diagnostics.upstreamYawRateInnovation=rateInnovation;
    estimate.diagnostics.packetCounts.upstreamYawRateValid=nnz(rateValid);
    estimate.diagnostics.actualOverlapCertified=false;
    certificate=continuous.continuousCertificate.verification;
    estimate.diagnostics.operatingSet=certificate.operatingSet;
    estimate.diagnostics.nominalFusionLmiVerified=certificate.nominalFusionLmiVerified;
    estimate.diagnostics.allAuxiliaryOutputsAvailable=velocityValid & accelerationValid;
    estimate.diagnostics.auxiliaryAvailabilityContractSatisfied= ...
        ~certificate.allAuxiliaryOutputsRequired || all(estimate.diagnostics.allAuxiliaryOutputsAvailable);
end

function valid=motionValidity(h,name,n)
% Missing masks preserve the existing measured-motion contract. Withdrawing
% an output removes both its residual and feedback; invalid payloads are ignored.
    valid=true(n,1);if ~isfield(h,name),return;end
    value=h.(name);
    assert(numel(value)==n && all(ismember(value(:),[0,1])), ...
        'VehicleLocalization:InvalidMotionValidity','Motion validity must contain one Boolean per frame.');
    valid=logical(value(:));
end

function s=gnssPositionSource(data,t)
    n=numel(t);s=struct('valid',false(n,1),'values',nan(n,2),'availableAt',t);
    if ~isfield(data,'gnss'),return;end
    a=data.gnss;
    assert(isfield(a,'delay') && isequal(a.delay,0),'VehicleLocalization:FullZeroDelayRequired','Declare zero perception delay.');
    assert(isequal(a.time(:),t),'VehicleLocalization:SynchronousInputsRequired','Source clocks must equal the localization clock.');
    assert(isequal(size(a.position),[n,2]) && numel(a.valid)==n && all(ismember(a.valid,[0,1])), ...
            'VehicleLocalization:InvalidFullSource','Invalid position packet dimensions.');
    s.valid=logical(a.valid(:));s.values=a.position;
    if isfield(a,'sourceRightTime')
        right=a.sourceRightTime(:);
        assert(numel(right)==n && all(isfinite(right(s.valid))) && all(right(s.valid)>=t(s.valid)), ...
            'VehicleLocalization:InvalidFullSource','Valid GNSS interpolation endpoints must cover the frame.');
        s.availableAt(s.valid)=right(s.valid);
    end
    assert(isreal(s.values) && all(isfinite(s.values(s.valid,:)),'all'), ...
            'VehicleLocalization:InvalidFullSource','Valid position packets must be finite.');
end

function x=wrap(x)
    x=atan2(sin(x),cos(x));
end
