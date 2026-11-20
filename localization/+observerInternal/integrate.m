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
    velocityValid=motionValidity(h,'bodyVelocityValid',n) ...
        & motionValidity(lateral,'velocityValid',n) & cfg.invariantObserver.velocityAvailable;
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
    G.valid=G.valid & cfg.invariantObserver.gnssPositionAvailable;
    heading.valid=heading.valid & cfg.invariantObserver.gnssHeadingAvailable;
    overlapDiagnostics=struct('evaluated',false(n,1),'available',false(n,1),'similarity',nan(n,1), ...
        'minimumInformation',nan(n,1),'seconds',nan(n,1),'gradient',nan(n,3), ...
        'channels',{cell(n,1)});
    alignment=struct('bodyOffset',[0;0],'bodyCovariance',zeros(2),'headingStdRad',0);
    if isfield(cfg.gnss,'outputPoint'),alignment=cfg.gnss.outputPoint;end
    [continuous,cfg]=localizationSupport.designFullObserverGains(cfg);
    lidarInjection=continuous.lidarMatched;
    stateCount=8;
    poseJacobian=zeros(3,stateCount);poseJacobian(1,1)=1;poseJacobian(2,4)=1;poseJacobian(3,7)=1;
    design=struct('kind',"nominal-fusion-implicit",'continuousDesign',continuous, ...
        'lidarMatched',lidarInjection,'lidarJumpCertificate',"Implicit locally affine fixed-metric contraction", ...
        'designRoute',continuous.designRoute, ...
        'sampledSystemCertified',false);
    x=cfg.initialState;
    assert(ismember(numel(x),[7,stateCount]) && all(isfinite(x)),'VehicleLocalization:FullInitializationRequired', ...
        'Supply an explicit common initial state.');x=x(:);
    if numel(x)==7,x(8)=0;end
    z=zeros(n,stateCount);mode=zeros(n,1);headingMode=zeros(n,1);correction=zeros(n,4);
    yawCorrection=zeros(n,1);gnssPosition=nan(n,2);
    headingInnovation=nan(n,1);
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
        predYaw=x(7)+rate*dt;
        q=localizationSupport.courseRate(x,cfg);
        maximumRate=max(maximumRate,abs(q));
        yaw=predYaw;
        if G.valid(k)
            gnssPosition(k,:)=localizationSupport.correctGnssOutputPoint(G.values(k,:),[],yaw,alignment);
        end
        x=localizationSupport.predict(x,dt,cfg);
        intrinsicStates(k,:)=x.';
        gnssRate=zeros(2,1);
        if G.valid(k) || heading.valid(k)
            ell=cfg.invariantObserver.headingLengthM;
            Cg=zeros(3,8);Cg(1,1)=1;Cg(2,4)=1;Cg(3,7)=ell;
            observation=Cg*x;
            if G.valid(k),observation(1:2)=gnssPosition(k,:).';end
            if heading.valid(k)
                headingInnovation(k)=atan2(sin(heading.heading(k)-x(7)),cos(heading.heading(k)-x(7)));
                observation(3)=ell*(x(7)+headingInnovation(k));
            end
            active=[G.valid(k),G.valid(k),heading.valid(k)];
            Kg=continuous.LG(:,active);Cg=Cg(active,:);observation=observation(active);
            x=(eye(8)+dt*Kg*Cg)\(x+dt*Kg*observation);
            gnssRate=Kg([1,4],:)*(observation-Cg*x);
        end
        packet=localizationSupport.measurements(speed,vy,h.longitudinalAcceleration(k), ...
            h.lateralAcceleration(k), ...
            velocityValid(k),accelerationValid(k),cfg);
        priorRate=x(8);
        [x,invariantAudits{k}]=localizationSupport.correct(x,packet,dt,continuous,cfg);
        % Complete the implicit psi-dot = estimated-rate equation using
        % the posterior STATE. No gyro sample appears in this prediction.
        x(7)=x(7)+dt*(x(8)-priorRate);
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
            if ~isfield(measurement,'frameReliability'),measurement.frameReliability=1;end
            if ~isfield(measurement,'directionReliability'),measurement.directionReliability=ones(3,1);end
            measurement.frameReliability=measurement.frameReliability*L.frameReliability(k);
            measurement.directionReliability=measurement.directionReliability(:).*L.directionReliability(k,:).';
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
            lidarPoseSupport.correctionApplied(k)=support.rank>0 && lidarInjection.kappa>0 && dt>0;
            rateCorrection=audit.continuousCorrection;
            if dt>0,rateCorrection=delta/dt;end
            correction(k,3:4)=rateCorrection([1,4]).';yawCorrection(k)=rateCorrection(7);
            lidarStrength(k)=min(audit.filter.strengths);
            if audit.filter.S(3,3)>0,headingMode(k)=2;end
        end
        assert(all(isfinite(x)),'VehicleLocalization:NonfiniteObserver','Synchronous update became nonfinite.');
        if G.valid(k),correction(k,1:2)=gnssRate.';end
        z(k,:)=x.';mode(k)=double(G.valid(k))+2*double(L.valid(k));
    end
    lidarPoseSupport.sourceValid=L.valid;
    estimate=struct('time',t,'z',z(:,1:7),'pose',[z(:,[1,4]),wrap(z(:,7))], ...
        'position',z(:,[1,4]),'velocity',z(:,[2,5]),'acceleration',z(:,[3,6]), ...
        'heading',wrap(z(:,7)),'headingUnwrapped',z(:,7),'lateral',reportedLateral,'observer',design);
    measuredSpeed=hypot(h.longitudinalSpeed(:),lateral.lateralVelocity(:));
    speedInterval=[cfg.invariantObserver.fusion.minimumCourseSpeed,cfg.iss.maximumSpeed];
    speedInDomain=velocityValid & measuredSpeed>=speedInterval(1) & measuredSpeed<=speedInterval(2);
    estimate.diagnostics=struct('mode',mode,'headingMode',headingMode,'positionCorrection',correction, ...
        'gnssPositionAtObserverPoint',gnssPosition, ...
        'yawCorrection',yawCorrection, ...
        'lidarMatched',{lidarAudits},'poseWeight',poseWeights,'preLidarState',preLidarStates, ...
        'lidarInformationBounds',informationBounds, ...
        'independentHeadingValid',heading.valid, ...
        'independentHeadingInnovation',headingInnovation, ...
        'observedMotionEnvelopeSatisfied', ...
            all(velocityValid) && all(accelerationValid) ...
            && all(speedInDomain) ...
            && all(hypot(h.longitudinalAcceleration(:),h.lateralAcceleration(:))<=cfg.iss.maximumAcceleration) ...
            && maximumRate<=cfg.maximumTrackAngleRate, ...
        'issSpeedInterval',speedInterval,'observedSpeedWithinIssDomain',speedInDomain, ...
        'zeroSpeedCertified',false,'minimumLidarStrength',lidarStrength,'gnssPositionValid',G.valid, ...
        'maximumTrackAngleRate',maximumRate, ...
        'rateEnvelopeSatisfied',maximumRate<=cfg.maximumTrackAngleRate, ...
        'packetCounts',struct('gnssValid',nnz(G.valid),'gnssInvalid',nnz(~G.valid), ...
        'lidarValid',nnz(L.valid),'lidarInvalid',nnz(~L.valid), ...
        'bodyVelocityValid',nnz(velocityValid),'bodyAccelerationValid',nnz(accelerationValid)), ...
        'localizationUpdates',n-1, ...
        'nominalOutputRateHz',1/median(diff(t)),'referenceUsed',false, ...
        'lidarChannel',L.name, ...
        'lidarOverlap',overlapDiagnostics, ...
        'lidarPoseSupport',lidarPoseSupport, ...
        'motionOutputValidity',struct('bodyVelocity',velocityValid & cfg.invariantObserver.velocityAvailable, ...
            'bodyAcceleration',velocityValid & accelerationValid ...
                & cfg.invariantObserver.velocityAvailable & cfg.invariantObserver.accelerationAvailable), ...
        'allTheoremHypothesesVerified',false,'sampledSystemCertified',false, ...
        'scope',"Discretization of the continuous LMI-designed observer with unchanged gains. Recorded information conditions are audited; measured motion is only a proxy for true maneuver bounds. The nominal fusion design assumes simultaneous sources and a compact invariant operating set; low-speed execution uses a smooth denominator extension and is outside this certificate. No automatic numerical or arbitrary-outage guarantee is asserted. Alignment remains offline.");
    estimate.internalState=z;estimate.yawRate=z(:,8);
    estimate.diagnostics.intrinsicPrediction=intrinsicStates;
    estimate.diagnostics.invariantOutputs=invariantAudits;
    estimate.diagnostics.invariantOutputNames=["speedSquared","velocityAccelerationDot", ...
        "velocityAccelerationCross","bodyLateralVelocity"];
    estimate.diagnostics.intrinsicPredictionUsesImu=false;
    estimate.diagnostics.yawRateFeedbackSource="matchedMotionAndPose";
    estimate.diagnostics.actualOverlapCertified=false;
    certificate=continuous.continuousCertificate.verification;
    estimate.diagnostics.operatingSet=certificate.operatingSet;
    estimate.diagnostics.nominalFusionLmiVerified=certificate.nominalFusionLmiVerified;
    estimate.diagnostics.allAuxiliaryOutputsAvailable=velocityValid & accelerationValid ...
        & cfg.invariantObserver.velocityAvailable & cfg.invariantObserver.accelerationAvailable;
    estimate.diagnostics.auxiliaryAvailabilityContractSatisfied= ...
        all(estimate.diagnostics.allAuxiliaryOutputsAvailable);
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
    n=numel(t);s=struct('valid',false(n,1),'values',nan(n,2));
    if ~isfield(data,'gnss'),return;end
    a=data.gnss;
    assert(isfield(a,'delay') && isequal(a.delay,0),'VehicleLocalization:FullZeroDelayRequired','Declare zero perception delay.');
    assert(isequal(a.time(:),t),'VehicleLocalization:SynchronousInputsRequired','Source clocks must equal the localization clock.');
    assert(isequal(size(a.position),[n,2]) && numel(a.valid)==n && all(ismember(a.valid,[0,1])), ...
            'VehicleLocalization:InvalidFullSource','Invalid position packet dimensions.');
    s.valid=logical(a.valid(:));s.values=a.position;
    assert(isreal(s.values) && all(isfinite(s.values(s.valid,:)),'all'), ...
            'VehicleLocalization:InvalidFullSource','Valid position packets must be finite.');
end

function x=wrap(x)
    x=atan2(sin(x),cos(x));
end
