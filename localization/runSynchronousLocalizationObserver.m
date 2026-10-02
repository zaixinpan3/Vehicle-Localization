function estimate=runSynchronousLocalizationObserver(data,cfg,lateral)
% runSynchronousLocalizationObserver Discretize the continuous ISS-designed observer.
% Inputs and real measurement reconstructions must share the same frame clock.
% No pose anchor is propagated, no inter-frame pose is generated, and absent
% measurements withdraw their own correction. An implicit baseline step is
% followed by an implicit Lyapunov-matched LiDAR step. Only the locally affine
% LiDAR jump is certified as nonexpansive; the full sampled system is not.
% Optional data.lidarMatcher(k,seed,positionAid) runs before the current update.
% It replaces data.lidar, receives the previous fused-state prediction and
% current point-corrected GNSS, and returns a registration result. Candidate
% selection may depend on GNSS; its information must exclude GNSS curvature.
% Wheel and lateral-observer velocities are used without LiDAR-derived bias.
% GNSS corrects position only; the full LiDAR information corrects pose jointly.
% Matcher events and recorded data.lidar.residualModels supply accepted frozen
% geometry for predicted-pose residual evaluation. Old pose-only records use
% the local information approximation with its initialization dependence.
% Optional data.lidar.evaluateResidual(k,predictedPose) supplies residual-level
% geometry at the post-baseline pose on each valid frame, overriding that model.
    h=data.highRate;t=h.time(:);n=numel(t);
    assert(n>=2 && all(isfinite(t)) && all(diff(t)>0), ...
        'VehicleLocalization:InvalidSyncClock','Require an increasing frame clock.');
    assert(isfield(lateral,'time') && isequal(lateral.time(:),t), ...
        'VehicleLocalization:AlignedLateralRequired','Supply lateral estimates on the frame clock.');
    fields=["longitudinalSpeed","longitudinalAcceleration","lateralAcceleration","yawRate"];
    for name=fields
        assert(numel(h.(name))==n && all(isfinite(h.(name))), ...
            'VehicleLocalization:InvalidFullInput','Invalid aligned motion.');
    end
    for name=["lateralVelocity","sideSlipAngleRate"]
        assert(numel(lateral.(name))==n && all(isfinite(lateral.(name))), ...
            'VehicleLocalization:InvalidFullInput','Invalid aligned lateral output.');
    end
    G=source(data,'gnss',t,2);L=source(data,'lidar',t,3);
    onlineMatching=isfield(data,'lidarMatcher');
    if onlineMatching
        assert(isa(data.lidarMatcher,'function_handle') && ~isfield(data,'lidar'), ...
            'VehicleLocalization:AmbiguousLidarInput','Supply a matcher or recorded LiDAR measurements, not both.');
    end
    matchingResults=cell(0,1);matchingSeeds=zeros(0,3);
    if onlineMatching,matchingResults=cell(n,1);matchingSeeds=zeros(n,3);end
    alignment=struct('bodyOffset',[0;0],'bodyCovariance',zeros(2),'headingStdRad',0);
    if isfield(cfg.gnss,'outputPoint'),alignment=cfg.gnss.outputPoint;end
    continuous=designFullObserverGains(cfg);
    lidarInjection=continuous.lidarMatched;
    poseJacobian=zeros(3,7);poseJacobian(1,1)=1;poseJacobian(2,4)=1;poseJacobian(3,7)=1;
    design=struct('kind',"continuous-iss-implicit",'continuousDesign',continuous, ...
        'lidarMatched',lidarInjection,'lidarJumpCertificate',"Implicit locally affine fixed-metric contraction", ...
        'designRoute',continuous.designRoute, ...
        'sampledSystemCertified',false,'gainsRetuned',isfield(cfg.gnss,'positionGainDesign'));
    x=cfg.initialState;
    assert(numel(x)==7 && all(isfinite(x)),'VehicleLocalization:FullInitializationRequired', ...
        'Supply an explicit common initial state.');x=x(:);
    z=zeros(n,7);mode=zeros(n,1);headingMode=zeros(n,1);correction=zeros(n,4);
    yawCorrection=zeros(n,1);gnssPosition=nan(n,2);gnssInformation=nan(2,2,n);
    weights=nan(n,2);maximumRate=0;
    lidarAudits=cell(n,1);poseWeights=zeros(3,3,n);baselineStates=zeros(n,7);
    informationBounds=nan(n,3);informationQualified=false(n,1);
    for k=1:n
        dt=0;if k>1,dt=t(k)-t(k-1);end
        speed=h.longitudinalSpeed(k);vy=lateral.lateralVelocity(k);
        rate=h.yawRate(k);
        if k>1,rate=(h.yawRate(k-1)+rate)/2;end
        predYaw=x(7)+rate*dt;
        if onlineMatching
            midYaw=x(7)+rate*dt/2;
            Rmid=[cos(midYaw),-sin(midYaw);sin(midYaw),cos(midYaw)];
            seed=[(x([1,4])+dt*Rmid*[speed;vy]).',predYaw];
            aid=struct('valid',G.valid(k),'timestamp',t(k));
            if aid.valid
                [aid.position,I]=correctGnssOutputPoint(G.values(k,:),G.information(:,:,k),predYaw,alignment);
                aid.covariance=I\eye(2);
            end
            matched=data.lidarMatcher(k,seed,aid);
            matchingResults{k}=matched;matchingSeeds(k,:)=seed;
            event=registrationSupport.registrationPoseMeasurement(matched,t(k));
            L.valid(k)=~isempty(event);
            if L.valid(k)
                L.values(k,:)=event.pose;L.information(:,:,k)=event.information;
                if isfield(event,'lidarResidualModel'),L.residualModels{k}=event.lidarResidualModel;end
                if isfield(matched,'frameReliability'),L.frameReliability(k)=matched.frameReliability;end
                if isfield(matched,'directionReliability'),L.directionReliability(k,:)=matched.directionReliability(:).';end
            end
        end
        q=h.yawRate(k)+lateral.sideSlipAngleRate(k);maximumRate=max(maximumRate,abs(q));
        yaw=predYaw;
        Wg=zeros(2);
        if G.valid(k)
            [gnssPosition(k,:),gnssInformation(:,:,k)]=correctGnssOutputPoint(G.values(k,:),G.information(:,:,k),yaw,alignment);
            I=gnssInformation(:,:,k);Wg=I/(I+cfg.gnss.gainInformationScale*eye(2));Wg=(Wg+Wg.')/2;
        end
        K=cfg.gnss.positionGain*Wg;
        R=[cos(yaw),-sin(yaw);sin(yaw),cos(yaw)];J=[0,-1;1,0];
        velocity=R*[speed;vy];acceleration=R*[h.longitudinalAcceleration(k);h.lateralAcceleration(k)];
        A=[(1+dt*cfg.gains(2))*eye(2),-dt*eye(2); ...
            -dt*q^2*eye(2),(1+dt*cfg.gains(3))*eye(2)-2*dt*q*J];
        va=A\[x([2,5])+dt*cfg.gains(2)*velocity;x([3,6])+dt*cfg.gains(3)*acceleration];
        b=zeros(2,1);
        if G.valid(k),b=b+cfg.gnss.positionGain*Wg*gnssPosition(k,:).';end
        p=(eye(2)+dt*K)\(x([1,4])+dt*va(1:2)+dt*b);
        x=[p(1);va(1);va(3);p(2);va(2);va(4);yaw];
        baselineStates(k,:)=x.';
        if L.valid(k)
            measurement=struct('pose',L.values(k,:).','information',L.information(:,:,k));
            if isfield(L,'evaluateResidual')
                measurement=L.evaluateResidual(k,x([1,4,7]));
                assert(isfield(measurement,'residual'),'VehicleLocalization:InvalidLidarResidual', ...
                    'evaluateResidual must return a predicted-pose residual channel.');
            elseif ~isempty(L.residualModels{k})
                measurement=evaluateLidarRegistrationResidual(L.residualModels{k},x([1,4,7]));
            end
            if ~isfield(measurement,'frameReliability'),measurement.frameReliability=1;end
            if ~isfield(measurement,'directionReliability'),measurement.directionReliability=ones(3,1);end
            measurement.frameReliability=measurement.frameReliability*L.frameReliability(k);
            measurement.directionReliability=measurement.directionReliability(:).*L.directionReliability(k,:).';
            [delta,audit]=computeLidarMatchedCorrection(measurement,x([1,4,7]),poseJacobian, ...
                lidarInjection,cfg,StepSize=dt,Discretization="implicit");
            assert(audit.jumpNonexpansive,'VehicleLocalization:LidarJumpCertificateFailed', ...
                'The numerical LiDAR jump failed its fixed-metric energy check.');
            x=x+delta;lidarAudits{k}=audit;poseWeights(:,:,k)=audit.filter.S;
            S=audit.filter.S;
            informationBounds(k,:)=[min(eig(S(1:2,1:2))),S(3,3),norm(S(1:2,3))];
            informationQualified(k)=informationBounds(k,1)>=cfg.iss.minimumPositionStrength ...
                && informationBounds(k,2)>=cfg.iss.minimumHeadingStrength ...
                && informationBounds(k,3)<=cfg.iss.maximumPositionHeadingCoupling;
            rateCorrection=audit.continuousCorrection;
            if dt>0,rateCorrection=delta/dt;end
            correction(k,3:4)=rateCorrection([1,4]).';yawCorrection(k)=rateCorrection(7);
            weights(k,2)=min(audit.filter.strengths);
            if audit.filter.S(3,3)>0,headingMode(k)=2;end
        end
        assert(all(isfinite(x)),'VehicleLocalization:NonfiniteObserver','Synchronous update became nonfinite.');
        if G.valid(k),correction(k,1:2)=(cfg.gnss.positionGain*Wg*(gnssPosition(k,:).'-p)).';weights(k,1)=min(eig(Wg));end
        z(k,:)=x.';mode(k)=double(G.valid(k))+2*double(L.valid(k));
    end
    estimate=struct('time',t,'z',z,'onlineZ',z,'pose',[z(:,[1,4]),wrap(z(:,7))], ...
        'position',z(:,[1,4]),'velocity',z(:,[2,5]),'acceleration',z(:,[3,6]), ...
        'heading',wrap(z(:,7)),'headingUnwrapped',z(:,7),'lateral',lateral,'observer',design);
    estimate.diagnostics=struct('mode',mode,'headingMode',headingMode,'positionCorrection',correction, ...
        'gnssPositionAtObserverPoint',gnssPosition,'gnssInformationAtObserverPoint',gnssInformation, ...
        'yawCorrection',yawCorrection, ...
        'lidarMatched',{lidarAudits},'poseWeight',poseWeights,'baselineState',baselineStates, ...
        'baselineFullStateCertified',false, ...
        'continuousLmiVerified',continuous.fullContinuousStateCertified, ...
        'continuousInformationBounds',informationBounds, ...
        'continuousInformationQualified',informationQualified, ...
        'observedMotionEnvelopeSatisfied', ...
            all(hypot(h.longitudinalSpeed(:),lateral.lateralVelocity(:))<=cfg.iss.maximumSpeed) ...
            && all(hypot(h.longitudinalAcceleration(:),h.lateralAcceleration(:))<=cfg.iss.maximumAcceleration) ...
            && maximumRate<=cfg.maximumTrackAngleRate, ...
        'minimumWeights',weights, ...
        'maximumTrackAngleRate',maximumRate, ...
        'rateEnvelopeSatisfied',maximumRate<=cfg.maximumTrackAngleRate,'stateResets',0, ...
        'packetCounts',struct('gnssValid',nnz(G.valid),'gnssInvalid',nnz(~G.valid), ...
        'lidarValid',nnz(L.valid),'lidarInvalid',nnz(~L.valid)), ...
        'localizationUpdates',n-1,'virtualPoseUpdates',0,'integrationSubsteps',0, ...
        'nominalOutputRateHz',1/median(diff(t)),'referenceUsed',false, ...
        'allTheoremHypothesesVerified',false,'sampledSystemCertified',false, ...
        'scope',"Discretization of the continuous LMI-designed observer with unchanged gains. Recorded information conditions are audited; measured motion is only a proxy for true maneuver bounds. Continuous ISS is conditional and is not an automatic numerical or arbitrary-outage guarantee. Alignment remains offline.");
    if onlineMatching
        estimate.matchingResults=matchingResults;estimate.matchingSeeds=matchingSeeds;
        estimate.diagnostics.matchingFeedback=true;
        estimate.diagnostics.matchingSource="Per-frame callback with fused seed and optional current position aid";
    end
end

function s=source(data,name,t,width)
    n=numel(t);s=struct('valid',false(n,1),'values',nan(n,width),'information',nan(width,width,n), ...
        'frameReliability',ones(n,1),'directionReliability',ones(n,3));
    s.residualModels=cell(n,1);
    if ~isfield(data,name),return;end
    a=data.(name);field='pose';if width==2,field='position';end
    assert(isfield(a,'delay') && isequal(a.delay,0),'VehicleLocalization:FullZeroDelayRequired','Declare zero perception delay.');
    assert(isequal(a.time(:),t),'VehicleLocalization:SynchronousInputsRequired','Source clocks must equal the localization clock.');
    assert(isequal(size(a.(field)),[n,width]) && numel(a.valid)==n && ...
        all(ismember(a.valid,[0,1])) && size(a.information,1)==width && size(a.information,2)==width && size(a.information,3)==n, ...
        'VehicleLocalization:InvalidFullSource','Invalid synchronized source dimensions.');
    s.valid=logical(a.valid(:));s.values=a.(field);s.information=a.information;
    if width==3
        if isfield(a,'residualModels')
            assert(iscell(a.residualModels) && numel(a.residualModels)==n, ...
                'VehicleLocalization:InvalidLidarResidualModel','Align residual models with source frames.');
            s.residualModels=a.residualModels(:);
        end
        for name=["frameReliability","directionReliability"]
            if isfield(a,name)
                assert(isequal(size(a.(name)),size(s.(name))) && isreal(a.(name)) && ...
                    all(isfinite(a.(name)),'all') && all(a.(name)>=0 & a.(name)<=1,'all'), ...
                    'VehicleLocalization:InvalidLidarReliability','Invalid aligned reliability array.');
                s.(name)=a.(name);
            end
        end
        if isfield(a,'evaluateResidual')
            assert(isa(a.evaluateResidual,'function_handle'),'VehicleLocalization:InvalidLidarResidual', ...
                'evaluateResidual must be a function handle.');
            s.evaluateResidual=a.evaluateResidual;
        end
    end
    for k=find(s.valid).'
        I=s.information(:,:,k);
        assert(isreal(s.values(k,:)) && all(isfinite(s.values(k,:))) && isreal(I) && all(isfinite(I),'all') && ...
            norm(I-I.','fro')<=1e-10*max(1,norm(I,'fro')), ...
            'VehicleLocalization:InvalidFullSource','Valid frames require finite symmetric information.');
        smallest=min(eig((I+I.')/2));validSpectrum=smallest>0;
        if width==3,validSpectrum=smallest>=-1e-10*max(1,norm(I,2));end
        assert(validSpectrum, ...
            'VehicleLocalization:InvalidFullSource','Valid frames require finite symmetric information: PSD LiDAR, PD GNSS.');
    end
end

function x=wrap(x)
    x=atan2(sin(x),cos(x));
end
