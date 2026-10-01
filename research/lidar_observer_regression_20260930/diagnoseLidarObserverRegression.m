function diagnoseLidarObserverRegression()
% diagnoseLidarObserverRegression Attribute the saved LiDAR-only position error.
% Frozen scenario-specific registrations isolate the downstream observer.
% Reference positions enter scoring and explicitly marked oracle controls only.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    out='output/lidar_observer_regression_20260930';if ~isfolder(out),mkdir(out);end
    cache=fullfile(out,'inputs.mat');
    if ~isfile(cache)
        fprintf('Loading the full experiment once.\n');
        b=load('output/support_full_localization_20260930/observer/experiment.mat', ...
            'data','lateral','cfg','runs','reference');
        data=rmfield(b.data,'gnss');saved=b.runs{2}.estimate;
        for k=1:numel(saved.time)
            r=saved.matchingResults{k};data.lidar.pose(k,:)=r.poseXYTheta;
            data.lidar.valid(k)=r.accepted;data.lidar.information(:,:,k)=r.information;
        end
        saved=rmfield(saved,{'matchingResults','matchingSeeds'});
        cfg=b.cfg;lateral=b.lateral;reference=b.reference;
        save(cache,'data','saved','cfg','lateral','reference');clear b;
    else
        load(cache,'data','saved','cfg','lateral','reference');
    end
    fprintf('Replaying the exact saved LiDAR-only packets.\n');
    base=runSynchronousLocalizationObserver(data,cfg,lateral);
    replayDifference=max(abs(base.z-saved.z),[],'all');assert(replayDifference==0);
    t=base.time;n=numel(t);K=zeros(2,2,n);weightEigenvalues=zeros(n,2);
    currentPoseFraction=zeros(n,2);inputVelocity=zeros(n,2);rawVelocity=inputVelocity;
    for k=1:n
        if data.lidar.valid(k)
            I=data.lidar.information(:,:,k);W=I/(I+cfg.lidar.gainInformationScale*eye(3));
            K(:,:,k)=cfg.gains(1)*(W(1:2,1:2)+W(1:2,1:2).')/2;
        end
        weightEigenvalues(k,:)=sort(eig(K(:,:,k))).';
        dt=0;if k>1,dt=t(k)-t(k-1);end
        currentPoseFraction(k,:)=sort(eig((eye(2)+dt*K(:,:,k))\(dt*K(:,:,k)))).';
        yaw=base.pose(k,3);R=[cos(yaw),-sin(yaw);sin(yaw),cos(yaw)];
        raw=[data.highRate.longitudinalSpeed(k);lateral.lateralVelocity(k)];
        bias=[base.diagnostics.lidarLongitudinalVelocityBias(k);base.diagnostics.lidarVelocityBias(k)];
        inputVelocity(k,:)=(R*(raw+bias)).';rawVelocity(k,:)=(R*raw).';
    end
    % Exact additive recursion in error coordinates avoids UTM cancellation.
    initial=zeros(n,2);measurement=initial;motion=initial;filter=initial;quadrature=initial;input=initial;
    initial(1,:)=base.position(1,:)-reference(1,1:2);
    for k=2:n
        dt=t(k)-t(k-1);P=(eye(2)+dt*K(:,:,k))\eye(2);dp=reference(k,1:2)-reference(k-1,1:2);
        innovation=zeros(2,1);
        if data.lidar.valid(k),innovation=dt*K(:,:,k)*(data.lidar.pose(k,1:2)-reference(k,1:2)).';end
        initial(k,:)=(P*initial(k-1,:).').';
        measurement(k,:)=(P*(measurement(k-1,:).'+innovation)).';
        motion(k,:)=(P*(motion(k-1,:)+dt*base.velocity(k,:)-dp).').';
        filter(k,:)=(P*(filter(k-1,:)+dt*(base.velocity(k,:)-inputVelocity(k,:))).').';
        quadrature(k,:)=(P*(quadrature(k-1,:)+dt/2*(inputVelocity(k,:)-inputVelocity(k-1,:))).').';
        input(k,:)=(P*(input(k-1,:)+dt/2*(inputVelocity(k,:)+inputVelocity(k-1,:))-dp).').';
    end
    closure=max(abs(initial+measurement+motion-(base.position-reference(:,1:2))),[],'all');
    motionClosure=max(abs(filter+quadrature+input-motion),[],'all');
    assert(closure<1e-7 && motionClosure<1e-10);
    controls=cell(1,11);names=strings(11,1);types=strings(11,1);
    controls{1}=base;names(1)="baseline";types(1)="production replay";
    c=cfg;c.bias.enabled=false;
    controls{2}=runSynchronousLocalizationObserver(data,c,lateral);names(2)="disable_velocity_bias";types(2)="runtime ablation";
    scales=[1,.001];
    for j=1:2
        scale=scales(j);
        c=cfg;c.lidar.gainInformationScale=scale;
        controls{j+2}=runSynchronousLocalizationObserver(data,c,lateral);
        names(j+2)="information_scale_"+string(scale);types(j+2)="runtime sensitivity";
    end
    c=cfg;c.gains(1)=12;
    controls{5}=runSynchronousLocalizationObserver(data,c,lateral);names(5)="position_gain_12";types(5)="runtime sensitivity";
    nativeVelocity=readtable(fullfile(out,'native_grid_velocity.csv'));
    vref=interp1(nativeVelocity.time,[nativeVelocity.vx,nativeVelocity.vy],t);
    increments={ [zeros(1,2);diff(t).*base.velocity(2:end,:)], ...
        [zeros(1,2);diff(t).*(base.velocity(1:end-1,:)+base.velocity(2:end,:))/2], ...
        [zeros(1,2);diff(t).*inputVelocity(2:end,:)], ...
        [zeros(1,2);diff(t).*(inputVelocity(1:end-1,:)+inputVelocity(2:end,:))/2], ...
        [zeros(1,2);diff(reference(:,1:2))], ...
        [zeros(1,2);diff(t).*(vref(1:end-1,:)+vref(2:end,:))/2]};
    reconstructionNames=["reconstruct_baseline","trapezoid_state_velocity","direct_input_endpoint","direct_input_trapezoid", ...
        "oracle_reference_displacement","oracle_ins_reported_velocity"];
    for j=1:numel(increments)
        p=reconstruct(base.position(1,:),increments{j},K,data.lidar,t);
        r=base;r.position=p;r.pose(:,1:2)=p;
        controls{j+5}=r;names(j+5)=reconstructionNames(j);types(j+5)="frozen-state position control";
    end
    assert(max(abs(controls{6}.position-base.position),[],'all')<1e-7);
    rows=cell(0,12);after=t>=t(1)+2;paired=after & data.lidar.valid;
    for j=1:numel(controls)
        r=controls{j};e=vecnorm(r.position-reference(:,1:2),2,2);
        for population=["after_2_seconds","paired_full_matches"]
            ix=after;if population=="paired_full_matches",ix=paired;end
            selected=find(ix);[peak,at]=max(e(ix));at=selected(at);
            rows(end+1,:)={names(j),types(j),population,nnz(ix),rms(e(ix)),prctile(e(ix),95),peak,at, ...
                e(847),rms(rad2deg(wrap(r.pose(ix,3)-reference(ix,3)))),rms(vecnorm(r.velocity(ix,:)-inputVelocity(ix,:),2,2)),replayDifference}; %#ok<AGROW>
        end
    end
    results=cell2table(rows,VariableNames={'control','kind','population','frames','positionRmseM','p95M', ...
        'maximumM','maximumFrame','frame847M','yawRmseDeg','velocityStateVsBaselineInputRmseMps','baselineReplayDifference'});
    writetable(results,fullfile(dest,'controls.csv'));disp(results(results.population=="paired_full_matches",1:10));
    err=base.position-reference(:,1:2);
    trace=table((1:n).',t,data.lidar.valid,err(:,1),err(:,2),initial(:,1),initial(:,2),measurement(:,1),measurement(:,2), ...
        motion(:,1),motion(:,2),filter(:,1),filter(:,2),quadrature(:,1),quadrature(:,2),input(:,1),input(:,2), ...
        weightEigenvalues(:,1),weightEigenvalues(:,2),currentPoseFraction(:,1),currentPoseFraction(:,2), ...
        base.velocity(:,1)-inputVelocity(:,1),base.velocity(:,2)-inputVelocity(:,2), ...
        VariableNames={'frame','time','fullMatch','errorX','errorY','initialX','initialY','matchX','matchY','motionX','motionY', ...
        'filterX','filterY','quadratureX','quadratureY','inputX','inputY','positionGainMin','positionGainMax', ...
        'currentMatchFractionMin','currentMatchFractionMax','velocityFilterX','velocityFilterY'});
    writetable(trace,fullfile(dest,'decomposition.csv'));
    summary=struct('baselineReplayMaximumDifference',replayDifference,'errorDecompositionMaximumResidualM',closure, ...
        'motionDecompositionMaximumResidualM',motionClosure,'frame847',table2struct(trace(847,:)), ...
        'frame847InputBodyVelocity',[data.highRate.longitudinalSpeed(847),lateral.lateralVelocity(847)], ...
        'frame847BiasBodyVelocity',[base.diagnostics.lidarLongitudinalVelocityBias(847),base.diagnostics.lidarVelocityBias(847)], ...
        'frame847StateVelocity',base.velocity(847,:),'frame847InputVelocity',inputVelocity(847,:), ...
        'frame847AccelerationInput',[data.highRate.longitudinalAcceleration(847),data.highRate.lateralAcceleration(847)], ...
        'frame847StateAcceleration',base.acceleration(847,:), ...
        'frame847Information',data.lidar.information(:,:,847),'frame847PositionGain',K(:,:,847), ...
        'gainEigenvalueMedian',median(weightEigenvalues(paired,:)), ...
        'currentMatchFractionMedian',median(currentPoseFraction(paired,:)), ...
        'positionComponentRmseAfter2Seconds',struct('initial',rms(vecnorm(initial(after,:),2,2)), ...
        'measurement',rms(vecnorm(measurement(after,:),2,2)),'motion',rms(vecnorm(motion(after,:),2,2)), ...
        'velocityFilter',rms(vecnorm(filter(after,:),2,2)),'endpointQuadrature',rms(vecnorm(quadrature(after,:),2,2)), ...
        'trapezoidInput',rms(vecnorm(input(after,:),2,2))), ...
        'referenceUsage',"Scoring, additive error attribution, and explicitly named oracle only. All runtime sensitivity controls are reference-free.", ...
        'scope',"Frozen LiDAR-only registrations from the complete closed-loop run; candidate settings do not certify closed-loop or cross-drive improvement.");
    native=readtable('output/receiver_synchronized_inputs/native_reference.csv');
    ins=readtable('data/raw/Missisipi/gnss/raw_data_2024-06-07-12-09-31_0_inspva.csv');
    assert(height(native)==height(ins) && max(abs(diff(native.time)-diff(ins.gps_seconds)))<1e-8);
    assert(max(abs(native.time-nativeVelocity.time))<1e-8);
    referenceBody=zeros(n,2);headingDelta=zeros(n,2);longDelta=zeros(n,2);latDelta=zeros(n,2);
    bias=[base.diagnostics.lidarLongitudinalVelocityBias,base.diagnostics.lidarVelocityBias];
    body=[data.highRate.longitudinalSpeed,lateral.lateralVelocity]+bias;
    for k=1:n
        yaw=reference(k,3);R=[cos(yaw),-sin(yaw);sin(yaw),cos(yaw)];
        referenceBody(k,:)=(R.'*vref(k,:).').';
        headingDelta(k,:)=inputVelocity(k,:)-(R*body(k,:).').';
        longDelta(k,:)=(R*[body(k,1)-referenceBody(k,1);0]).';
        latDelta(k,:)=(R*[0;body(k,2)-referenceBody(k,2)]).';
    end
    assert(max(abs(headingDelta+longDelta+latDelta+vref-inputVelocity),[],'all')<1e-10);
    velocityComponents={headingDelta,longDelta,latDelta,vref};componentNames=["heading","longitudinal","lateral","reference_kinematics"];
    integrated=cell(1,4);parts=zeros(n,2);
    for j=1:4
        e=zeros(n,2);v=velocityComponents{j};
        for k=2:n
            dt=t(k)-t(k-1);delta=dt/2*(v(k,:)+v(k-1,:));
            if j==4,delta=delta-(reference(k,1:2)-reference(k-1,1:2));end
            e(k,:)=((eye(2)+dt*K(:,:,k))\(e(k-1,:)+delta).').';
        end
        integrated{j}=e;parts=parts+e;
        summary.inputErrorContributions.(componentNames(j))=struct('frame847',e(847,:), ...
            'after2SecondsRmseM',rms(vecnorm(e(after,:),2,2)));
    end
    assert(max(abs(parts-input),[],'all')<1e-10);
    detail=table((1:n).',t,data.highRate.longitudinalSpeed,lateral.lateralVelocity,bias(:,1),bias(:,2), ...
        referenceBody(:,1),referenceBody(:,2),data.highRate.yawRate,lateral.sideSlipAngleRate, ...
        rad2deg(wrap(base.pose(:,3)-reference(:,3))), ...
        integrated{1}(:,1),integrated{1}(:,2),integrated{2}(:,1),integrated{2}(:,2), ...
        integrated{3}(:,1),integrated{3}(:,2),integrated{4}(:,1),integrated{4}(:,2), ...
        VariableNames={'frame','time','wheelVx','lateralVy','biasVx','biasVy','insBodyVx','insBodyVy','yawRate','betaRate','yawErrorDeg', ...
        'headingX','headingY','longitudinalX','longitudinalY','lateralX','lateralY','referenceKinematicsX','referenceKinematicsY'});
    writetable(detail,fullfile(dest,'motion_inputs.csv'));
    summary.frame847Motion=table2struct(detail(847,:));
    summary.motionReferenceCaveat="INSPVA reported velocity is diagnostic only. Its trapezoid integral need not equal reported position increments; that difference is retained separately.";
    fid=fopen(fullfile(dest,'summary.json'),'w');fprintf(fid,'%s\n',jsonencode(summary,PrettyPrint=true));fclose(fid);
    save(fullfile(out,'diagnostic.mat'),'summary','trace','results','controls','names','inputVelocity','rawVelocity','K','reference','t');
    fprintf('Exact decomposition residual %.3g m; motion split %.3g m.\n',closure,motionClosure);
    disp(trace(847,:));
end

function p=reconstruct(p0,delta,K,L,t)
    p=zeros(numel(t),2);p(1,:)=p0;
    for k=2:numel(t)
        dt=t(k)-t(k-1);b=zeros(2,1);
        if L.valid(k),b=dt*K(:,:,k)*(L.pose(k,1:2)-p(k-1,:)).';end
        p(k,:)=p(k-1,:)+((eye(2)+dt*K(:,:,k))\(delta(k,:).'+b)).';
    end
end
function a=wrap(a),a=atan2(sin(a),cos(a));end
