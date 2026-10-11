function summary=judgeReceiverContradictions()
% judgeReceiverContradictions Vehicle sensors decide which receiver quantity is trusted.
% Second step of the reference build (see scripts/buildReceiverReference.py).
% Rule: where the receiver contradicts itself, vehicle sensors may decide which
% receiver quantity is trusted; their values never enter the reference. This
% function writes verdicts only.
%
% Vehicle sensors: four-wheel speed, the vehicle IMU (separate hardware from
% the NovAtel IMU), steering wheel angle and the lateral observer built on
% them. Their constant offsets, steering zero and output point were fitted on
% other drives; that calibrates the arbiter only.
%
% Alignment. Each receiver track is compared with the dead-reckoned vehicle
%    track over 4 s windows (2 s either side of a mapping frame) after the
%    best planar similarity transform (rotation, translation and scale). The
%    rotation and translation remove the vehicle track's unknown heading and
%    position, the scale removes the wheel-speed scale error, which otherwise
%    costs about 3 cm of residual per window and hides the contradictions.
%    Window length and alignment were chosen on this drive by the residual on
%    windows outside every candidate (about 1 cm), not by the verdicts.
% 1. Velocity integral model. The builder models the velocity of the receiver's
%    position point as the INSPVA velocity plus a fitted rigid relation (lever
%    arm and constants). Over windows in turns (|yaw rate| > 3 deg/s) outside
%    every candidate, the raw and the modeled velocity integral are each
%    aligned to the vehicle track; the integral with the smaller median
%    residual is used everywhere. Decided once, before the segment verdicts.
% 2. Position against velocity, per candidate. In the windows centred on the
%    mapping frames inside the candidate, the receiver position track and the
%    chosen velocity integral are each aligned to the vehicle track. Every
%    contradiction is resolved (user decisions, 2026-10-11). A verdict is
%    clear when one quantity's residual is smaller by more than 2 cm in more
%    than half of the windows; that quantity is trusted. Otherwise the
%    verdict is weak and the receiver position is trusted: without evidence
%    the reference is not moved, and the weak candidates of this drive lie
%    mostly in its RTK-fixed part, where the position is the receiver's best
%    quantity. A candidate without a window is weak. A slow uniform slew
%    looks like a heading change to this arbiter, so its verdict is weak.
% 3. Azimuth against course on straight driving. Per straight run the apparent
%    sideslip, course minus azimuth, is rejected as vehicle motion when the
%    road-wheel angle explains less than half of it and the lateral specific
%    force is below 0.5 m/s^2 (a vehicle moves at an angle to its heading only
%    with its wheels pointed that way or with a lateral tyre force of
%    kilonewtons per degree).
% 4. Gyro check of the heading through turns (report only).
    HALF_WINDOW=2.0;MINIMUM_SAMPLES=150;MARGIN=0.02;
    root=setupVehicleLocalization();out=fullfile(root,'output','receiver_reference');
    samples=readtable(fullfile(out,'consistency_samples.csv'));
    candidates=readtable(fullfile(out,'contradiction_candidates.csv'));
    poses=readtable(fullfile(root,'data','raw','Missisipi','gnss','raw_data_2024-06-07-12-09-31_0_front_lidar_synchronized_pose_1_1170.csv'));
    inspva=readtable(fullfile(root,'data','raw','Missisipi','gnss','raw_data_2024-06-07-12-09-31_0_inspva.csv'));
    assert(height(inspva)==height(samples),'Consistency table and INSPVA log differ in length.');
    [prepared,~,~]=prepareMncavObserverReplay("output/mncav_wheel_only_20260916/sensors","",table(),0,IncludeOdom=false);
    h=prepared.highRate;
    lateral=runLateralVelocityObserver(h,designLateralObserverGains(lateralObserverConfig()),lateralObserverConfig("mncav"));
    interface=jsondecode(fileread(fullfile(root,'config','mncavReplayInterface.json')));vehicle=mncavVehicleConfig();
    % Vehicle dead reckoning at 100 Hz, on the receiver clock.
    t=h.time;u=h.longitudinalSpeed;r=h.yawRate;w=lateral.lateralVelocity;
    psi=[0;cumsum(diff(t).*(r(1:end-1)+r(2:end))/2)];
    world=[u.*cos(psi)-w.*sin(psi),u.*sin(psi)+w.*cos(psi)];
    W=[0 0;cumsum(diff(t).*(world(1:end-1,:)+world(2:end,:))/2,1)];
    % Receiver quantities on the native grid inside the vehicle-data span.
    tn=samples.time;keep=tn>=t(1) & tn<=t(end);tn=tn(keep);
    P=[samples.x(keep),samples.y(keep)];
    vr=[samples.vxRaw(keep),samples.vyRaw(keep)];vm=[samples.vxModel(keep),samples.vyModel(keep)];
    Vr=P(1,:)+[0 0;cumsum(diff(tn).*(vr(1:end-1,:)+vr(2:end,:))/2,1)];
    Vm=P(1,:)+[0 0;cumsum(diff(tn).*(vm(1:end-1,:)+vm(2:end,:))/2,1)];
    Wn=interp1(t,W,tn);yawRate=samples.yawRateRadps(keep);
    writetable(table(t,W(:,1),W(:,2),psi,u,w,r,VariableNames={'time','x','y','psi','longitudinalSpeedMps','lateralVelocityMps','yawRateRadps'}), ...
        fullfile(out,'vehicle_track.csv'));
    % Windows of 2 s centred on mapping frames.
    frameTime=poses.receiver_time_sec;n=numel(frameTime);
    [eP,eVr,eVm,span,turn]=deal(nan(n,1));inside=zeros(n,1);
    for s=1:height(candidates)
        inside(frameTime>=candidates.beginSeconds(s) & frameTime<=candidates.endSeconds(s))=candidates.candidate(s);
    end
    for k=1:n
        ix=find(abs(tn-frameTime(k))<=HALF_WINDOW+1e-9);
        if frameTime(k)-HALF_WINDOW<tn(1) || frameTime(k)+HALF_WINDOW>tn(end) || numel(ix)<MINIMUM_SAMPLES,continue;end
        eP(k)=similarityResidual(P(ix,:),Wn(ix,:));eVr(k)=similarityResidual(Vr(ix,:),Wn(ix,:));eVm(k)=similarityResidual(Vm(ix,:),Wn(ix,:));
        span(k)=norm(P(ix(end),:)-P(ix(1),:));turn(k)=rad2deg(median(abs(yawRate(ix))));
    end
    valid=isfinite(eP) & span>=2;
    % 1. Velocity integral model, decided on turning windows outside candidates.
    choiceWindows=valid & inside==0 & turn>3;
    medians=[median(eVr(choiceWindows)),median(eVm(choiceWindows))];
    chosen="raw";if medians(2)<medians(1),chosen="modeled";end
    eV=eVr;if chosen=="modeled",eV=eVm;end
    windows=table(poses.frame_index,frameTime,inside,turn,span,eP,eVr,eVm,eP-eV, ...
        VariableNames={'frame','time','candidate','medianAbsYawRateDegps','windowSpanM','positionResidualM', ...
        'rawVelocityResidualM','modeledVelocityResidualM','positionMinusChosenVelocityResidualM'});
    writetable(windows,fullfile(out,'position_velocity_windows.csv'));
    % 2. Verdict per candidate.
    rows=cell(0,15);
    for s=1:height(candidates)
        k=candidates.candidate(s);ix=find(valid & inside==k);
        velocityBetter=NaN;positionBetter=NaN;velocitySmaller=NaN;medianDifference=NaN;mP=NaN;mV=NaN;
        if ~isempty(ix)
            d=eP(ix)-eV(ix);velocityBetter=mean(d>MARGIN);positionBetter=mean(d<-MARGIN);mP=median(eP(ix));mV=median(eV(ix));
            velocitySmaller=mean(d>0);medianDifference=median(d);
        end
        verdict="position";confidence="weak";
        if velocityBetter>0.5,verdict="velocity";confidence="clear";
        elseif positionBetter>0.5,confidence="clear";end
        rows(end+1,:)={k,candidates.beginSeconds(s),candidates.endSeconds(s),candidates.firstFrame(s),candidates.lastFrame(s), ...
            numel(ix),mP,mV,velocitySmaller,medianDifference,velocityBetter,positionBetter,verdict,confidence, ...
            median(interp1(t,u,frameTime(ix)))}; %#ok<AGROW>
    end
    segments=cell2table(rows,VariableNames={'candidate','beginSeconds','endSeconds','firstFrame','lastFrame','windows', ...
        'medianPositionResidualM','medianVelocityResidualM','fractionVelocitySmaller','medianPositionMinusVelocityResidualM', ...
        'fractionVelocityBetterBy2cm','fractionPositionBetterBy2cm','verdict','confidence','medianWheelSpeedMps'});
    writetable(segments,fullfile(out,'position_velocity_segments.csv'));disp(segments);
    % 3. Straight driving and sideslip.
    ti=samples.time;speed=hypot(inspva.east_velocity_mps,inspva.north_velocity_mps);
    course=atan2d(inspva.east_velocity_mps,inspva.north_velocity_mps);offset=mod(inspva.azimuth_deg-course+180,360)-180;
    rate=gradient(unwrap(deg2rad(inspva.azimuth_deg)),ti);rate=rad2deg(conv(rate,ones(25,1)/25,'same'));
    straight=speed>=5 & abs(rate)<=1;
    delta=interp1(t,h.steeringAngle,ti);steeringWheel=delta*vehicle.steeringRatio+interface.steeringWheelOffsetRad;
    ay=interp1(t,h.lateralAcceleration,ti);slip=interp1(t,lateral.lateralVelocity./max(u,1),ti);
    edges=diff([false;straight;false]);starts=find(edges==1);stops=find(edges==-1)-1;rows=cell(0,13);
    for s=1:numel(starts)
        ix=(starts(s):stops(s)).';ix=ix(isfinite(delta(ix)));if isempty(ix),continue;end
        apparent=median(offset(ix));roadWheel=rad2deg(median(delta(ix)));lateralForce=median(ay(ix));
        rejected=abs(roadWheel)<0.5*abs(apparent) && abs(lateralForce)<0.5;
        direction=atan2d(median(inspva.north_velocity_mps(ix)),median(inspva.east_velocity_mps(ix)));
        rows(end+1,:)={ti(ix(1)),ti(ix(end)),numel(ix),direction,apparent,rad2deg(median(steeringWheel(ix))),roadWheel, ...
            lateralForce,median(inspva.roll_deg(ix)),rad2deg(median(slip(ix))),median(speed(ix)),rejected,ti(ix(end))-ti(ix(1))}; %#ok<AGROW>
    end
    runs=cell2table(rows,VariableNames={'beginSeconds','endSeconds','samples','travelDirectionEnuDeg','courseMinusAzimuthEnuDeg', ...
        'steeringWheelDeg','roadWheelDeg','lateralSpecificForceMps2','receiverRollDeg','lateralModelSideslipDeg','speedMps', ...
        'sideslipRejected','durationSeconds'});
    writetable(runs,fullfile(out,'straight_runs.csv'));
    % 4. Gyro check: scale and bias fitted on receiver course over consistent straights (report only).
    consistent=samples.candidate==0;
    usable=straight & consistent & isfinite(delta);
    for s=find(~runs.sideslipRejected).'
        usable(ti>=runs.beginSeconds(s) & ti<=runs.endSeconds(s))=false;
    end
    psiGyro=interp1(t,psi,ti);courseEnu=unwrap(deg2rad(90-course));azimuthEnu=unwrap(deg2rad(90-inspva.azimuth_deg));
    A=[ones(nnz(usable),1),psiGyro(usable),ti(usable)];p=A\(courseEnu(usable)-psiGyro(usable));
    residual=rad2deg(courseEnu(usable)-psiGyro(usable)-A*p);
    calibrated=p(1)+(1+p(2))*psiGyro+p(3)*ti;azimuthError=rad2deg(azimuthEnu-calibrated);
    azimuthError=azimuthError-360*round(median(azimuthError,'omitnan')/360);
    sample=(0:floor(ti(end))).';[~,at]=min(abs(ti-sample.'),[],1);at=at.';
    gyro=table(ti(at),azimuthError(at),offset(at),straight(at),rate(at), ...
        VariableNames={'time','azimuthMinusCalibratedGyroDeg','courseMinusAzimuthEnuDeg','straight','azimuthRateDegPerS'});
    writetable(gyro,fullfile(out,'gyro_heading_check.csv'));
    summary=struct('rule',"Vehicle sensors only select which receiver quantity is trusted; no vehicle-sensor value enters the reference", ...
        'velocityIntegral',struct('chosen',chosen,'medianResidualRawM',medians(1),'medianResidualModeledM',medians(2), ...
            'windows',nnz(choiceWindows),'decision',"turning windows (|yaw rate| > 3 deg/s) outside candidates; smaller median similarity-aligned residual against the vehicle track"), ...
        'alignment',struct('transform',"planar similarity (rotation, translation, scale)",'windowSeconds',2*HALF_WINDOW,'minimumSamples',MINIMUM_SAMPLES,'marginM',MARGIN), ...
        'allWindows',struct('medianPositionResidualM',median(eP(valid)),'medianRawVelocityResidualM',median(eVr(valid)), ...
            'medianModeledVelocityResidualM',median(eVm(valid)),'windows',nnz(valid)), ...
        'positionVelocity',struct('segments',segments,'window',sprintf("%g s centred on each mapping frame, native 50 Hz receiver samples",2*HALF_WINDOW), ...
            'decision',"every candidate resolved: clear when one quantity's residual is smaller by more than 2 cm in more than half of the candidate's windows, and that quantity is trusted; otherwise weak and the receiver position is trusted"), ...
        'straightDriving',struct('runs',height(runs),'runsWithSideslipRejected',nnz(runs.sideslipRejected), ...
            'decision',"apparent sideslip rejected when |road-wheel angle| < 0.5 |course - azimuth| and |lateral specific force| < 0.5 m/s^2"), ...
        'gyro',struct('scale',p(2),'biasDegPerS',rad2deg(p(3)),'residualRmsDeg',rms(residual),'residualMaximumDeg',max(abs(residual)), ...
            'samples',nnz(usable),'use',"report only"));
    fid=fopen(fullfile(out,'judge_summary.json'),'w');fprintf(fid,'%s\n',jsonencode(summary,PrettyPrint=true));fclose(fid);
    fprintf('velocity integral: %s (median residual raw %.4f m, modeled %.4f m over %d turning windows)\n', ...
        chosen,medians(1),medians(2),nnz(choiceWindows));
    fprintf('verdicts: %d velocity, %d position of %d candidates (%d clear, %d weak)\n',nnz(segments.verdict=="velocity"), ...
        nnz(segments.verdict=="position"),height(segments),nnz(segments.confidence=="clear"),nnz(segments.confidence~="clear"));
    fprintf('straight runs %d, sideslip rejected on %d; gyro residual RMS %.3f deg\n',height(runs),nnz(runs.sideslipRejected),rms(residual));
end

function e=similarityResidual(X,Y)
% RMS residual after the best planar rotation, translation and scale of X onto Y.
    mx=mean(X,1);my=mean(Y,1);A=X-mx;B=Y-my;[U,S,Vt]=svd(A.'*B);D=diag([1,sign(det(U*Vt.'))]);R=U*D*Vt.';
    s=trace(S*D)/sum(A(:).^2);
    e=sqrt(mean(sum((s*A*R-B).^2,2)));
end
