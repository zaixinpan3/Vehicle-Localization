function diagnoseMaximumErrorFrame()
% diagnoseMaximumErrorFrame Attribute the largest fused "both" discrepancy.
% The saved GNSS and closed-loop LiDAR packets are frozen, so the position
% recursion of runSynchronousLocalizationObserver is additive in its inputs.
% Reference positions and INSPVA velocity enter scoring, attribution and
% explicitly named oracle controls only.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    out='output/max_error_frame_20261001';
    b=load(fullfile(out,'run','observer','experiment.mat'),'data','lateral','cfg','runs','reference');
    m=load(fullfile(out,'run','matching','report.mat'),'report');calls=m.report.calls;
    data=b.data;lateral=b.lateral;cfg=b.cfg;ref=b.reference;saved=b.runs{1}.estimate;clear b m;
    assert(isfield(data.lidar,'conditionedOnGnssSelection'));
    t=saved.time;n=numel(t);h=data.highRate;
    [distance,index]=min(abs(calls.timeSeconds-t.'),[],1);assert(max(distance)<1e-7);frame=calls.frame(index.');
    base=runSynchronousLocalizationObserver(data,cfg,lateral);
    replayDifference=max(abs(base.z-saved.z),[],'all');assert(replayDifference<1e-9);
    summaryFile=jsondecode(fileread(fullfile(dest,'maximum_summary.json')));
    at=find(frame==summaryFile.fusedMaximumFrame);after=t>=t(1)+2;
    % Frozen gains and measurement discrepancies of the position update.
    Kg=zeros(2,2,n);Kl=Kg;gnssError=zeros(n,2);lidarError=zeros(n,2);
    G=base.diagnostics.gnssPositionAtObserverPoint;
    for k=1:n
        if data.gnss.valid(k)
            I=base.diagnostics.gnssInformationAtObserverPoint(:,:,k);
            W=I/(I+cfg.gnss.gainInformationScale*eye(2));Kg(:,:,k)=cfg.gnss.positionGain*(W+W.')/2;
            gnssError(k,:)=G(k,:)-ref(k,1:2);
        end
        if data.lidar.valid(k)
            I=data.lidar.information(:,:,k);I=(I+I.')/2;W=I/(I+cfg.lidar.gainInformationScale*eye(3));W=(W+W.')/2;
            Kl(:,:,k)=cfg.gains(1)*W(1:2,1:2);lidarError(k,:)=data.lidar.pose(k,1:2)-ref(k,1:2);
        end
    end
    K=Kg+Kl;
    nativeVelocity=readtable('output/lidar_observer_regression_20260930/native_grid_velocity.csv');
    vref=interp1(nativeVelocity.time,[nativeVelocity.vx,nativeVelocity.vy],t);
    body=[h.longitudinalSpeed,lateral.lateralVelocity];
    inputVelocity=zeros(n,2);referenceBody=zeros(n,2);incrementBody=nan(n,2);
    headingDelta=zeros(n,2);longDelta=zeros(n,2);latDelta=zeros(n,2);
    for k=1:n
        R=rot(base.pose(k,3));Rr=rot(ref(k,3));
        inputVelocity(k,:)=(R*body(k,:).').';referenceBody(k,:)=(Rr.'*vref(k,:).').';
        if k>1,incrementBody(k,:)=(Rr.'*(ref(k,1:2)-ref(k-1,1:2)).').'/(t(k)-t(k-1));end
        headingDelta(k,:)=inputVelocity(k,:)-(Rr*body(k,:).').';
        longDelta(k,:)=(Rr*[body(k,1)-referenceBody(k,1);0]).';
        latDelta(k,:)=(Rr*[0;body(k,2)-referenceBody(k,2)]).';
    end
    assert(max(abs(headingDelta+longDelta+latDelta+vref-inputVelocity),[],'all')<1e-10);
    % Exact additive recursion in error coordinates.
    names=["initial","gnss","lidar","velocityFilter","endpointQuadrature","heading","longitudinal","lateral","referenceKinematics"];
    part=zeros(n,2,numel(names));part(1,:,1)=base.position(1,:)-ref(1,1:2);
    for k=2:n
        dt=t(k)-t(k-1);P=(eye(2)+dt*K(:,:,k))\eye(2);dp=ref(k,1:2)-ref(k-1,1:2);
        trapezoid=@(v)dt/2*(v(k,:)+v(k-1,:));
        drive={[0 0],(dt*Kg(:,:,k)*gnssError(k,:).').',(dt*Kl(:,:,k)*lidarError(k,:).').', ...
            dt*(base.velocity(k,:)-inputVelocity(k,:)),dt/2*(inputVelocity(k,:)-inputVelocity(k-1,:)), ...
            trapezoid(headingDelta),trapezoid(longDelta),trapezoid(latDelta),trapezoid(vref)-dp};
        for j=1:numel(names),part(k,:,j)=(P*(part(k-1,:,j)+drive{j}).').';end
    end
    err=base.position-ref(:,1:2);closure=max(abs(sum(part,3)-err),[],'all');assert(closure<1e-7);
    % Express every propagated contribution in reference vehicle axes.
    bodyPart=zeros(n,2,numel(names));
    for k=1:n,for j=1:numel(names),bodyPart(k,:,j)=(rot(ref(k,3)).'*part(k,:,j).').';end,end
    total=sum(bodyPart,3);
    rows=cell(numel(names)+1,5);
    for j=1:numel(names)
        rows(j,:)={names(j),bodyPart(at,1,j),bodyPart(at,2,j),rms(vecnorm(part(after,:,j),2,2)),mean(bodyPart(after,2,j))};
    end
    rows(end,:)={"total",total(at,1),total(at,2),rms(vecnorm(err(after,:),2,2)),mean(total(after,2))};
    budget=cell2table(rows,VariableNames={'contribution','longitudinalAtMaximumM','lateralAtMaximumM','routeRmseAfter2SecondsM','routeMeanLateralAfter2SecondsM'});
    writetable(budget,fullfile(dest,'error_budget.csv'));disp(budget);
    trace=array2table([frame,t,total,reshape(bodyPart,n,[])],VariableNames=["frame","time","totalLongitudinal","totalLateral", ...
        reshape([names+"Longitudinal";names+"Lateral"],1,[])]);
    writetable(trace,fullfile(dest,'decomposition.csv'));
    % Reference course versus reference yaw, and the supplied body motion.
    gainEigenvalues=zeros(n,2);for k=1:n,gainEigenvalues(k,:)=sort(eig(K(:,:,k))).';end
    lagRate=zeros(n,1);lateralGain=zeros(n,1);
    for k=1:n
        left=rot(ref(k,3))*[0;1];lateralGain(k)=left.'*K(:,:,k)*left;
        lagRate(k)=body(k,2)-incrementBody(k,2);
    end
    slip=rad2deg(atan2(referenceBody(:,2),referenceBody(:,1)));slipIncrement=rad2deg(atan2(incrementBody(:,2),incrementBody(:,1)));
    estimateSlip=rad2deg(atan2(body(:,2),body(:,1)));
    drift=[0;cumsum(diff(t).*(incrementBody(2:end,2)-(referenceBody(1:end-1,2)+referenceBody(2:end,2))/2))];
    matchBody=nan(n,2);gnssBody=nan(n,2);
    for k=1:n
        if data.lidar.valid(k),matchBody(k,:)=(rot(ref(k,3)).'*lidarError(k,:).').';end
        if data.gnss.valid(k),gnssBody(k,:)=(rot(ref(k,3)).'*gnssError(k,:).').';end
    end
    detail=table(frame,t,h.longitudinalSpeed,rad2deg(h.yawRate),rad2deg(h.steeringAngle),lateral.lateralVelocity, ...
        referenceBody(:,2),incrementBody(:,2),estimateSlip,slip,slipIncrement,drift,total(:,2),matchBody(:,2),gnssBody(:,2), ...
        rad2deg(wrap(base.pose(:,3)-ref(:,3))),lateralGain,gainEigenvalues(:,1),gainEigenvalues(:,2),lagRate./lateralGain, ...
        VariableNames={'frame','time','wheelSpeedMps','yawRateDegPerS','steeringDeg','lateralVelocityInputMps', ...
        'inspvaVelocityLateralMps','inspvaIncrementLateralMps','inputSlipDeg','inspvaVelocitySlipDeg','inspvaIncrementSlipDeg', ...
        'referencePositionMinusVelocityLateralM','fusedLateralM','matchLateralM','gnssLateralM','fusedYawDeg', ...
        'lateralPositionGainPerS','positionGainMinPerS','positionGainMaxPerS','quasiSteadyLateralLagM'});
    writetable(detail,fullfile(dest,'motion_reference.csv'));
    window=detail(max(1,at-27):min(n,at+23),:);writetable(window,fullfile(dest,'maximum_window.csv'));
    % Route statistics of the course offset on nearly straight driving.
    moving=after & h.longitudinalSpeed>=5 & isfinite(slipIncrement);straight=moving & abs(h.yawRate)<=deg2rad(1);
    bins=cell(0,9);
    for first=0:10:floor(t(end)/10)*10
        ix=straight & t>=first & t<first+10;if nnz(ix)<10,continue;end
        bins(end+1,:)={first,first+10,nnz(ix),median(h.longitudinalSpeed(ix)),median(estimateSlip(ix)),median(slip(ix)), ...
            median(slipIncrement(ix)),median(total(ix,2)),median(matchBody(ix,2),'omitnan')}; %#ok<AGROW>
    end
    course=cell2table(bins,VariableNames={'beginSeconds','endSeconds','straightFrames','medianSpeedMps','medianInputSlipDeg', ...
        'medianInspvaVelocitySlipDeg','medianInspvaIncrementSlipDeg','medianFusedLateralM','medianMatchLateralM'});
    writetable(course,fullfile(dest,'course_offset.csv'));disp(course);
    % Frozen-packet controls. Oracles use evaluation data and are not deployable.
    offset=median(slip(straight)-estimateSlip(straight));
    controlNames=["baseline","zero_lateral_velocity","oracle_constant_course_offset","oracle_inspva_lateral_velocity", ...
        "oracle_increment_lateral_velocity","position_gains_12"];
    controls=cell(numel(controlNames),1);controls{1}=base;
    for j=2:numel(controlNames)
        l=lateral;c=cfg;
        switch controlNames(j)
            case "zero_lateral_velocity",l.lateralVelocity(:)=0;
            case "oracle_constant_course_offset",l.lateralVelocity=l.lateralVelocity+h.longitudinalSpeed*tand(offset);
            case "oracle_inspva_lateral_velocity",l.lateralVelocity=referenceBody(:,2);
            case "oracle_increment_lateral_velocity",l.lateralVelocity=[incrementBody(2,2);incrementBody(2:end,2)];
            case "position_gains_12",c.gains(1)=12;c.gnss.positionGain=12;
        end
        controls{j}=runSynchronousLocalizationObserver(data,c,l);
    end
    rows=cell(numel(controls),9);
    for j=1:numel(controls)
        e=controls{j}.position-ref(:,1:2);position=vecnorm(e,2,2);selected=find(after);[peak,where]=max(position(after));
        atBody=rot(ref(at,3)).'*e(at,:).';lateralAll=zeros(n,1);
        for k=1:n,v=rot(ref(k,3)).'*e(k,:).';lateralAll(k)=v(2);end
        rows(j,:)={controlNames(j),rms(position(after)),prctile(position(after),95),peak,frame(selected(where)), ...
            position(at),atBody(2),mean(lateralAll(after)),rad2deg(rms(wrap(controls{j}.pose(after,3)-ref(after,3))))};
    end
    results=cell2table(rows,VariableNames={'control','positionRmseM','p95M','maximumM','maximumFrame','errorAtBaselineMaximumM', ...
        'lateralAtBaselineMaximumM','meanLateralM','yawRmseDeg'});
    writetable(results,fullfile(dest,'controls.csv'));disp(results);
    summary=struct('frame',frame(at),'time',t(at),'positionErrorM',norm(err(at,:)),'longitudinalM',total(at,1),'lateralM',total(at,2), ...
        'yawErrorDeg',rad2deg(wrap(base.pose(at,3)-ref(at,3))),'replayMaximumStateDifference',replayDifference, ...
        'decompositionMaximumResidualM',closure,'wheelSpeedMps',h.longitudinalSpeed(at),'yawRateDegPerS',rad2deg(h.yawRate(at)), ...
        'lateralVelocityInputMps',lateral.lateralVelocity(at),'inspvaVelocityLateralMps',referenceBody(at,2), ...
        'inspvaIncrementLateralMps',incrementBody(at,2),'inspvaVelocitySlipDeg',slip(at),'inspvaIncrementSlipDeg',slipIncrement(at), ...
        'positionGain',K(:,:,at),'gnssGain',Kg(:,:,at),'lidarGain',Kl(:,:,at),'lateralPositionGainPerS',lateralGain(at), ...
        'matchLateralM',matchBody(at,2),'gnssLateralM',gnssBody(at,2), ...
        'straightFramesAfterStartup',nnz(straight),'medianStraightInputSlipDeg',median(estimateSlip(straight)), ...
        'medianStraightInspvaVelocitySlipDeg',median(slip(straight)),'medianStraightInspvaIncrementSlipDeg',median(slipIncrement(straight)), ...
        'oracleConstantCourseOffsetDeg',offset,'meanFusedLateralAfterStartupM',mean(total(after,2)), ...
        'meanMatchLateralAfterStartupM',mean(matchBody(after,2),'omitnan'),'meanGnssLateralAfterStartupM',mean(gnssBody(after,2),'omitnan'), ...
        'referenceUsage',"Scoring, additive attribution and explicitly named oracle controls only", ...
        'scope',"Frozen GNSS and closed-loop LiDAR packets of the both scenario; controls do not rematch scans");
    fid=fopen(fullfile(dest,'summary.json'),'w');fprintf(fid,'%s\n',jsonencode(summary,PrettyPrint=true));fclose(fid);
    save(fullfile(out,'diagnostic.mat'),'summary','budget','trace','detail','course','results','part','K','Kg','Kl','frame','t','ref','at');
    disp(summary);disp(window(:,[1 3 4 6 7 8 12 13 14 15 17 20]));
end
function r=rot(a),r=[cos(a) -sin(a);sin(a) cos(a)];end
function a=wrap(a),a=atan2(sin(a),cos(a));end
