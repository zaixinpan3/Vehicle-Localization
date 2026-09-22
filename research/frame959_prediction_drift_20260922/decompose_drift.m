function summary=decompose_drift()
% decompose_drift Attribute prediction drift without adding error magnitudes.
% All reference-derived motion/velocity is offline diagnostic information.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    saved=load('output/temporal_perception_20260922/five_frame_matching/report.mat','report');
    c=saved.report.calls;d=saved.report.deadReckoning;frames=(950:959).';
    basis=rotation(c.referencePsi(959));rows=zeros(numel(frames),18);
    initial=([c.x(949) c.y(949)]-[c.referenceX(949) c.referenceY(949)])*basis;
    for j=1:numel(frames)
        k=frames(j);a=c(k-1,:);b=c(k,:);dt=b.timeSeconds-a.timeSeconds;
        odometry=([d.x(k) d.y(k)]-[d.x(k-1) d.y(k-1)])*rotation(d.psi(k-1));
        reference=([b.referenceX b.referenceY]-[a.referenceX a.referenceY])*rotation(a.referencePsi);
        inputError=(odometry-reference)*rotation(a.referencePsi).';
        headingError=odometry*(rotation(a.psi).'-rotation(a.referencePsi).');
        match=[b.x-b.predictedX b.y-b.predictedY];
        priorError=[a.x-a.referenceX a.y-a.referenceY];
        predictedError=[b.predictedX-b.referenceX b.predictedY-b.referenceY];
        residual=norm(priorError+inputError+headingError-predictedError);
        assert(residual<1e-7);
        rows(j,:)=[k,dt,norm(priorError),norm(predictedError),b.positionErrorM, ...
            inputError*basis,headingError*basis,match*basis,odometry/dt,reference/dt, ...
            rad2deg(a.psi-a.referencePsi),residual,b.accepted];
    end
    steps=array2table(rows,VariableNames={'frame','dt','previousErrorM','predictedErrorM','matchedErrorM', ...
        'motionForwardM','motionLeftM','headingForwardM','headingLeftM','matchForwardM','matchLeftM', ...
        'odometryForwardMps','odometryLeftMps','referenceForwardMps','referenceLeftMps','previousYawErrorDeg', ...
        'decompositionResidualM','accepted'});
    writetable(steps,fullfile(dest,'steps.csv'));
    % Stop before frame 959 matching: this is the disputed incoming prediction.
    motion=sum(steps{:,{'motionForwardM','motionLeftM'}},1);
    heading=sum(steps{:,{'headingForwardM','headingLeftM'}},1);
    matching=sum(steps{1:end-1,{'matchForwardM','matchLeftM'}},1);
    endpoint=[c.predictedX(959)-c.referenceX(959),c.predictedY(959)-c.referenceY(959)]*basis;
    assert(norm(initial+motion+heading+matching-endpoint)<1e-7);
    terms=table(["frame949_after_matching";"motion_increment_mismatch";"prior_heading_mismatch"; ...
        "matching_corrections_950_958";"frame959_prediction"],[initial;motion;heading;matching;endpoint], ...
        VariableNames={'term','forwardLeftM'});writetable(terms,fullfile(dest,'error_budget.csv'));
    % Recreate the exact wheel/lateral inputs used by the stored matching run.
    [prepared,~,~]=prepareMncavObserverReplay('output/mncav_wheel_only_20260916/sensors', ...
        'output/mncav_interface_audit_20260916/vehicle_parameters.json',table(),0,IncludeOdom=false);
    design=load('output/mncav_inspva_observer_20260915/experiment.mat','lateralDesign');
    h=prepared.highRate;lateral=runLateralVelocityObserver(h,design.lateralDesign,design.lateralDesign.cfg);
    query=c.timeSeconds(949:959);
    path=integrateRecordedPlanarMotion(h.time,[h.longitudinalSpeed,lateral.lateralVelocity,h.yawRate],query);
    expected=([d.x(949:959) d.y(949:959)]-[d.x(949) d.y(949)])*rotation(d.psi(949));
    motionReproduction=max(abs(path(:,1:2)-expected),[],'all');assert(motionReproduction<1e-7);
    pva=readtable('data/raw/Missisipi/gnss/raw_data_2024-06-07-12-09-31_0_inspva.csv');
    clock=saved.report.metadata.clock;
    time=(pva.gps_week-clock.receiverOriginGpsWeek)*604800+pva.gps_seconds-clock.receiverOriginGpsSeconds;
    first=receiverClockTime(clock,c.rosStamp(1));time=time-first;
    azimuth=deg2rad(pva.azimuth_deg);
    body=[sin(azimuth).*pva.east_velocity_mps+cos(azimuth).*pva.north_velocity_mps, ...
        -cos(azimuth).*pva.east_velocity_mps+sin(azimuth).*pva.north_velocity_mps];
    select=h.time>=query(1)&h.time<=query(end);t=h.time(select);
    velocity=interp1(time,body,t,'linear');assert(all(isfinite(velocity),'all'));
    inputAudit=table(t,h.longitudinalSpeed(select),lateral.lateralVelocity(select),velocity(:,1),velocity(:,2), ...
        h.steeringAngle(select),h.yawRate(select),h.lateralAcceleration(select),lateral.lateralAccelerationBias(select), ...
        lateral.dynamicState(select,1),lateral.diagnostics.dynamicParticipation(select), ...
        VariableNames={'time','wheelForwardMps','observerLeftMps','inspvaForwardMps','inspvaLeftMps', ...
        'roadWheelAngleRad','yawRateRadps','lateralAccelerationMps2','estimatedAccelBiasMps2', ...
        'dynamicLeftMps','dynamicParticipation'});
    writetable(inputAudit,fullfile(dest,'motion_inputs.csv'));
    kinematic=h.lateralAcceleration-lateral.lateralAccelerationBias-h.yawRate.*h.longitudinalSpeed;
    balance=struct('meanKinematicVyRateMps2',mean(kinematic(select)), ...
        'meanDynamicInjectionMps2',mean(lateral.correctionInjection.dynamic(select,1)), ...
        'meanTotalVyRateMps2',mean(lateral.lateralVelocityRate(select)), ...
        'meanMasterVyMps',mean(lateral.lateralVelocity(select)), ...
        'meanDynamicVyMps',mean(lateral.dynamicState(select,1)), ...
        'meanParticipation',mean(lateral.diagnostics.dynamicParticipation(select)));
    fid=fopen(fullfile(dest,'observer_balance.json'),'w');assert(fid>=0);
    fprintf(fid,'%s\n',jsonencode(balance,PrettyPrint=true));fclose(fid);
    summary=struct('startingFrame',949,'predictionFrame',959,'durationSeconds',sum(steps.dt), ...
        'basis','forward/left axes of reference frame 959; signed vector accounting', ...
        'startingErrorM',norm(initial),'predictionErrorM',norm(endpoint),'terms',table2struct(terms), ...
        'maximumDecompositionResidualM',max(steps.decompositionResidualM),'motionReproductionMaxAbsM',motionReproduction, ...
        'meanWheelForwardMps',mean(inputAudit.wheelForwardMps),'meanObserverLeftMps',mean(inputAudit.observerLeftMps), ...
        'meanInspvaForwardMps',mean(inputAudit.inspvaForwardMps),'meanInspvaLeftMps',mean(inputAudit.inspvaLeftMps), ...
        'meanDynamicParticipation',mean(inputAudit.dynamicParticipation),'productionChanged',false);
    fid=fopen(fullfile(dest,'summary.json'),'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(summary,PrettyPrint=true));disp(terms);disp(summary);
    save('output/frame959_prediction_drift_20260922/motion_audit.mat','h','lateral','steps','terms','inputAudit','summary');
end
function r=rotation(a)
    r=[cos(a) -sin(a);sin(a) cos(a)];
end
