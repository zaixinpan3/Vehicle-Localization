function report=exportMncavVdbTrajectory(out,plantReport,outputFolder)
% exportMncavVdbTrajectory Export VDB truth separately from synthetic sensors.
% VDB uses SAE forward/right/down. Project sensors use forward/left/up.
% Wheel rates come from the tire rotational states, never from body speed.
    cfg=plantReport.config;v=out.vehicle;w=out.wheels;
    t=v.InertFrm.Cg.Disp.X.Time(:);n=numel(t);
    data=zeros(n,40);data(:,1)=t;
    data(:,2:4)=[col(v.InertFrm.Cg.Disp.X),col(v.InertFrm.Cg.Disp.Y),col(v.InertFrm.Cg.Disp.Z)];
    data(:,5:7)=[col(v.InertFrm.Cg.Ang.phi),col(v.InertFrm.Cg.Ang.theta),col(v.InertFrm.Cg.Ang.psi)];
    data(:,8:10)=[col(v.BdyFrm.Cg.Vel.xdot),col(v.BdyFrm.Cg.Vel.ydot),col(v.BdyFrm.Cg.Vel.zdot)];
    data(:,11:13)=[col(v.BdyFrm.Cg.AngVel.p),col(v.BdyFrm.Cg.AngVel.q),col(v.BdyFrm.Cg.AngVel.r)];
    % ax/ay/az include rotating-frame transport; xddot/yddot/zddot are
    % derivatives of the body velocity coordinates and omit omega cross v.
    data(:,14:16)=9.81*[col(v.BdyFrm.Cg.Acc.ax),col(v.BdyFrm.Cg.Acc.ay),col(v.BdyFrm.Cg.Acc.az)];
    data(:,17:20)=w.TireFrame.Omega.Data;data(:,21:24)=w.Steering.WhlAng.Data;
    data(:,25:28)=w.TireFrame.Fz.Data;
    data(:,29:32)=w.TireFrame.Re.Data;data(:,33:36)=w.TireFrame.Kappa.Data;
    data(:,37:40)=w.TireFrame.Alpha.Data;
    assert(all(isfinite(data),'all'),'VehicleLocalization:VdbNonfinite','Nonfinite plant output.');
    done=find(out.driver.Data(:,1)>=plantReport.routePointCount-3 & data(:,8)<.1 & t>10,1);
    if ~isempty(done)
        last=min(n,done+round(2/cfg.simulation.sampleTimeSeconds));data=data(1:last,:);t=t(1:last);n=last;
    end
    names={'time','x','y','zDown','roll','pitch','yaw','vx','vyRight','vzDown','p','q','r', ...
        'ax','ayRight','azDown','omegaFL','omegaFR','omegaRL','omegaRR','steerFL','steerFR','steerRL','steerRR', ...
        'fzFL','fzFR','fzRL','fzRR','reFL','reFR','reRL','reRR', ...
        'kappaFL','kappaFR','kappaRL','kappaRR','alphaFL','alphaFR','alphaRL','alphaRR'};
    writetable(array2table(data,'VariableNames',names),fullfile(outputFolder,'plant_truth.csv'));
    % A fixed nominal sensor radius from the stationary plant, not a
    % time-varying truth correction or a velocity fit on the evaluation lap.
    stationary=t>=2 & t<cfg.simulation.settleSeconds;
    interface=struct('effectiveRadiusM',mean(data(stationary,29:32),1), ...
        'lagCompensationSeconds',zeros(1,4),'source', ...
        "Nominal static loaded tire radii at t=2..3 s; frozen before driving; no body-speed fit");
    f=fopen(fullfile(outputFolder,'sensor_interface.json'),'w');fprintf(f,'%s\n',jsonencode(interface,PrettyPrint=true));fclose(f);
    report=struct('samples',n,'durationSeconds',t(end),'minimumNormalLoadN',min(data(:,25:28),[],'all'), ...
        'maximumSpeedMps',max(data(:,8)),'maximumAbsLateralVelocityMps',max(abs(data(:,9))), ...
        'maximumAbsRollDeg',rad2deg(max(abs(data(:,5)))),'maximumAbsPitchDeg',rad2deg(max(abs(data(:,6)))), ...
        'maximumLateralAccelerationMps2',max(abs(data(:,15))), ...
        'maximumNearestWaypointDistanceM',max(out.driver.Data(1:n,2)),'finalRouteIndex',out.driver.Data(n,1),'routePointCount',plantReport.routePointCount,'lapCompleted',~isempty(done));
    fid=fopen(fullfile(outputFolder,'plant_summary.json'),'w');c=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));
end
function x=col(ts)
    x=squeeze(ts.Data);x=x(:);
end
