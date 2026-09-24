function calibration=calibrateLateralLeverArm()
% calibrateLateralLeverArm Fit the output-point lateral lever arm on the separate drive.
% Model: vyObserver - vyReference = a + b*r, with r the measured yaw rate. b is
% the longitudinal offset of the CG-referenced lateral observer from the
% INSPVA/map output point. Seconds 1--40 of 12-11-24 fit; the remainder of
% that drive and the whole 12-09-31 evaluation drive are held out.
    setupVehicleLocalization;dest=fileparts(mfilename('fullpath'));
    prior=load('output/mncav_inspva_observer_20260915/experiment.mat','lateralDesign');design=prior.lateralDesign;
    parameters=jsondecode(fileread('output/mncav_interface_audit_20260916/vehicle_parameters.json'));
    drives={"12-11-24","output/mncav_wheel_only_20260916/calibration_sensors", ...
        "output/mncav_wheel_only_20260916/calibration_sensors/inspva.csv", ...
        "output/mncav_wheel_only_20260916/calibration_sensors/inspva.csv"; ...
        "12-09-31","output/mncav_wheel_only_20260916/sensors", ...
        "data/raw/Missisipi/gnss/raw_data_2024-06-07-12-09-31_0_inspva.csv", ...
        "output/mncav_wheel_only_20260916/sensors/inspva.csv"};
    samples=cell(2,1);
    for d=1:2
        clock=loadReceiverClock(drives{d,3});ins=readtable(drives{d,4});
        insTime=receiverClockTime(clock,ins.stamp_sec);
        start=insTime(1)+.5;stop=insTime(end)-.5;
        high=prepareWheelMotionInputs(drives{d,2},parameters,clock,start,stop);
        lateral=runLateralVelocityObserver(high,design,design.cfg);
        yaw=deg2rad(90-ins.azimuth);
        vyRef=-ins.east_velocity.*sin(yaw)+ins.north_velocity.*cos(yaw);
        vxRef=ins.east_velocity.*cos(yaw)+ins.north_velocity.*sin(yaw);
        t=high.time;ref=interp1(insTime-start,[vxRef,vyRef],t,'linear');
        keep=all(isfinite(ref),2) & ref(:,1)>1;
        samples{d}=table(t(keep),high.yawRate(keep),lateral.lateralVelocity(keep),ref(keep,2),ref(keep,1), ...
            VariableNames={'time','yawRate','vyObserver','vyReference','vxReference'});
    end
    fit=samples{1}(samples{1}.time>=1 & samples{1}.time<=40,:);
    X=[ones(height(fit),1),fit.yawRate];coefficients=X\(fit.vyObserver-fit.vyReference);
    rows=cell(0,6);
    names=["12-11-24 fit (1-40 s)","12-11-24 held out (>40 s)","12-09-31 evaluation drive"];
    sets={samples{1}(samples{1}.time>=1 & samples{1}.time<=40,:),samples{1}(samples{1}.time>40,:),samples{2}};
    for j=1:3
        s=sets{j};e=s.vyObserver-s.vyReference;
        transported=e-coefficients(2)*s.yawRate;corrected=transported-coefficients(1);
        local=[ones(height(s),1),s.yawRate]\e;
        rows(end+1,:)={names(j),height(s),rms(e),rms(transported),rms(corrected),local(2)}; %#ok<AGROW>
    end
    report=cell2table(rows,VariableNames={'population','samples','rawRmseMps','leverArmOnlyRmseMps', ...
        'leverArmAndOffsetRmseMps','locallyFittedLeverArmM'});
    disp(report);writetable(report,fullfile(dest,'lever_arm_calibration.csv'));
    calibration=struct('leverArmM',-coefficients(2),'offsetMps',coefficients(1), ...
        'fitDrive',"raw_data_2024-06-07-12-11-24_0",'fitSeconds',[1 40],'evaluationDriveUsed',false, ...
        'model',"vyPoint = vyObserver + leverArmM*yawRate; leverArmM is the output point's forward offset from the observer point");
    fprintf('Calibrated output-point forward offset %.3f m, lateral offset %.3f m/s\n',calibration.leverArmM,calibration.offsetMps);
    fid=fopen(fullfile(dest,'lever_arm_calibration.json'),'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(calibration,PrettyPrint=true));
end
