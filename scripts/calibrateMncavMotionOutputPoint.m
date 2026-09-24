function calibration=calibrateMncavMotionOutputPoint()
% calibrateMncavMotionOutputPoint Fit the motion-sensor lever arm on the separate drive.
% The lateral observer integrates the vehicle IMU, so its lateral velocity
% belongs to the IMU location, not to the INSPVA output point that the map,
% GNSS correction and pose state use. For a rigid body, vyOutput =
% vyObserver - forwardOffset*yawRate. Seconds 1--40 of the 12-11-24 drive fit
% the offset; the rest of that drive and the 12-09-31 evaluation drive are
% reported as held-out checks only. Writes config/mncavMotionOutputPoint.json.
    root=setupVehicleLocalization();
    prior=load(fullfile(root,'output','mncav_inspva_observer_20260915','experiment.mat'),'lateralDesign');
    % Run the stored design at its own point; the calibrated output point is
    % the product of this script, so it cannot be an input here.
    design=prior.lateralDesign;lateralCfg=design.cfg;
    lateralCfg.outputPoint=struct('forwardOffsetM',0,'identifier',"observer-point");
    parameters=jsondecode(fileread(fullfile(root,'output','mncav_interface_audit_20260916','vehicle_parameters.json')));
    drives={"raw_data_2024-06-07-12-11-24_0","output/mncav_wheel_only_20260916/calibration_sensors", ...
        "output/mncav_wheel_only_20260916/calibration_sensors/inspva.csv"; ...
        "raw_data_2024-06-07-12-09-31_0","output/mncav_wheel_only_20260916/sensors", ...
        "data/raw/Missisipi/gnss/raw_data_2024-06-07-12-09-31_0_inspva.csv"};
    samples=cell(2,1);
    for d=1:2
        clock=loadReceiverClock(fullfile(root,drives{d,3}));ins=readtable(fullfile(root,drives{d,2},'inspva.csv'));
        insTime=receiverClockTime(clock,ins.stamp_sec);start=insTime(1)+.5;stop=insTime(end)-.5;
        high=prepareWheelMotionInputs(fullfile(root,drives{d,2}),parameters,clock,start,stop);
        lateral=runLateralVelocityObserver(high,design,lateralCfg);
        yaw=deg2rad(90-ins.azimuth);
        vyReference=-ins.east_velocity.*sin(yaw)+ins.north_velocity.*cos(yaw);
        vxReference=ins.east_velocity.*cos(yaw)+ins.north_velocity.*sin(yaw);
        t=high.time;reference=interp1(insTime-start,[vxReference,vyReference],t,'linear');
        keep=all(isfinite(reference),2) & reference(:,1)>1;
        samples{d}=table(t(keep),high.yawRate(keep),lateral.lateralVelocity(keep),reference(keep,2), ...
            VariableNames={'time','yawRate','vyObserver','vyReference'});
    end
    fit=samples{1}(samples{1}.time>=1 & samples{1}.time<=40,:);
    coefficients=[ones(height(fit),1),fit.yawRate]\(fit.vyObserver-fit.vyReference);
    forwardOffset=coefficients(2);
    names=["12-11-24 fit (1-40 s)","12-11-24 held out (>40 s)","12-09-31 evaluation drive (held out)"];
    sets={fit,samples{1}(samples{1}.time>40,:),samples{2}};rows=cell(3,5);
    for j=1:3
        s=sets{j};e=s.vyObserver-s.vyReference;local=[ones(height(s),1),s.yawRate]\e;
        rows(j,:)={names(j),height(s),rms(e),rms(e-forwardOffset*s.yawRate),local(2)};
    end
    report=cell2table(rows,VariableNames={'population','samples','rawRmseMps','transportedRmseMps','locallyFittedOffsetM'});
    disp(report);
    calibration=struct('identifier',"mncav-motion-output-point-12-11-24-seconds-1-40-v1", ...
        'forwardOffsetM',forwardOffset, ...
        'model',"vyOutput = vyObserver - forwardOffsetM*yawRate; the lateral observer/IMU point lies forwardOffsetM metres ahead of the INSPVA output point", ...
        'residualOffsetMps',coefficients(1),'fitDrive',drives{1,1},'fitReceiverIntervalSeconds',[1 40], ...
        'fitSamples',height(fit),'evaluationDriveUsed',false,'runtimeReferenceUsed',false, ...
        'heldOut',report, ...
        'interpretation',"Empirical effective lever arm absorbing installation geometry and lateral-model error; not a surveyed IMU position. The 12-09-31 drive carries an additional constant lateral offset left to the online bias learner.");
    fid=fopen(fullfile(root,'config','mncavMotionOutputPoint.json'),'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(calibration,PrettyPrint=true));
    fprintf('Forward offset %.4f m written to config/mncavMotionOutputPoint.json\n',forwardOffset);
end
