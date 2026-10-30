function runMississippiNdtBaseline(folder,tmp)
% Calibrate NDT on twenty prefix scans; test with only the EKF prediction seed.
    arguments
        folder string="output/mississippi_baselines_20261030"
        tmp string="/tmp/pdvl_mississippi_baselines"
    end
    setupVehicleLocalization();addpath(fileparts(mfilename('fullpath')));maxNumCompThreads(2);[d,~,ref]=mississippiBaselineCalibrationInputs(tmp);
    ids=unique(round(linspace(1,size(ref,1),20)));best=Inf;calibrationRows={};
    for cellSize=[.5,1,2]
        errors=nan(numel(ids),3);
        for j=1:numel(ids)
            k=ids(j);seed=ref(k,:).'+[.3;-.2;deg2rad(2)];m=mississippiNdtMeasurement(d.lidarOverlap.map,d.lidarOverlap.sources{k},seed,cellSize,eye(3));
            if ~isempty(m),errors(j,:)=m.pose.'-ref(k,:);errors(j,3)=atan2(sin(errors(j,3)),cos(errors(j,3)));end
        end
        good=all(isfinite(errors),2);score=Inf;if all(good),score=mean(sum(errors(:,1:2).^2,2));end
        calibrationRows(end+1,:)={cellSize,numel(ids),nnz(good),score}; %#ok<AGROW>
        if score<best,best=score;chosenCell=cellSize;noise=errors.'*errors/size(errors,1)+1e-9*diag([1,1,.01]);end
        fprintf('NDT CALIBRATION cell %.3g coverage %d/%d score %.6g\n',cellSize,nnz(good),numel(ids),score);
    end
    writetable(cell2table(calibrationRows,VariableNames={'cellSizeM','frames','admitted','meanSquaredPositionErrorM2'}),fullfile(folder,'ndt_calibration.csv'));assert(isfinite(best),'VehicleLocalization:NdtCalibration','No fully supported calibration candidate.');
    tuning=jsondecode(fileread(fullfile(folder,'ekf_tuning.json')));rows={};accepted=0;
    for drive=1:2
        b=load(fullfile(tmp,"drive"+drive+"_paired.mat"),'data','lateral','reference');data=b.data;l=b.lateral;ref=b.reference;cfg=mncavSmallOffsetObserverConfig();v=[cos(ref(1,3)),-sin(ref(1,3));sin(ref(1,3)),cos(ref(1,3))]*[data.highRate.longitudinalSpeed(1);l.lateralVelocity(1)];cfg.initialState=[ref(1,1);v(1);0;ref(1,2);v(2);0;ref(1,3);0];t=data.highRate.time-data.highRate.time(1);
        for scenario=["both","gnss_loss","no_gnss"]
            current=data;if scenario=="gnss_loss",current.gnss.valid(t>=.35*t(end) & t<.65*t(end))=false;end;if scenario=="no_gnss",current.gnss.valid(:)=false;end
            accepted=0;timer=tic;e=runInvariantEkfBaseline(current,l,cfg,ProcessScale=tuning.ekf.processScale,PoseProvider=@provider);seconds=toc(timer);err=vecnorm(e.pose(:,1:2)-ref(:,1:2),2,2);angle=atan2(sin(e.pose(:,3)-ref(:,3)),cos(e.pose(:,3)-ref(:,3)));scores=[rms(err),prctile(err,95),max(err),rad2deg(rms(angle))];if any(~isfinite(e.pose),'all'),scores(:)=Inf;end
            rows(end+1,:)={drive,scenario,numel(t),scores(1),scores(2),scores(3),scores(4),seconds,accepted}; %#ok<AGROW>
            fprintf('NDT drive%d %s RMSE %.6g max %.6g accepted %d\n',drive,scenario,scores(1),scores(3),accepted);
        end
    end
    writetable(cell2table(rows,VariableNames={'drive','scenario','frames','rmseM','p95M','maximumM','headingRmseDeg','runtimeS','admittedFrames'}),fullfile(folder,'ndt_metrics.csv'));
    f=fopen(fullfile(folder,'ndt_summary.json'),'w');fprintf(f,'%s',jsonencode(struct('cellSizeM',chosenCell,'calibrationSecondMoment',noise,'calibrationFrames',ids,'processScale',tuning.ekf.processScale,'protocol',"Twenty drive2 prefix scans with declared 0.3,-0.2m,2deg perturbations for supervised front-end calibration only; test seeds only EKF predictions. Gaussian-support samples, not raw scan NDT; semantics/masses ignored"),PrettyPrint=true));fclose(f);
    function m=provider(k,p)
        m=[];if ~current.lidarOverlap.valid(k),return;end
        m=mississippiNdtMeasurement(current.lidarOverlap.map,current.lidarOverlap.sources{k},p,chosenCell,noise);accepted=accepted+~isempty(m);
    end
end
