function runMississippiNominalSensitivity(folder,tmp)
% Report the nominal-only calibration optimum alongside fixed robust tuning.
% Parameters are selected from the retained calibration grid, never test scores.
    arguments
        folder string="output/mississippi_baselines_20261030"
        tmp string="/tmp/pdvl_mississippi_baselines"
    end
    setupVehicleLocalization();addpath(fileparts(mfilename('fullpath')));maxNumCompThreads(2);
    grid=readtable(fullfile(folder,'ekf_tuning_grid.csv'),TextType='string');rows={};
    for drive=1:2
        b=load(fullfile(tmp,"drive"+drive+"_paired.mat"),'data','lateral','reference');d=b.data;l=b.lateral;ref=b.reference;cfg=mncavSmallOffsetObserverConfig();
        v=[cos(ref(1,3)),-sin(ref(1,3));sin(ref(1,3)),cos(ref(1,3))]*[d.highRate.longitudinalSpeed(1);l.lateralVelocity(1)];cfg.initialState=[ref(1,1);v(1);0;ref(1,2);v(2);0;ref(1,3);0];t=d.highRate.time-d.highRate.time(1);
        for method=["ekf","iekf"]
            candidates=grid(grid.method==method,:);[~,j]=min(candidates.rmseM);chosen=candidates(j,:);iterations=1;if method=="iekf",iterations=5;end
            for scenario=["both","gnss_loss","no_gnss"]
                current=d;if scenario=="gnss_loss",current.gnss.valid(t>=.35*t(end) & t<.65*t(end))=false;end
                if scenario=="no_gnss",current.gnss.valid(:)=false;end
                timer=tic;e=runInvariantEkfBaseline(current,l,cfg,LidarTemperature=chosen.temperature,ProcessScale=chosen.processScale,Iterations=iterations);seconds=toc(timer);
                errors=vecnorm(e.pose(:,1:2)-ref(:,1:2),2,2);angle=atan2(sin(e.pose(:,3)-ref(:,3)),cos(e.pose(:,3)-ref(:,3)));values=[rms(errors),prctile(errors,95),max(errors),rad2deg(rms(angle))];if any(~isfinite(e.pose),'all'),values(:)=Inf;end
                rows(end+1,:)={drive,method,scenario,numel(t),chosen.temperature,chosen.processScale,values(1),values(2),values(3),values(4),seconds}; %#ok<AGROW>
                fprintf('NOMINAL SENSITIVITY drive%d %s %s RMSE %.5g max %.5g\n',drive,method,scenario,values(1),values(3));
            end
        end
    end
    writetable(cell2table(rows,VariableNames={'drive','method','scenario','frames','temperature','processScale','rmseM','p95M','maximumM','headingRmseDeg','runtimeS'}),fullfile(folder,'nominal_sensitivity.csv'));
end
