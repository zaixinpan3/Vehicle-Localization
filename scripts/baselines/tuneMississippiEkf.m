function tuneMississippiEkf(folder,tmp)
% Tune only on drive-2 t<40 s; holdout scores never select parameters.
    arguments
        folder string="output/mississippi_baselines_20261030"
        tmp string="/tmp/pdvl_mississippi_baselines"
    end
    setupVehicleLocalization();addpath(fileparts(mfilename('fullpath')));maxNumCompThreads(2);
    [d,l,ref,cfg]=mississippiBaselineCalibrationInputs(tmp);
    rows={};tuning=struct();
    for method=["ekf","iekf","gnss_ekf"]
        best=Inf;iterations=1;if method=="iekf",iterations=5;end
        current=d;temperatures=[.001,.01,.1,1,10,100];
        if method=="gnss_ekf",current=rmfield(current,'lidarOverlap');temperatures=1;end
        for temperature=temperatures
            for process=[.1,1,10,100,1000]
                e=runInvariantEkfBaseline(current,l,cfg,LidarTemperature=temperature,ProcessScale=process,Iterations=iterations);
                err=vecnorm(e.pose(:,1:2)-ref(:,1:2),2,2);score=rms(err);if any(~isfinite(err)),score=Inf;end
                rows(end+1,:)={method,temperature,process,score,max(err)}; %#ok<AGROW>
                fprintf('TUNING %s temperature %.3g process %.3g rmse %.6g\n',method,temperature,process,score);
                if score<best,best=score;tuning.(method)=struct('temperature',temperature,'processScale',process,'trainingRmseM',score);end
                writetable(cell2table(rows,VariableNames={'method','temperature','processScale','rmseM','maximumM'}),fullfile(folder,'ekf_tuning_grid.csv'));
            end
        end
    end
    tuning.gridVersion=2;
    tuning.protocol="Drive2 prefix t<40 s only; original reference; squared position loss; frozen before final test replay";
    fid=fopen(fullfile(folder,'ekf_tuning.json'),'w');fprintf(fid,'%s',jsonencode(tuning,PrettyPrint=true));fclose(fid);
end
