function tuning=refineMississippiEkfTuning(folder,tmp)
% Freeze finite candidate selection across nominal and lost-GNSS calibration.
% For each temperature retain its best nominal process scale, then score the
% candidate under both sensors, middle-30% loss and complete GNSS withdrawal.
    [d,l,ref,cfg]=mississippiBaselineCalibrationInputs(tmp);grid=readtable(fullfile(folder,'ekf_tuning_grid.csv'),TextType='string');rows={};tuning=struct();t=d.highRate.time;middle=t>=t(1)+.35*(t(end)-t(1)) & t<t(1)+.65*(t(end)-t(1));
    for method=["ekf","iekf","gnss_ekf"]
        candidates=grid(grid.method==method,:);best=Inf;
        shortlist=candidates([],:);
        if method=="gnss_ekf",shortlist=candidates;else
            for value=unique(candidates.temperature).'
                group=candidates(candidates.temperature==value,:);[~,j]=min(group.rmseM);shortlist=[shortlist;group(j,:)]; %#ok<AGROW>
            end
        end
        for index=1:height(shortlist)
            candidate=shortlist(index,:);temperature=candidate.temperature;scores=[candidate.rmseM,Inf,Inf];its=1;if method=="iekf",its=5;end
            for regime=2:3
                current=d;if method=="gnss_ekf",current=rmfield(current,'lidarOverlap');end
                if regime==2,current.gnss.valid(middle)=false;else,current.gnss.valid(:)=false;end
                e=runInvariantEkfBaseline(current,l,cfg,LidarTemperature=temperature,ProcessScale=candidate.processScale,Iterations=its);
                errors=vecnorm(e.pose(:,1:2)-ref(:,1:2),2,2);scores(regime)=rms(errors);if any(~isfinite(e.pose),'all'),scores(regime)=Inf;end
            end
            score=mean(scores.^2);rows(end+1,:)={method,temperature,candidate.processScale,scores(1),scores(2),scores(3),score}; %#ok<AGROW>
            fprintf('ROBUST TUNING %s T %.3g Q %.3g nominal/gap/none %s\n',method,temperature,candidate.processScale,mat2str(scores,5));
            if score<best,best=score;tuning.(method)=struct('temperature',temperature,'processScale',candidate.processScale,'trainingMeanSquaredErrorM2',score);end
            writetable(cell2table(rows,VariableNames={'method','temperature','processScale','nominalRmseM','gapRmseM','noGnssRmseM','meanSquaredErrorM2'}),fullfile(folder,'ekf_availability_tuning.csv'));
        end
        assert(isfield(tuning,method),'VehicleLocalization:BaselineCalibrationFailure','No finite calibration candidate.');
    end
    tuning.gridVersion=3;tuning.protocol="Drive2 t<40 s only; best nominal process per temperature, then equal MSE over nominal, middle-30% GNSS loss and no GNSS. Frozen before final tests; finite search, not a global optimum";
    fid=fopen(fullfile(folder,'ekf_tuning.json'),'w');fprintf(fid,'%s',jsonencode(tuning,PrettyPrint=true));fclose(fid);
end
