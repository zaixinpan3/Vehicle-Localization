function benchmarkPartialMatching()
% benchmarkPartialMatching Paired alternating-order timing on fixed causal seeds.
% Includes local cropping, view conditioning, and geometric registration.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    old=load('output/mississippi_mapping_calibrated/view_conditioned_cloud.mat','cloud');
    adopted=load('output/mississippi_mapping_calibrated/view_conditioned_cloud.mat','cloud');maps={old.cloud,adopted.cloud};
    data=load('output/root_cause_matching_20260929/finalSurface_sources.mat','sources');
    b=readtable('research/matching_objective_20260929/final_raw.csv');n=readtable(fullfile(dest,'production.csv'));replays={b,n};
    o=load('output/line_direction_matching_20260928/sources.mat','motion');m=o.motion;
    a=load('output/line_direction_matching_20260928/production/report.mat','report');calls=a.report.calls;cfg=distributionRegistrationConfig();baselineCfg=cfg;baselineCfg.partialSign.enabled=false;configs={baselineCfg,cfg};
    times=zeros(1170,2);differences=times;
    for k=1:1170
        order=1:2;if mod(k,2)==0,order=[2 1];end
        for j=order
            predicted=calls{1,{'predictedX','predictedY','predictedPsi'}};
            if k>1
                previous=replays{j}{k-1,{'x','y','psi'}};R=@(q)[cos(q) -sin(q);sin(q) cos(q)];
                predicted=[previous(1:2)+(m(k,1:2)-m(k-1,1:2))*R(m(k-1,3))*R(previous(3)).',previous(3)+m(k,3)-m(k-1,3)];
            end
            timer=tic;r=matchLocalProbabilityCloud(maps{j},data.sources{k},predicted,configs{j});times(k,j)=1000*toc(timer);
            event=registrationSupport.registrationPoseMeasurement(r,calls.timeSeconds(k));estimate=predicted;if ~isempty(event),estimate=event.pose;end
            differences(k,j)=max(abs(estimate-replays{j}{k,{'x','y','psi'}}));
        end
    end
    assert(max(differences,[],'all')<1e-7);
    report=table((1:1170).',times(:,1),times(:,2),times(:,2)-times(:,1),differences(:,1),differences(:,2), ...
        VariableNames={'frame','legacyMs','adoptedMs','differenceMs','legacyPoseDifference','adoptedPoseDifference'});
    writetable(report,fullfile(dest,'paired_matching_runtime.csv'));
    summary=table(median(times(:,1)),median(times(:,2)),median(times(:,2)-times(:,1)),prctile(times(:,1),95),prctile(times(:,2),95), ...
        VariableNames={'legacyMedianMs','adoptedMedianMs','pairedMedianDifferenceMs','legacyP95Ms','adoptedP95Ms'});
    writetable(summary,fullfile(dest,'paired_matching_runtime_summary.csv'));disp(summary);
end
