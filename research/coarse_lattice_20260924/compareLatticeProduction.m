function results=compareLatticeProduction()
% compareLatticeProduction Production chain on the 0.3 m lattice and two 0.6 m pole gatings.
% Both chains use fresh coarse perception, the same map, matcher, gains and
% calibration; only the coarse pillar lattice and its stage parameters differ.
% Reference poses are used for evaluation only. RMSE after the common 2 s
% initialization transient, as in comparePyramidProduction.
    setupVehicleLocalization;dest=fileparts(mfilename('fullpath'));
    runs={"0.3 m lattice",'output/mncav_coarse_localization_20260924b'; ...
        "0.6 m lattice, radial-std pole gate",'output/mncav_coarse_localization_20260924c'; ...
        "0.6 m lattice, density-core pole gate",'output/mncav_coarse_localization_20260924d'};
    rows=cell(0,10);stage=cell(size(runs,1),1);
    for j=1:size(runs,1)
        E=load(fullfile(runs{j,2},'observer','experiment.mat'),'runs','reference');ref=E.reference;t=E.runs{1}.estimate.time;post=t>=t(1)+2;
        for k=1:numel(E.runs)
            est=E.runs{k}.estimate;e=vecnorm(est.position-ref(:,1:2),2,2);
            yaw=rad2deg(atan2(sin(est.pose(:,3)-ref(:,3)),cos(est.pose(:,3)-ref(:,3))));
            rows(end+1,:)={runs{j,1},E.runs{k}.scenario,100*rms(e),100*rms(e(post)),100*median(e(post)), ...
                100*prctile(e(post),95),100*max(e(post)),nnz(e(post)>.3),rms(yaw(post)),E.runs{k}.seconds}; %#ok<AGROW>
        end
        r=E.runs{1}.estimate.matchingResults;a=cellfun(@(x)x.accepted,r);
        p=cell2mat(cellfun(@(x)x.poseXYTheta,r,UniformOutput=false));em=vecnorm(p(:,1:2)-ref(:,1:2),2,2);
        sec=cellfun(@(x)x.matchingSeconds,r);
        M=load(fullfile(runs{j,2},'matching','report.mat'),'report');c=M.report.calls;
        es=hypot(c.x-c.referenceX,c.y-c.referenceY);as=logical(c.accepted);
        stage{j}=struct('label',runs{j,1},'fusedRunMatchingRmseCm',100*rms(em(a)),'fusedRunMatchingMaxCm',100*max(em(a)), ...
            'fusedRunMatchingMsMedian',1e3*median(sec),'fusedRunMatchingMsP95',1e3*prctile(sec,95), ...
            'recursiveAccepted',nnz(as),'recursiveRmseCm',100*rms(es(as)),'recursiveMedianCm',100*median(es(as)),'recursiveP95Cm',100*prctile(es(as),95), ...
            'recursiveMaxCm',100*max(es(as)),'recursiveAbove30cm',nnz(es(as)>.3),'recursiveAbove50cm',nnz(es(as)>.5), ...
            'recursiveYawRmseDeg',M.report.summary.yawRmseDeg,'recursiveTotalMsMedian',M.report.summary.totalMedianMs, ...
            'recursivePerceptionMsMedian',M.report.summary.perceptionMedianMs,'recursiveRegistrationMsMedian',M.report.summary.registrationMedianMs, ...
            'recursiveCallsOver100ms',M.report.summary.callsOver100ms);
        fprintf('%s: fused-run matching RMSE %.2f cm (max %.2f, %.1f ms median) | recursive stage accepted %d RMSE %.2f median %.2f P95 %.2f max %.2f >30cm %d >50cm %d yaw %.3f deg, total %.1f ms (perception %.1f, registration %.1f ms), >100 ms %d\n', ...
            runs{j,1},stage{j}.fusedRunMatchingRmseCm,stage{j}.fusedRunMatchingMaxCm,stage{j}.fusedRunMatchingMsMedian, ...
            stage{j}.recursiveAccepted,stage{j}.recursiveRmseCm,stage{j}.recursiveMedianCm,stage{j}.recursiveP95Cm,stage{j}.recursiveMaxCm,stage{j}.recursiveAbove30cm, ...
            stage{j}.recursiveAbove50cm,stage{j}.recursiveYawRmseDeg,stage{j}.recursiveTotalMsMedian,stage{j}.recursivePerceptionMsMedian,stage{j}.recursiveRegistrationMsMedian,stage{j}.recursiveCallsOver100ms);
    end
    results=cell2table(rows,VariableNames={'run','scenario','rmseAllCm','rmseCm','medianCm','p95Cm','maxCm','above30cm','headingRmseDeg','runtimeSeconds'});
    disp(results(:,1:9));writetable(results,fullfile(dest,'scenario_comparison.csv'));
    writetable(struct2table([stage{:}]),fullfile(dest,'matching_stage_comparison.csv'));
end
