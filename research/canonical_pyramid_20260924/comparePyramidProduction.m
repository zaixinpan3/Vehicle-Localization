function results=comparePyramidProduction()
% comparePyramidProduction Production chain before and after the canonical map pyramid.
% Both chains use fresh coarse perception, closed-loop GNSS-aided rematching,
% the same map, gains and calibration; only the registration driver differs.
% Reference poses are used for evaluation only.
    setupVehicleLocalization;dest=fileparts(mfilename('fullpath'));
    runs={"before (single level)",'output/mncav_coarse_localization_20260924'; ...
        "after (canonical pyramid)",'output/mncav_coarse_localization_20260924b'};
    rows=cell(0,10);stage=cell(2,1);
    for j=1:2
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
        retained=0;if isfield(r{1},'pyramid'),retained=nnz(cellfun(@(x)x.pyramid.coarseRetained,r));end
        stage{j}=struct('label',runs{j,1},'fusedRunMatchingRmseCm',100*rms(em(a)),'fusedRunMatchingMaxCm',100*max(em(a)), ...
            'fusedRunMatchingMsMedian',1e3*median(sec),'fusedRunMatchingMsP95',1e3*prctile(sec,95),'fusedRunCoarseRetained',retained, ...
            'recursiveAccepted',nnz(as),'recursiveRmseCm',100*rms(es(as)),'recursiveP95Cm',100*prctile(es(as),95), ...
            'recursiveMaxCm',100*max(es(as)),'recursiveAbove30cm',nnz(es(as)>.3),'recursiveAbove50cm',nnz(es(as)>.5), ...
            'recursiveFrame851Cm',100*es(851),'recursiveTotalMsMedian',M.report.summary.totalMedianMs, ...
            'recursiveRegistrationMsMedian',M.report.summary.registrationMedianMs);
        fprintf('%s: fused-run matching RMSE %.2f cm (max %.2f, %.1f ms median, coarse retained %d) | recursive stage accepted %d RMSE %.2f P95 %.2f max %.2f >30cm %d >50cm %d frame851 %.1f cm, total %.1f ms (registration %.1f ms)\n', ...
            runs{j,1},stage{j}.fusedRunMatchingRmseCm,stage{j}.fusedRunMatchingMaxCm,stage{j}.fusedRunMatchingMsMedian,retained, ...
            stage{j}.recursiveAccepted,stage{j}.recursiveRmseCm,stage{j}.recursiveP95Cm,stage{j}.recursiveMaxCm,stage{j}.recursiveAbove30cm, ...
            stage{j}.recursiveAbove50cm,stage{j}.recursiveFrame851Cm,stage{j}.recursiveTotalMsMedian,stage{j}.recursiveRegistrationMsMedian);
    end
    results=cell2table(rows,VariableNames={'run','scenario','rmseAllCm','rmseCm','medianCm','p95Cm','maxCm','above30cm','headingRmseDeg','runtimeSeconds'});
    disp(results(:,1:9));writetable(results,fullfile(dest,'scenario_comparison.csv'));
    writetable(struct2table([stage{:}]),fullfile(dest,'matching_stage_comparison.csv'));
end
