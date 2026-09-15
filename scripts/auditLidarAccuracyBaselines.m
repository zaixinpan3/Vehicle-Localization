function report=auditLidarAccuracyBaselines(outputFolder)
% auditLidarAccuracyBaselines Distinguish historical centimeter metrics.
% Recompute stored matching statistics and compare the current actual input
% and output distributions on identical times and the same INSPVA reference.
% No matching, observer tuning, reference alignment or sample rejection runs.
    arguments
        outputFolder (1,1) string="output/mncav_baseline_audit_20260914"
    end
    setupVehicleLocalization;if ~isfolder(outputFolder),mkdir(outputFolder);end
    geometric=readtable('research/results/geometric_registration_20260905/registration.csv');
    identity=string(geometric.calibration)=="identity" & string(geometric.variant)=="geometricXY";
    accepted=identity & geometric.accepted==1;
    saved=readtable('research/results/saved_perception_matching_20260914/cases.csv');
    additional=readtable('research/matching_bias_20260914/additional_baseline.csv');
    additionalAccepted=ismember(lower(string(additional.accepted)),["1","true"]);
    previous=readtable('research/results/mississippi_full_sequence_20260907/d2d_calls.csv');
    current=readtable('output/mncav_zero_delay_20260914/precomputed_calls.csv');
    historicalRows={
        "geometric_20260905_frame900_start1","one accepted frame",geometric.translationDifferenceM(identity & geometric.frame==900 & geometric.start==1);
        "geometric_20260905_identity","18 accepted trials from 7 query frames / 3 starts",geometric.translationDifferenceM(accepted);
        "saved_fine_20260914_original","20 accepted trials from 7 query frames / 3 starts",saved.xy_difference_m(string(saved.decision)=="accepted");
        "saved_fine_20260914_additional","103 full accepted trials from 38 query frames / 3 starts",additional.distance(additionalAccepted & string(additional.reason)=="accepted");
        "recursive_20260907","all 1170 frames including prediction on rejection",previous.positionErrorM;
        "recursive_20260914_lateral_aided","all 1170 frames including prediction on rejection",current.positionErrorM};
    rows=cell(size(historicalRows,1),6);
    for k=1:size(historicalRows,1)
        e=historicalRows{k,3};assert(~isempty(e) && all(isfinite(e)) && all(e>=0));
        rows(k,:)=[historicalRows(k,1:2),{numel(e),median(e),rms(e),max(e)}];
    end
    history=cell2table(rows,VariableNames={'source','population','samples','medianM','rmseM','maximumM'});
    base=load('output/mncav_zero_delay_20260914/experiment.mat','data','pvaPose','uniformIndices');
    motion=load('output/mncav_motion_aided_20260914/experiment.mat','estimate');
    assert(isequal(motion.estimate.time(:),base.data.highRate.time(:)));
    ix=base.uniformIndices;time=base.data.highRate.time(ix);reference=base.pvaPose(ix,1:2);
    raw=vecnorm(base.data.lidar.pose(ix,1:2)-reference,2,2);
    fused=vecnorm(motion.estimate.pose(ix,1:2)-reference,2,2);
    assert(numel(unique(ix))==numel(ix) && all(diff(time)>0));
    assert(all(isfinite(raw)) && all(isfinite(fused)));
    values=zeros(2,11);
    for k=1:2
        e=raw;if k==2,e=fused;end
        ordered=sort(e);n=numel(e);tail=ceil(.05*n);
        values(k,:)=[n,mean(e),median(e),rms(e),prctile(e,95),max(e), ...
            mean(e<=.05),mean(e<=.1),mean(e>=.3), ...
            sum(ordered(end-tail+1:end).^2)/sum(e.^2),rms(ordered(1:end-tail))];
    end
    distribution=array2table(values,VariableNames={'samples','meanM','medianM','rmseM','p95M', ...
        'maximumM','fractionAtMost5cm','fractionAtMost10cm','fractionAtLeast30cm', ...
        'largest5PercentSquaredErrorShare','lower95PercentDiagnosticRmseM'});
    distribution=addvars(distribution,["actual_continuous_lidar";"motion_aided_observer"],Before=1,NewVariableNames='method');
    assert(abs(distribution.rmseM(1)-.18846470284125785)<1e-12);
    assert(abs(distribution.rmseM(2)-.176934281205468)<1e-12);
    summary=struct('sameTimeReference',true,'reference',"INSPVA diagnostic, not independent truth", ...
        'samples',numel(ix),'rawMedianCm',100*median(raw),'observerMedianCm',100*median(fused), ...
        'rawRmseCm',100*rms(raw),'observerRmseCm',100*rms(fused), ...
        'rawFractionAtMost5cm',mean(raw<=.05),'observerFractionAtMost5cm',mean(fused<=.05), ...
        'pairedPositionImprovementFraction',mean(fused<raw), ...
        'medianImproved',median(fused)<=median(raw), ...
        'fractionWithin5cmImproved',mean(fused<=.05)>=mean(raw<=.05), ...
        'allFrameFewCentimeterLidarRmseLocated',false, ...
        'scope',"Reviewed identified local result tables and the current conversation's user-facing history. Centimeter claims include synthetic observer RMSE, reference-injection RMSE and a virtual noise scale; which one the user recalled remains unconfirmed.", ...
        'tailDiagnostic',"Largest ceil(5%*N) errors only characterize squared-error concentration. Full metrics retain all samples; lower95% is not an accepted benchmark.");
    report=struct('summary',summary,'historical',history,'distribution',distribution);
    writetable(history,fullfile(outputFolder,'historical_baselines.csv'));
    writetable(distribution,fullfile(outputFolder,'current_distribution.csv'));
    fid=fopen(fullfile(outputFolder,'summary.json'),'w');assert(fid>=0);
    fprintf(fid,'%s\n',jsonencode(summary,PrettyPrint=true));fclose(fid);
    save(fullfile(outputFolder,'audit.mat'),'report','time','raw','fused','-v7.3');
    fig=figure('Color','w','Name','Actual LiDAR and observer error distributions');
    plot(100*sort(raw),(1:numel(raw))/numel(raw),'LineWidth',1.3);hold on;
    plot(100*sort(fused),(1:numel(fused))/numel(fused),'LineWidth',1.3);xline(5,'--');xline(10,':');
    xlabel('Position discrepancy from INSPVA (cm)');ylabel('Fraction of uniform samples');grid on;
    xlim([0,50]);ylim([0,1]);legend('Actual LiDAR input','Motion-aided observer','5 cm','10 cm',Location='southeast');
    exportgraphics(fig,fullfile(outputFolder,'error_cdf.png'),'Resolution',160);
    exportgraphics(fig,fullfile(outputFolder,'error_cdf.pdf'),'ContentType','vector');
    disp(history);disp(distribution);
end
