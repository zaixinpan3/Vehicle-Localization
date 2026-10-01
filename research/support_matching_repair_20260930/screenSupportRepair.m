function screenSupportRepair()
% screenSupportRepair Test real-frame basins before a causal route replay.
% Re-execution evaluates the committed model. Preserve the historical screen
% tables, which were generated while the implementation was still evolving.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    a=load('output/source_shape_matching_20260929/shape50.mat','sources');
    b=load(featureMapBuildConfig().probabilityCloudPath,'cloud');
    old=load('output/line_direction_matching_20260928/production/report.mat','report');
    baseline=readtable('research/source_shape_matching_20260929/final_raw.csv');rows=cell(0,7);
    for power=[1 2 3]
        for angular=[.10 .25 .40]
            cfg=supportRegistrationConfig();cfg.support.slidingPower=power;cfg.support.scatterScale=angular;
            for k=[178 894 932]
                seed=baseline{k,{'x','y','psi'}};ref=old.report.calls{k,{'referenceX','referenceY','referencePsi'}};
                result=matchLocalProbabilityCloud(b.cloud,a.sources{k},seed,cfg);
                rows(end+1,:)={power,angular,k,norm(result.poseXYTheta(1:2)-ref(1:2)), ...
                    rad2deg(atan2(sin(result.poseXYTheta(3)-ref(3)),cos(result.poseXYTheta(3)-ref(3)))),result.observableRank,result.reason}; %#ok<AGROW>
            end
        end
    end
    report=cell2table(rows,VariableNames={'power','scatterScale','frame','errorM','yawErrorDeg','rank','reason'});
    writetable(report,fullfile(dest,'current_screen.csv'));disp(report);
end
