function exportValidation()
% exportValidation Export reproducible observer traces and focused ablations.
    setupVehicleLocalization();
    folder='output/matching_refinement_20260919';
    s=load(fullfile(folder,'observer_final/experiment.mat'));
    destination='research/matching_refinement_20260919';
    copyfile(fullfile(folder,'observer_final/metrics.csv'),fullfile(destination,'observer_metrics.csv'));
    for k=1:numel(s.runs)
        r=s.runs{k};e=r.estimate;
        values=table(e.time,e.pose(:,1),e.pose(:,2),e.pose(:,3), ...
            s.reference(:,1),s.reference(:,2),s.reference(:,3), ...
            e.diagnostics.mode,e.diagnostics.lidarLongitudinalVelocityBias,e.diagnostics.lidarVelocityBias, ...
            VariableNames={'time','x','y','psi','referenceX','referenceY','referencePsi', ...
            'mode','longitudinalBias','lateralBias'});
        writetable(values,fullfile(folder,'observer_final',r.scenario+"_poses.csv"));
    end
    % The same matching measurements and initial state isolate gain choice.
    cfg=s.cfg;cfg.lidar.gainInformationScale=.001;
    r=runFullLocalizationObserver(s.data,s.lateralDesign,cfg,LateralInputs=s.lateral);
    e=vecnorm(r.position-s.reference(:,1:2),2,2);
    ablation=table("almost_saturated_lidar_gain",rms(e),prctile(e,95),max(e), ...
        VariableNames={'variant','positionRmseM','positionP95M','positionMaximumM'});
    old=readtable(fullfile(folder,'validated_observer/metrics.csv'),TextType='string');
    old=old(old.population=="full" & old.scenario=="both_outage",:);
    ablation(end+1,:)={"lateral_bias_only_both_outage",old.positionRmseM,old.positionP95M,old.positionMaximumM};
    writetable(ablation,fullfile(destination,'observer_ablation.csv'));
    disp(ablation);
end
