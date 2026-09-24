function results=checkFusedMode()
% checkFusedMode Evaluate the LiDAR-only remedies with GNSS available.
    setupVehicleLocalization;dest=fileparts(mfilename('fullpath'));
    E=load('output/localization_evaluation_20260923/observer/experiment.mat','data','lateral','lateralDesign','cfg','reference');
    C=load('output/localization_evaluation_20260923/sources.mat','fixed','sources');
    lever=jsondecode(fileread(fullfile(dest,'lever_arm_calibration.json')));
    base=rmfield(E.data,'lidar');ref=E.reference;t=base.highRate.time;h=base.highRate;post=t>=t(1)+2;
    transported=E.lateral;vx=max(h.longitudinalSpeed,1);vyPoint=E.lateral.lateralVelocity+lever.leverArmM*h.yawRate;
    delta=atan2(vyPoint,vx)-atan2(E.lateral.lateralVelocity,vx);transported.lateralVelocity=vyPoint;
    transported.sideSlipAngleRate=E.lateral.sideSlipAngleRate+[0;diff(delta)./diff(t)];
    variants={"production",E.lateral,16,5,false;"lever_bias2_gain",transported,4,2,false; ...
        "lever_bias2_gain_height",transported,4,2,true};rows=cell(3,7);
    for j=1:3
        cfg=E.cfg;cfg.lidar.gainInformationScale=variants{j,3};cfg.bias.minimumSpeed=variants{j,4};
        regCfg=distributionRegistrationConfig();regCfg.relativeHeight.enabled=variants{j,5};
        data=base;data.lidarMatcher=@(k,seed,aid) matchLocalProbabilityCloud(C.fixed,C.sources{k},seed,regCfg,aid);
        est=runFullLocalizationObserver(data,E.lateralDesign,cfg,LateralInputs=variants{j,2});
        e=vecnorm(est.position-ref(:,1:2),2,2);yaw=rad2deg(atan2(sin(est.pose(:,3)-ref(:,3)),cos(est.pose(:,3)-ref(:,3))));
        rows(j,:)={variants{j,1},100*rms(e),100*rms(e(post)),100*prctile(e(post),95),100*max(e(post)),nnz(e(post)>.3),rms(yaw(post))};
        fprintf('%-26s all %.2f cm, after 2 s %.2f, P95 %.2f, max %.2f, >30cm %d, heading %.3f deg\n',rows{j,:});
    end
    results=cell2table(rows,VariableNames={'variant','rmseAllCm','rmseCm','p95Cm','maxCm','fusedAbove30cm','headingRmseDeg'});
    writetable(results,fullfile(dest,'fused_mode_check.csv'));
end
