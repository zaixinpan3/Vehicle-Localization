root=fileparts(fileparts(fileparts(mfilename('fullpath'))));cd(root);setupVehicleLocalization;
out=fullfile(root,'output/coarse_fusion_diagnosis_20260918');
s=load('output/mncav_coarse_localization_20260918/observer/experiment.mat');
names=["current","quarter_lidar_position_gain","disable_lidar_velocity_bias"];
rows=cell(3,7);runs=cell(3,1);
for k=1:3
 cfg=s.cfg;
 if k==2,cfg.gains(1)=1;end
 if k==3,cfg.bias.enabled=false;end
 r=runFullLocalizationObserver(s.data,s.lateralDesign,cfg,LateralInputs=s.lateral);runs{k}=r;
 error=vecnorm(r.position-s.reference(:,1:2),2,2);angle=atan2(sin(r.heading-s.reference(:,3)),cos(r.heading-s.reference(:,3)));
 rows(k,:)={names(k),rms(error),median(error),prctile(error,95),rad2deg(rms(angle)),max(abs(r.heading-runs{1}.heading)),max(abs(r.velocity-runs{1}.velocity),[],'all')};
end
assert(rows{2,6}==0 && rows{2,7}==0,'Position-gain control must preserve heading and velocity.');
metrics=cell2table(rows,VariableNames={'variant','positionRmseM','positionMedianM','positionP95M','headingRmseDeg','maximumHeadingChangeRad','maximumVelocityChangeMps'});
writetable(metrics,fullfile(out,'ablation.csv'));save(fullfile(out,'ablation.mat'),'runs','metrics');disp(metrics);
