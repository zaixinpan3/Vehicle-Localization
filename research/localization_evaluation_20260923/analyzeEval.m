addpath(pwd);setupVehicleLocalization;
out='output/localization_evaluation_20260923';
now_=load(fullfile(out,'observer','experiment.mat'),'runs','reference','report');
old=load('output/gnss_aided_matching_20260922/production_fresh/experiment.mat','runs');
ref=now_.reference;n=size(ref,1);
% 1) Reproducibility against the last recorded production replay
for k=1:numel(now_.runs)
    a=now_.runs{k}.estimate.pose;b=old.runs{k}.estimate.pose;
    fprintf('REPRO %-13s max|dpose| = %.3g\n',now_.runs{k}.scenario,max(abs(a-b),[],'all'));
end
src=load(fullfile(out,'sources.mat'),'sources');prev=load('output/height_association_20260923/sources.mat','sources');
same=cellfun(@(a,b)isequaln(a.components,b.components),src.sources,prev.sources);
fprintf('SOURCES identical XY components: %d / %d\n',nnz(same),numel(same));
% 2) LiDAR matching measurement quality inside the fused run
r=now_.runs{1}.estimate.matchingResults;
pose=cell2mat(cellfun(@(x)x.poseXYTheta,r,UniformOutput=false));acc=cellfun(@(x)x.accepted,r);
sec=cellfun(@(x)x.matchingSeconds,r(acc));
e=vecnorm(pose(:,1:2)-ref(:,1:2),2,2);h=rad2deg(abs(atan2(sin(pose(:,3)-ref(:,3)),cos(pose(:,3)-ref(:,3)))));
ea=e(acc);[mx,im]=max(ea);ids=find(acc);
fprintf('MATCH accepted %d/%d  RMSE %.2f cm  median %.2f  P95 %.2f  P99 %.2f  max %.2f cm (idx %d)  >30cm %d  heading RMSE %.3f deg\n', ...
    nnz(acc),n,100*rms(ea),100*median(ea),100*prctile(ea,95),100*prctile(ea,99),100*mx,ids(im),nnz(ea>.3),rms(h(acc)));
fprintf('MATCHTIME median %.1f ms  P95 %.1f ms  max %.1f ms\n',1e3*median(sec),1e3*prctile(sec,95),1e3*max(sec));
rej=find(~acc);fprintf('REJECTED idx: %s\n',mat2str(rej.'));
% 3) Fused errors per scenario, full and after 2 s
t=now_.runs{1}.estimate.time;rows=cell(0,9);
for k=1:numel(now_.runs)
    p=now_.runs{k}.estimate.pose;ef=vecnorm(p(:,1:2)-ref(:,1:2),2,2);
    eh=rad2deg(atan2(sin(p(:,3)-ref(:,3)),cos(p(:,3)-ref(:,3))));post=t>=t(1)+2;
    rows(end+1,:)={now_.runs{k}.scenario,100*rms(ef),100*median(ef),100*prctile(ef,95),100*max(ef), ...
        100*rms(ef(post)),100*max(ef(post)),rms(eh),rms(eh(post))}; %#ok<AGROW>
end
T=cell2table(rows,VariableNames={'scenario','rmseCm','medianCm','p95Cm','maxCm','rmseAfter2sCm','maxAfter2sCm','headingRmseDeg','headingRmseAfter2sDeg'});
disp(T);writetable(T,fullfile(out,'scenario_summary.csv'));
m=table(e,acc,h,VariableNames={'matchErrorM','accepted','headingErrorDeg'});
m.fusedErrorM=vecnorm(now_.runs{1}.estimate.position-ref(:,1:2),2,2);m.time=t;
writetable(m,fullfile(out,'frame_errors.csv'));
