function diagnoseLidarOnly()
% diagnoseLidarOnly Separate seed drift from per-frame matching limits without GNSS.
% The reference-seeded solve is an evaluation-only ceiling, never a method input.
    setupVehicleLocalization;dest=fileparts(mfilename('fullpath'));
    E=load('output/localization_evaluation_20260923/observer/experiment.mat','runs','reference');
    C=load('output/localization_evaluation_20260923/sources.mat','fixed','sources');
    ref=E.reference;n=size(ref,1);est=E.runs{2}.estimate;assert(E.runs{2}.scenario=="lidar_only");
    cfg=distributionRegistrationConfig();
    r=est.matchingResults;seeds=est.matchingSeeds;
    closed=cell2mat(cellfun(@(x)x.poseXYTheta,r,UniformOutput=false));accepted=cellfun(@(x)x.accepted,r);
    oracle=zeros(n,3);oracleAccepted=false(n,1);
    for k=1:n
        o=matchLocalProbabilityCloud(C.fixed,C.sources{k},ref(k,:),cfg,[]);
        oracle(k,:)=o.poseXYTheta;oracleAccepted(k)=o.accepted;
    end
    err=@(p)vecnorm(p(:,1:2)-ref(:,1:2),2,2);
    yawErr=@(p)rad2deg(atan2(sin(p(:,3)-ref(:,3)),cos(p(:,3)-ref(:,3))));
    t=est.time-est.time(1);
    T=table(t,err(est.pose),err(seeds),err(closed),accepted,err(oracle),oracleAccepted,yawErr(est.pose),yawErr(oracle), ...
        VariableNames={'time','fusedM','seedM','matchM','matchAccepted','oracleMatchM','oracleAccepted','fusedYawDeg','oracleYawDeg'});
    writetable(T,fullfile(dest,'diagnosis_frames.csv'));
    post=t>=2;a=accepted&post;o=oracleAccepted&post;
    fprintf('LiDAR-only fused: RMSE %.2f cm, P95 %.2f, max %.2f\n',100*rms(T.fusedM(post)),100*prctile(T.fusedM(post),95),100*max(T.fusedM(post)));
    fprintf('Closed-loop seeds: RMSE %.2f cm\n',100*rms(T.seedM(post)));
    fprintf('Closed-loop match (accepted %d): RMSE %.2f, median %.2f, P95 %.2f, max %.2f, >30cm %d\n',nnz(a), ...
        100*rms(T.matchM(a)),100*median(T.matchM(a)),100*prctile(T.matchM(a),95),100*max(T.matchM(a)),nnz(T.matchM(a)>.3));
    fprintf('Reference-seeded match (accepted %d): RMSE %.2f, median %.2f, P95 %.2f, max %.2f, >30cm %d, yaw RMSE %.3f\n',nnz(o), ...
        100*rms(T.oracleMatchM(o)),100*median(T.oracleMatchM(o)),100*prctile(T.oracleMatchM(o),95),100*max(T.oracleMatchM(o)), ...
        nnz(T.oracleMatchM(o)>.3),rms(T.oracleYawDeg(o)));
    % Segments where the fused error exceeds 30 cm.
    bad=post & T.fusedM>.3;d=diff([0;bad;0]);s=find(d==1);e=find(d==-1)-1;
    for i=1:numel(s)
        k=s(i):e(i);[~,j]=max(T.fusedM(k));
        fprintf('segment %.1f-%.1f s (frames %d-%d): fused max %.1f cm, closed match median %.1f cm, oracle match median %.1f cm\n', ...
            t(s(i)),t(e(i)),s(i),e(i),100*T.fusedM(k(j)),100*median(T.matchM(k)),100*median(T.oracleMatchM(k)));
    end
    save('output/lidar_only_20260923/diagnosis.mat','T','oracle','closed','seeds','-v7.3');
end
