function results=runLidarOnlyGraph()
% runLidarOnlyGraph Switchable sliding-window pose graph with no GNSS factors.
% Compares the recorded wheel/gyro/CG-lateral odometry with odometry whose
% lateral velocity is transported to the output point by the separate-drive
% lever arm. Reference poses are used for evaluation only.
    setupVehicleLocalization;dest=fileparts(mfilename('fullpath'));
    E=load('output/localization_evaluation_20260923/observer/experiment.mat','data','lateral','cfg','reference');
    clouds=load('output/localization_evaluation_20260923/sources.mat');
    lever=jsondecode(fileread(fullfile(dest,'lever_arm_calibration.json')));
    replay=load('output/temporal_perception_20260922/five_frame_matching/report.mat','report');
    data=rmfield(E.data,{'lidar','gnss'});h=data.highRate;t=h.time;n=numel(t);ref=E.reference;
    recorded=replay.report.deadReckoning{1:n,{'x','y','psi'}};
    transported=integrateMotion(t,h.longitudinalSpeed,E.lateral.lateralVelocity+lever.leverArmM*h.yawRate,h.yawRate,recorded(1,:));
    first=clouds.calls{1,{'predictedX','predictedY','predictedPsi'}};
    cfg=robustPoseGraphConfig();post=t>=t(1)+2;
    motions={"graph_recorded_odometry",recorded;"graph_lever_odometry",transported};rows=cell(2,8);
    for j=1:2
        est=runRobustGraphLocalization(data,E.lateral,clouds,clouds.fixed,motions{j,2},first,cfg,E.cfg);
        e=vecnorm(est.pose(:,1:2)-ref(:,1:2),2,2);
        yaw=rad2deg(atan2(sin(est.pose(:,3)-ref(:,3)),cos(est.pose(:,3)-ref(:,3))));
        odo=relativeDrift(motions{j,2},ref);
        rows(j,:)={motions{j,1},100*rms(e(post)),100*median(e(post)),100*prctile(e(post),95),100*max(e(post)), ...
            nnz(e(post)>.3),rms(yaw(post)),odo};
        fprintf('%-26s RMSE %.2f cm, P95 %.2f, max %.2f, >30cm %d, heading %.3f deg, odometry 1 s drift RMSE %.2f cm\n',rows{j,:});
    end
    results=cell2table(rows,VariableNames={'variant','rmseCm','medianCm','p95Cm','maxCm','fusedAbove30cm', ...
        'headingRmseDeg','odometryOneSecondDriftRmseCm'});
    writetable(results,fullfile(dest,'graph_variants.csv'));
end

function pose=integrateMotion(t,vx,vy,r,start)
    pose=zeros(numel(t),3);pose(1,:)=start;
    for k=2:numel(t)
        dt=t(k)-t(k-1);yaw=pose(k-1,3)+dt*(r(k-1)+r(k))/2;mid=(pose(k-1,3)+yaw)/2;
        v=[(vx(k-1)+vx(k))/2,(vy(k-1)+vy(k))/2];
        pose(k,:)=[pose(k-1,1:2)+dt*[cos(mid)*v(1)-sin(mid)*v(2),sin(mid)*v(1)+cos(mid)*v(2)],yaw];
    end
end

function value=relativeDrift(motion,ref)
% RMS position error of 10-frame relative motion expressed in the start frame.
    k=(1:size(ref,1)-10).';e=zeros(numel(k),1);
    for i=k.'
        R=@(y)[cos(y) sin(y);-sin(y) cos(y)];
        a=(motion(i+10,1:2)-motion(i,1:2))*R(motion(i,3)).';b=(ref(i+10,1:2)-ref(i,1:2))*R(ref(i,3)).';
        e(i)=norm(a-b);
    end
    value=100*rms(e);
end
