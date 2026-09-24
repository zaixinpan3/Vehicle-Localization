function results=compareOutputPointTransport()
% compareOutputPointTransport Compare the production chain before and after the transport.
% Both runs use fresh coarse perception, closed-loop GNSS-aided rematching and
% the same map, gains and calibration; only the lateral-velocity output point
% differs. Reference poses are used for evaluation only.
    setupVehicleLocalization;dest=fileparts(mfilename('fullpath'));
    runs={"before (observer point)",'output/localization_evaluation_20260923/observer/experiment.mat', ...
        'output/temporal_perception_20260922/five_frame_matching/report.mat'; ...
        "after (output point)",'output/mncav_coarse_localization_20260924/observer/experiment.mat', ...
        'output/mncav_coarse_localization_20260924/matching/report.mat'};
    rows=cell(0,10);odometry=cell(2,1);
    for j=1:2
        E=load(runs{j,2},'runs','reference','lateral','data');ref=E.reference;t=E.runs{1}.estimate.time;post=t>=t(1)+2;
        for k=1:numel(E.runs)
            est=E.runs{k}.estimate;e=vecnorm(est.position-ref(:,1:2),2,2);
            yaw=rad2deg(atan2(sin(est.pose(:,3)-ref(:,3)),cos(est.pose(:,3)-ref(:,3))));
            rows(end+1,:)={runs{j,1},E.runs{k}.scenario,100*rms(e),100*rms(e(post)),100*median(e(post)), ...
                100*prctile(e(post),95),100*max(e(post)),nnz(e(post)>.3),rms(yaw(post)),E.runs{k}.seconds}; %#ok<AGROW>
        end
        % Raw matching inside the fused run and the source-window odometry drift.
        r=E.runs{1}.estimate.matchingResults;a=cellfun(@(x)x.accepted,r);
        p=cell2mat(cellfun(@(x)x.poseXYTheta,r,UniformOutput=false));em=vecnorm(p(:,1:2)-ref(:,1:2),2,2);
        M=load(runs{j,3},'report');motion=M.report.deadReckoning{:,{'x','y','psi'}};n=size(ref,1);
        drift=zeros(n-5,1);
        for i=1:n-5
            R=@(y)[cos(y) sin(y);-sin(y) cos(y)];
            drift(i)=norm((motion(i+5,1:2)-motion(i,1:2))*R(motion(i,3)).'-(ref(i+5,1:2)-ref(i,1:2))*R(ref(i,3)).');
        end
        v=[0 0;diff(ref(:,1:2))./diff(t)];vyRef=-sin(ref(:,3)).*v(:,1)+cos(ref(:,3)).*v(:,2);
        odometry{j}=struct('label',runs{j,1},'acceptedMatches',nnz(a),'matchingRmseCm',100*rms(em(a)), ...
            'matchingMaxCm',100*max(em(a)),'matchingAbove30cm',nnz(em(a)>.3), ...
            'halfSecondOdometryDriftRmsCm',100*rms(drift),'lateralVelocityRmseMps',rms(E.lateral.lateralVelocity(post)-vyRef(post)));
        fprintf('%s: matches %d, matching RMSE %.2f cm (max %.2f, >30cm %d), 0.5 s odometry drift %.2f cm, vy RMSE %.3f m/s\n', ...
            runs{j,1},odometry{j}.acceptedMatches,odometry{j}.matchingRmseCm,odometry{j}.matchingMaxCm, ...
            odometry{j}.matchingAbove30cm,odometry{j}.halfSecondOdometryDriftRmsCm,odometry{j}.lateralVelocityRmseMps);
    end
    results=cell2table(rows,VariableNames={'run','scenario','rmseAllCm','rmseCm','medianCm','p95Cm','maxCm', ...
        'above30cm','headingRmseDeg','runtimeSeconds'});
    disp(results(:,1:9));writetable(results,fullfile(dest,'scenario_comparison.csv'));
    writetable(struct2table([odometry{:}]),fullfile(dest,'matching_and_odometry.csv'));
end
