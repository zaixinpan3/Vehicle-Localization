function results=runLidarOnlyVariants()
% runLidarOnlyVariants LiDAR-only closed-loop observer variants without any GNSS input.
% Methods use only motion, lateral observer, map, sources and the lever arm
% calibrated on the separate 12-11-24 drive. Variants marked "diagnostic"
% substitute reference lateral velocity and bound the attainable benefit.
    setupVehicleLocalization;dest=fileparts(mfilename('fullpath'));
    E=load('output/localization_evaluation_20260923/observer/experiment.mat','data','lateral','lateralDesign','cfg','reference');
    C=load('output/localization_evaluation_20260923/sources.mat','fixed','sources');
    lever=jsondecode(fileread(fullfile(dest,'lever_arm_calibration.json')));
    base=rmfield(E.data,{'lidar','gnss'});ref=E.reference;t=base.highRate.time;h=base.highRate;
    transported=transportLateral(E.lateral,h,t,E.lateral.lateralVelocity+lever.leverArmM*h.yawRate);
    % Evaluation-only reference lateral velocity at the output point.
    v=zeros(numel(t),2);v(2:end-1,:)=(ref(3:end,1:2)-ref(1:end-2,1:2))./(t(3:end)-t(1:end-2));
    v([1 end],:)=v([2 end-1],:);vyRef=-sin(ref(:,3)).*v(:,1)+cos(ref(:,3)).*v(:,2);
    oracle=transportLateral(E.lateral,h,t,vyRef);
    % Columns: name, lateral input, LiDAR gain information scale, bias
    % full-participation speed, uses reference, relative height enabled.
    variants={"baseline",E.lateral,16,5,false,false; ...
        "gain",E.lateral,4,5,false,false; ...
        "lever",transported,16,5,false,false; ...
        "lever_bias2",transported,16,2,false,false; ...
        "lever_gain",transported,4,5,false,false; ...
        "lever_bias2_gain",transported,4,2,false,false; ...
        "lever_bias2_gain_height",transported,4,2,false,true; ...
        "reference_vy (diagnostic)",oracle,16,5,true,false; ...
        "reference_vy_gain (diagnostic)",oracle,4,5,true,false};
    post=t>=t(1)+2;rows=cell(size(variants,1),12);traces=zeros(numel(t),size(variants,1));
    for j=1:size(variants,1)
        cfg=E.cfg;cfg.lidar.gainInformationScale=variants{j,3};cfg.bias.minimumSpeed=variants{j,4};
        regCfg=distributionRegistrationConfig();regCfg.relativeHeight.enabled=variants{j,6};
        data=base;data.lidarMatcher=@(k,seed,aid) matchLocalProbabilityCloud(C.fixed,C.sources{k},seed,regCfg,aid);
        est=runFullLocalizationObserver(data,E.lateralDesign,cfg,LateralInputs=variants{j,2});
        e=vecnorm(est.position-ref(:,1:2),2,2);traces(:,j)=e;
        yaw=rad2deg(atan2(sin(est.pose(:,3)-ref(:,3)),cos(est.pose(:,3)-ref(:,3))));
        p=cell2mat(cellfun(@(x)x.poseXYTheta,est.matchingResults,UniformOutput=false));
        a=cellfun(@(x)x.accepted,est.matchingResults)&post;em=vecnorm(p(:,1:2)-ref(:,1:2),2,2);
        s=vecnorm(est.matchingSeeds(:,1:2)-ref(:,1:2),2,2);
        rows(j,:)={variants{j,1},variants{j,5},100*rms(e(post)),100*median(e(post)),100*prctile(e(post),95), ...
            100*max(e(post)),nnz(e(post)>.3),rms(yaw(post)),100*rms(s(post)),nnz(a),100*rms(em(a)),nnz(em(a)>.3)};
        fprintf('%-32s fused %.2f cm (P95 %.2f, max %.2f, >30cm %d), seed %.2f, match %.2f cm (>30cm %d)\n', ...
            rows{j,1},rows{j,3},rows{j,5},rows{j,6},rows{j,7},rows{j,9},rows{j,11},rows{j,12});
    end
    results=cell2table(rows,VariableNames={'variant','usesReference','rmseCm','medianCm','p95Cm','maxCm', ...
        'fusedAbove30cm','headingRmseDeg','seedRmseCm','acceptedMatches','matchRmseCm','matchAbove30cm'});
    writetable(results,fullfile(dest,'variants.csv'));names=string(variants(:,1));
    if ~isfolder('output/lidar_only_20260923'),mkdir('output/lidar_only_20260923');end
    save('output/lidar_only_20260923/variant_traces.mat','traces','names','t','-v7.3');
end

function lateral=transportLateral(lateral,h,t,vyPoint)
% Replace lateral velocity and keep the track-angle rate consistent with it.
    vx=max(h.longitudinalSpeed,1);delta=atan2(vyPoint,vx)-atan2(lateral.lateralVelocity,vx);
    lateral.lateralVelocity=vyPoint;
    lateral.sideSlipAngleRate=lateral.sideSlipAngleRate+[0;diff(delta)./diff(t)];
end
