function report=runInspvaMotionGapComparison(outputFolder)
% runInspvaMotionGapComparison Test motion-informed gap inputs at fixed gains.
% This supplemental experiment follows the identified straight-chord failure.
% All source measurements, information, evaluation times and gains are fixed.
% The new interpolation uses future accepted endpoints offline, with no
% reference access. The INSPVA reference enters only scoring below.
    arguments
        outputFolder (1,1) string="output/mncav_inspva_observer_20260915"
    end
    setupVehicleLocalization();saved=load(fullfile(outputFolder,'experiment.mat'),'experiments','cfg','report');
    frames=readtable(fullfile(outputFolder,'frame_errors.csv'),TextType="string");
    rows=cell(0,11);cases=cell(3,1);experiments=cell(3,1);frameRows=cell(3,1);
    for j=1:3
        e=saved.experiments{j};mode=saved.report.cases(j).mode;
        f=frames(frames.mode==mode,:);accepted=f.fullMeasurement==1;
        frameTime=e.data.highRate.time(e.frameIndices);
        assert(max(abs(frameTime-f.time))<1e-9,'Frame CSV must retain the same capture clock.');
        [data,reconstruction]=reconstructMotionAidedLidarGaps(e.data,e.lateralInput,frameTime(accepted));
        reconstruction.maximumFrameCsvTimeDifferenceSeconds=max(abs(frameTime-f.time));
        estimate=runMotionAidedVehicleObserver(data,e.lateralInput,saved.cfg);
        t=data.highRate.time;ix=e.uniformIndices;native=e.frameIndices;
        populations={"native_full",native(accepted),e.frameReference(accepted,:); ...
            "native_all",native,e.frameReference;"uniform_all",ix,e.reference(ix,:); ...
            "uniform_first60",ix(t(ix)<=60),e.reference(ix(t(ix)<=60),:); ...
            "uniform_after60",ix(t(ix)>60),e.reference(ix(t(ix)>60),:); ...
            "native_without_full_measurement",native(~accepted),e.frameReference(~accepted,:)};
        for k=1:size(populations,1)
            indices=populations{k,2};ref=populations{k,3};
            variants={"motion_gap_reconstruction",data.lidar.pose(indices,:);"observer_with_motion_gaps",estimate.pose(indices,:)};
            for m=1:2
                error=vecnorm(variants{m,2}(:,1:2)-ref(:,1:2),2,2);
                yaw=atan2(sin(variants{m,2}(:,3)-ref(:,3)),cos(variants{m,2}(:,3)-ref(:,3)));
                values=[rms(error),median(error),prctile(error,95),max(error),mean(error<=.05),mean(error<=.1),rad2deg(rms(yaw))];
                rows(end+1,:)=[{mode,populations{k,1},variants{m,1},numel(indices)},num2cell(values)]; %#ok<AGROW>
            end
        end
        numerical=struct();ablation=struct();
        if j==1
            repeated=runMotionAidedVehicleObserver(data,e.lateralInput,saved.cfg);
            refinedCfg=saved.cfg;refinedCfg.maximumIntegrationStep=saved.cfg.maximumIntegrationStep/2;
            refined=runMotionAidedVehicleObserver(data,e.lateralInput,refinedCfg);
            numerical.repeatedStateDifference=max(abs(repeated.z-estimate.z),[],'all');
            numerical.halfStepMaximumPositionDifferenceM=max(vecnorm(refined.position-estimate.position,2,2));
            assert(numerical.repeatedStateDifference==0 && numerical.halfStepMaximumPositionDifferenceM<1e-5);
            zeroLateral=e.lateralInput;
            for name=["lateralVelocity","sideSlipAngle","sideSlipAngleRate"],zeroLateral.(name)(:)=0;end
            zeroData=reconstructMotionAidedLidarGaps(e.data,zeroLateral,frameTime(accepted));
            zeroEstimate=runMotionAidedVehicleObserver(zeroData,zeroLateral,saved.cfg);
            nativeError=vecnorm(zeroEstimate.position(native(accepted),:)-e.frameReference(accepted,1:2),2,2);
            ablation=struct('change',"Zero lateral estimates in both reconstruction and observer; all other inputs and gains fixed", ...
                'uniformZeroLateralRmseM',rms(vecnorm(zeroEstimate.position(ix,:)-e.reference(ix,1:2),2,2)), ...
                'nativeFullZeroLateralRmseM',rms(nativeError),'nativeFullZeroLateralMedianM',median(nativeError), ...
                'parametersSelectedFromThisAblation',false);
        end
        cases{j}=struct('mode',mode,'reconstruction',reconstruction,'numericalChecks',numerical, ...
            'diagnostics',estimate.diagnostics,'lateralAblation',ablation);
        experiments{j}=struct('data',data,'estimate',estimate);
        frameRows{j}=table(f.frame,f.time,repmat(mode,height(f),1),accepted, ...
            vecnorm(data.lidar.pose(native,1:2)-e.frameReference(:,1:2),2,2), ...
            vecnorm(estimate.position(native,:)-e.frameReference(:,1:2),2,2), ...
            VariableNames={'frame','time','mode','fullMeasurement','motionGapReconstructionErrorM','observerWithMotionGapsErrorM'});
    end
    metrics=[saved.report.metrics;cell2table(rows,VariableNames=saved.report.metrics.Properties.VariableNames)];
    metadata=struct('globalGains',saved.cfg.gains,'gainsRetuned',false,'gapThresholdSeconds',.25, ...
        'referenceUsedForReconstructionOrObserver',false,'matchingRerun',false,'futureEndpointUsed',true, ...
        'scope',"Post-diagnostic offline reconstruction change; same-drive reference-seeded measurements; no independent or causal validation");
    report=struct('metadata',metadata,'cases',vertcat(cases{:}),'metrics',metrics);
    writetable(metrics,fullfile(outputFolder,'motion_gap_metrics.csv'));
    writetable(vertcat(frameRows{:}),fullfile(outputFolder,'motion_gap_frame_errors.csv'));
    save(fullfile(outputFolder,'motion_gap_experiment.mat'),'report','experiments','-v7.3');
    save(fullfile(outputFolder,'motion_gap_ablation.mat'),'zeroData','zeroEstimate','-v7.3');
    fid=fopen(fullfile(outputFolder,'motion_gap_summary.json'),'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));
    e=saved.experiments{1};ix=e.uniformIndices;t=e.data.highRate.time(ix);ref=e.reference(ix,1:2);
    errors=[vecnorm(e.data.lidar.pose(ix,1:2)-ref,2,2),vecnorm(e.estimate.position(ix,:)-ref,2,2), ...
        vecnorm(experiments{1}.data.lidar.pose(ix,1:2)-ref,2,2),vecnorm(experiments{1}.estimate.position(ix,:)-ref,2,2)];
    writetable(array2table([t,errors],VariableNames={'time','lidarLinearErrorM','originalObserverErrorM', ...
        'motionReconstructionErrorM','motionGapObserverErrorM'}),fullfile(outputFolder,'motion_gap_uniform_errors.csv'));
    fig=figure('Color','w','Position',[150,150,1250,750]);tiledlayout(fig,2,1,TileSpacing='compact');
    nexttile;curves=plot(t,errors(:,[1,2,4]));palette=lines(4);curves(3).Color=palette(4,:);
    grid on;xlabel('Receiver time (s)');ylabel('Position discrepancy (m)');
    title('INSPVA-map replay: identical measurements and fixed observer gains');
    label=legend('LiDAR interpolation','Observer + linear gaps','Observer + motion gaps',FontSize=10);
    label.Units='normalized';label.Position=[.65,.78,.29,.13];
    nexttile;plot(t,errors);xlim([79,85]);grid on;xlabel('Receiver time (s)');ylabel('Position discrepancy (m)');
    title('Offline gap reconstruction from measured motion and both LiDAR endpoints');
    label=legend('LiDAR interpolation','Observer + linear gaps','Motion interpolation','Observer + motion gaps',FontSize=10);
    label.Units='normalized';label.Position=[.65,.29,.29,.16];
    exportgraphics(fig,fullfile(outputFolder,'motion_gap_comparison.png'),'Resolution',160);
    exportgraphics(fig,fullfile(outputFolder,'motion_gap_comparison.pdf'),'ContentType','vector');
    disp(metrics(metrics.mode=="per_frame_zero" & (metrics.population=="native_full"|metrics.population=="uniform_all"),:));
end
