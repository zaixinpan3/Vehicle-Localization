function diagnostics=analyzeInspvaObserverReplay(outputFolder)
% analyzeInspvaObserverReplay Audit gap errors and a fixed lateral ablation.
% Reference-chord interpolation is a diagnostic only, never an input.
% The ablation holds gains fixed and replaces lateral estimates with zero;
% it does not select a replacement observer using the evaluation reference.
    arguments
        outputFolder (1,1) string="output/mncav_inspva_observer_20260915"
    end
    setupVehicleLocalization();saved=load(fullfile(outputFolder,'experiment.mat'),'experiments','cfg');
    e=saved.experiments{1};t=e.data.highRate.time;ix=e.uniformIndices;
    frames=readtable(fullfile(outputFolder,'frame_errors.csv'),TextType="string");
    frames=frames(frames.mode=="per_frame_zero",:);full=frames.fullMeasurement==1;
    accepted=find(full);times=frames.time(full);gaps=diff(times);
    lidarError=vecnorm(e.data.lidar.pose(ix,1:2)-e.reference(ix,1:2),2,2);
    observerError=vecnorm(e.estimate.position(ix,:)-e.reference(ix,1:2),2,2);
    selected=find(gaps>.25);rows=zeros(numel(selected),11);mask=false(numel(ix),1);
    for j=1:numel(selected)
        k=selected(j);inside=t(ix)>times(k)&t(ix)<times(k+1);mask=mask|inside;
        chord=interp1(times(k:k+1),e.frameReference(accepted(k:k+1),1:2),t(ix(inside)),'linear');
        chordError=vecnorm(chord-e.reference(ix(inside),1:2),2,2);
        rows(j,:)=[accepted(k),accepted(k+1),times(k),times(k+1),gaps(k),nnz(inside), ...
            rms(lidarError(inside)),rms(observerError(inside)),max(chordError), ...
            sum(lidarError(inside).^2)/sum(lidarError.^2),sum(observerError(inside).^2)/sum(observerError.^2)];
    end
    gapTable=array2table(rows,VariableNames={'leftFrame','rightFrame','startTime','endTime','durationSeconds', ...
        'uniformSamples','lidarRmseM','observerRmseM','maximumReferenceChordErrorM','lidarSquaredErrorFraction','observerSquaredErrorFraction'});
    zeroLateral=e.lateralInput;
    for field=["lateralVelocity","sideSlipAngle","sideSlipAngleRate"],zeroLateral.(field)(:)=0;end
    ablated=runMotionAidedVehicleObserver(e.data,zeroLateral,saved.cfg);
    ablationError=vecnorm(ablated.position(ix,:)-e.reference(ix,1:2),2,2);
    native=e.frameIndices(full);ref=e.frameReference(full,1:2);
    diagnostics=struct('gapThresholdSeconds',.25,'longGapUniformSamples',nnz(mask), ...
        'longGapSampleFraction',mean(mask),'longGapLidarSquaredErrorFraction',sum(lidarError(mask).^2)/sum(lidarError.^2), ...
        'longGapObserverSquaredErrorFraction',sum(observerError(mask).^2)/sum(observerError.^2), ...
        'outsideLongGaps',struct('samples',nnz(~mask),'lidarRmseM',rms(lidarError(~mask)), ...
        'observerRmseM',rms(observerError(~mask))),'gaps',gapTable, ...
        'lateralAblation',struct('change',"vy, beta and betaDot replaced with zero; all gains and LiDAR inputs unchanged", ...
        'uniformWithLateralRmseM',rms(observerError),'uniformZeroLateralRmseM',rms(ablationError), ...
        'nativeFullWithLateralRmseM',rms(vecnorm(e.estimate.position(native,:)-ref,2,2)), ...
        'nativeFullZeroLateralRmseM',rms(vecnorm(ablated.position(native,:)-ref,2,2)), ...
        'nativeFullZeroLateralMedianM',median(vecnorm(ablated.position(native,:)-ref,2,2)), ...
        'parametersSelectedFromThisAblation',false));
    writetable(gapTable,fullfile(outputFolder,'gap_diagnostics.csv'));
    fid=fopen(fullfile(outputFolder,'diagnostics.json'),'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(diagnostics,PrettyPrint=true));
    save(fullfile(outputFolder,'diagnostics.mat'),'diagnostics','ablated','mask','-v7.3');
    disp(gapTable);disp(diagnostics.lateralAblation);
end
