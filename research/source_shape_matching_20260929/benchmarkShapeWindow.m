function benchmarkShapeWindow()
% benchmarkShapeWindow Alternate timing order, preserving separate histories.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    data=load('output/root_cause_matching_20260929/finalSurface_sources.mat','currentClouds','sources');
    expected=load('output/source_shape_matching_20260929/shape50.mat','sources');
    old=load('output/line_direction_matching_20260928/production/report.mat','report');calls=old.report.calls;
    odom=load('output/line_direction_matching_20260928/sources.mat','motion');motion=odom.motion;
    base=localizationSourceWindowConfig();base.maximumShapeDistance=Inf;
    configs={base,localizationSourceWindowConfig()};histories={[],[]};outputs={data.sources,expected.sources};times=zeros(1170,2);
    for k=1:1170
        order=1:2;if mod(k,2)==0,order=[2 1];end
        for j=order
            timer=tic;[actual,histories{j}]=updateLocalizationSourceWindow(data.currentClouds{k},calls.timeSeconds(k),motion(k,:),histories{j},configs{j});times(k,j)=1000*toc(timer);
            assert(isequaln(actual,outputs{j}{k}));
        end
    end
    writetable(table((1:1170).',times(:,1),times(:,2),times(:,2)-times(:,1), ...
        VariableNames={'frame','baselineMs','adoptedMs','differenceMs'}),fullfile(dest,'paired_window_runtime.csv'));
    summary=table(median(times(:,1)),median(times(:,2)),median(times(:,2)-times(:,1)),prctile(times(:,1),95),prctile(times(:,2),95), ...
        VariableNames={'baselineMedianMs','adoptedMedianMs','pairedMedianDifferenceMs','baselineP95Ms','adoptedP95Ms'});
    disp(summary);writetable(summary,fullfile(dest,'paired_window_runtime_summary.csv'));
end
