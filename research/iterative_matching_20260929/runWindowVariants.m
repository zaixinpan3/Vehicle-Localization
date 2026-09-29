function runWindowVariants()
% runWindowVariants Test the causal support lifetime with unchanged detections.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    data=load('output/pole_boundary_recovery_20260929/replay.mat','currentClouds');currentClouds=data.currentClouds;
    p=load('output/line_direction_matching_20260928/production/report.mat','report');calls=p.report.calls;
    o=load('output/line_direction_matching_20260928/sources.mat','motion');summaries=table();
    for count=[3 4 6 7 9]
        wc=localizationSourceWindowConfig();wc.maximumFrames=count;wc.maximumAgeSeconds=.1*(count-.5);
        sources=cell(1170,1);history=[];
        for k=1:1170,[sources{k},history]=updateLocalizationSourceWindow(currentClouds{k},calls.timeSeconds(k),o.motion(k,:),history,wc);end
        label="window"+count;sourceFile=fullfile('output/iterative_matching_20260929',label+"_sources.mat");save(sourceFile,'sources','currentClouds','wc','-v7.3');
        summary=replayCloudVariant(sourceFile,label,baselineMatchingConfig());summaries=[summaries;summary]; %#ok<AGROW>
        writetable(summaries,fullfile(dest,'window_screen.csv'));disp(summary);
    end
end
