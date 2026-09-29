function runFinalSensitivity()
% runFinalSensitivity Challenge the selected solution with new combinations.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));out='output/iterative_matching_20260929';
    b=load('output/pole_boundary_recovery_20260929/replay.mat','currentClouds');currentClouds=b.currentClouds;
    old=load('output/line_direction_matching_20260928/production/report.mat','report');calls=old.report.calls;
    odom=load('output/line_direction_matching_20260928/sources.mat','motion');saved=load(fullfile(out,'cleanSoft.mat'),'cfg');base=saved.cfg;summaries=table();
    for n=[3 4 6]
        wc=localizationSourceWindowConfig();wc.maximumFrames=n;wc.maximumAgeSeconds=.1*(n-.5);history=[];sources=cell(1170,1);
        for k=1:1170,[sources{k},history]=updateLocalizationSourceWindow(currentClouds{k},calls.timeSeconds(k),odom.motion(k,:),history,wc);end
        label="softWindow"+n;file=fullfile(out,label+"_sources.mat");save(file,'sources','currentClouds','wc','-v7.3');
        row=replayCloudVariant(file,label,base);summaries=[summaries;row]; %#ok<AGROW>
        writetable(summaries,fullfile(dest,'final_sensitivity_screen.csv'));
    end
    for noise=[.05 .15 .2 .3]
        cfg=base;cfg.geometric.noiseStandardDeviation=noise;
        row=replayCloudVariant('output/pole_boundary_recovery_20260929/replay.mat',"softNoise"+round(noise*100),cfg);summaries=[summaries;row]; %#ok<AGROW>
        writetable(summaries,fullfile(dest,'final_sensitivity_screen.csv'));
    end
end
