function runCurrentGeometryVariants()
% runCurrentGeometryVariants Use history for confirmation, current curb geometry.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));summaries=table();
    b=load('output/pole_boundary_recovery_20260929/replay.mat','currentClouds');currentClouds=b.currentClouds;
    p=load('output/line_direction_matching_20260928/production/report.mat','report');calls=p.report.calls;
    o=load('output/line_direction_matching_20260928/sources.mat','motion');
    for variant=1:2
        wc=localizationSourceWindowConfig();wc.currentGeometryClasses=["curb","facade"];
        if variant==2,wc.currentGeometryClasses=["curb","facade","pole","trafficSign"];end
        history=[];sources=cell(1170,1);
        for k=1:1170,[sources{k},history]=updateLocalizationSourceWindow(currentClouds{k},calls.timeSeconds(k),o.motion(k,:),history,wc);end
        label="currentGeometry"+variant;sourceFile=fullfile('output/iterative_matching_20260929',label+"_sources.mat");save(sourceFile,'sources','currentClouds','wc','-v7.3');
        cfg=baselineMatchingConfig();cfg.pyramid.trustClasses="pole";
        row=replayCloudVariant(sourceFile,label,cfg);summaries=[summaries;row]; %#ok<AGROW>
        writetable(summaries,fullfile(dest,'current_geometry_screen.csv'));
    end
end
