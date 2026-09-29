function runSourceDiagnostics()
% runSourceDiagnostics Diagnose motion error versus temporal geometry bias.
% The oracle-motion control uses reference relative motion for diagnosis only.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));out='output/root_cause_matching_20260929';
    data=load('output/pole_boundary_recovery_20260929/replay.mat','sources','currentClouds');currentClouds=data.currentClouds;
    old=load('output/line_direction_matching_20260928/production/report.mat','report');calls=old.report.calls;
    cfg=distributionRegistrationConfig();wc=localizationSourceWindowConfig();summaries=table();
    for variant=1:3
        sources=data.sources;
        if variant==1
            history=[];motion=calls{:,{'referenceX','referenceY','referencePsi'}};
            for k=1:1170,[sources{k},history]=updateLocalizationSourceWindow(currentClouds{k},calls.timeSeconds(k),motion(k,:),history,wc);end
            label="oracleHistoryMotion";
        else
            for k=1:1170
                c=sources{k}.components;e=sources{k}.heightEvidence;keep=e.available;
                if variant==3,keep=keep & c.semanticName=="curb";end
                c.mean(keep,:)=e.mean(keep,1:2);c.covariance(:,:,keep)=e.covariance(1:2,1:2,keep);sources{k}.components=c;
            end
            label="currentConfirmedWithStale"+variant;
        end
        file=fullfile(out,label+"_sources.mat");save(file,'sources','currentClouds','-v7.3');row=rootCauseReplay(file,label,cfg);summaries=[summaries;row]; %#ok<AGROW>
        writetable(summaries,fullfile(dest,'source_diagnostics.csv'));
    end
end
