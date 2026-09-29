function runCorroboratedPoleVariants()
% runCorroboratedPoleVariants Do not let unconfirmed poles be the sole XY anchor.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));out='output/iterative_matching_20260929';
    base=load('output/pole_boundary_recovery_20260929/replay.mat','sources');saved=load(fullfile(out,'cleanSoft.mat'),'cfg');summaries=table();
    for j=1:3
        d=load(fullfile(out,"singletonPole"+(j+2)+"_sources.mat"),'sources','currentClouds');sources=d.sources;currentClouds=d.currentClouds;
        for k=1:numel(sources)
            if ~any(ismember(base.sources{k}.components.semanticName,["pole","trafficSign"])),sources{k}=base.sources{k};end
        end
        label="corroboratedPole"+j;file=fullfile(out,label+"_sources.mat");save(file,'sources','currentClouds','-v7.3');
        row=replayCloudVariant(file,label,saved.cfg);summaries=[summaries;row]; %#ok<AGROW>
        writetable(summaries,fullfile(dest,'corroborated_pole_screen.csv'));
    end
end
