function runSingletonPoleVariants()
% runSingletonPoleVariants Test weak admission of unconfirmed current poles.
% Research-only: detector masks and pillar moments remain untouched.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));out='output/iterative_matching_20260929';
    data=load('output/pole_boundary_recovery_20260929/replay.mat','currentClouds','sources');currentClouds=data.currentClouds;
    saved=load(fullfile(out,'cleanSoft.mat'),'cfg');cfg=saved.cfg;summaries=table();settings=[.9 .1;.9 .2;.94 .1;.94 .2;.94 .4;.98 .2];
    for j=1:size(settings,1)
        sources=data.sources;added=0;
        for k=1:numel(sources)
            cloud=sources{k};c=cloud.components;raw=currentClouds{k}.components;
            ids=find(raw.semanticName=="pole" & raw.semanticProbability>=settings(j,1));
            for id=ids.'
                if any(sum((c.mean(c.semanticName=="pole",:)-raw.mean(id,:)).^2,2)<=.75^2),continue;end
                n=c.numComponents+1;c.mean(n,:)=raw.mean(id,:);c.covariance(:,:,n)=raw.covariance(:,:,id);
                c.semanticName(n,1)="pole";c.semanticProbability(n,1)=raw.semanticProbability(id);c.occupancyProbability(n,1)=raw.occupancyProbability(id);
                c.detectionFrameCount(n,1)=1;c.detectionFrameMask(n,:)=false;c.detectionFrameMask(n,end)=true;
                c.temporalStability(n,1)=settings(j,2);c.numComponents=n;
                e=registrationSupport.getHeightEvidence(currentClouds{k});
                cloud.heightEvidence.mean(n,:)=e.mean(id,:);cloud.heightEvidence.covariance(:,:,n)=e.covariance(:,:,id);cloud.heightEvidence.available(n,1)=e.available(id);added=added+1;
            end
            mass=c.semanticProbability.*c.occupancyProbability.*c.temporalStability;c.mixtureWeight=mass/max(sum(mass),realmin);cloud.components=c;sources{k}=cloud;
        end
        label="singletonPole"+j;file=fullfile(out,label+"_sources.mat");save(file,'sources','currentClouds','settings','j','added','-v7.3');
        row=replayCloudVariant(file,label,cfg);row.added=added;summaries=[summaries;row]; %#ok<AGROW>
        writetable(summaries,fullfile(dest,'singleton_pole_screen.csv'));
    end
end
