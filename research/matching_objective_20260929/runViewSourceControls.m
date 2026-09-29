function runViewSourceControls()
% runViewSourceControls Recheck the new maximum after repairing map geometry.
% All detector masks remain fixed. These are source-representation controls.
    setupVehicleLocalization();out='output/matching_objective_20260929';
    base=load('output/root_cause_matching_20260929/finalSurface_sources.mat','sources','currentClouds');currentClouds=base.currentClouds;
    mapFile='output/mississippi_mapping_calibrated/view_conditioned_cloud.mat';
    for label=["viewSingletonPole","viewCurrentPointCenters","viewCurrentCurbCenters","viewCurrentPointGeometry"]
        sources=base.sources;
        for k=1:numel(sources)
            cloud=sources{k};c=cloud.components;e=cloud.heightEvidence;
            if label=="viewSingletonPole"
                if ~any(ismember(c.semanticName,["pole","trafficSign"])),continue;end
                raw=currentClouds{k}.components;ids=find(raw.semanticName=="pole" & raw.semanticProbability>=.94);
                for id=ids.'
                    if any(sum((c.mean(c.semanticName=="pole",:)-raw.mean(id,:)).^2,2)<=.75^2),continue;end
                    n=c.numComponents+1;c.mean(n,:)=raw.mean(id,:);c.covariance(:,:,n)=raw.covariance(:,:,id);
                    c.semanticName(n,1)="pole";c.semanticProbability(n,1)=raw.semanticProbability(id);c.occupancyProbability(n,1)=raw.occupancyProbability(id);
                    c.detectionFrameCount(n,1)=1;c.detectionFrameMask(n,:)=false;c.detectionFrameMask(n,end)=true;c.temporalStability(n,1)=.1;c.numComponents=n;
                    rawHeight=registrationSupport.getHeightEvidence(currentClouds{k});e.mean(n,:)=rawHeight.mean(id,:);e.covariance(:,:,n)=rawHeight.covariance(:,:,id);e.available(n,1)=rawHeight.available(id);
                end
                mass=c.semanticProbability.*c.occupancyProbability.*c.temporalStability;c.mixtureWeight=mass/max(sum(mass),realmin);
            else
                keep=e.available & ismember(c.semanticName,["pole","trafficSign"]);
                if label=="viewCurrentCurbCenters",keep=e.available & c.semanticName=="curb";end
                c.mean(keep,:)=e.mean(keep,1:2);
                if label=="viewCurrentPointGeometry",c.covariance(:,:,keep)=e.covariance(1:2,1:2,keep);end
            end
            cloud.components=c;cloud.heightEvidence=e;sources{k}=cloud;
        end
        file=fullfile(out,label+"_sources.mat");save(file,'sources','currentClouds','-v7.3');
        replayStudy(file,label,distributionRegistrationConfig(),mapFile);
    end
end
