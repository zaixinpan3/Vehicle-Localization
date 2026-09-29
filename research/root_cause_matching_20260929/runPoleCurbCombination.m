function runPoleCurbCombination(curbFile,label)
% runPoleCurbCombination Combine independently measured class location models.
    setupVehicleLocalization();out='output/root_cause_matching_20260929';a=load(curbFile,'currentClouds');b=load(fullfile(out,'poleAxis_sources.mat'),'currentClouds');
    currentClouds=a.currentClouds;sources=cell(1170,1);history=[];wc=localizationSourceWindowConfig();
    old=load('output/line_direction_matching_20260928/production/report.mat','report');odom=load('output/line_direction_matching_20260928/sources.mat','motion');
    for k=1:1170
        c=a.currentClouds{k}.components;d=b.currentClouds{k}.components;keep=c.semanticName~="pole";extra=d.semanticName=="pole";names=fieldnames(c);
        for j=1:numel(names)
            name=names{j};if strcmp(name,'numComponents'),continue;end
            x=c.(name);y=d.(name);
            if ismember(name,{'covariance','invCovariance','covarianceXYZ'}),c.(name)=cat(3,x(:,:,keep),y(:,:,extra));else,c.(name)=[x(keep,:);y(extra,:)];end
        end
        c.numComponents=nnz(keep)+nnz(extra);c.mixtureWeight=c.unnormalizedWeight/sum(c.unnormalizedWeight);
        currentClouds{k}.components=c;
        [sources{k},history]=updateLocalizationSourceWindow(currentClouds{k},old.report.calls.timeSeconds(k),odom.motion(k,:),history,wc);
    end
    file=fullfile(out,label+"_sources.mat");save(file,'sources','currentClouds','-v7.3');rootCauseReplay(file,label,distributionRegistrationConfig());
end
