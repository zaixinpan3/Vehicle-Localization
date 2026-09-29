function runFreshBoundaryGeometry()
% runFreshBoundaryGeometry Retain temporal confirmation/scatter with fresh centers.
% Pooling viewpoint-dependent centers can introduce lag even with exact motion.
    setupVehicleLocalization();out='output/root_cause_matching_20260929';
    b=load(fullfile(out,'boundaryScatter2_sources.mat'),'sources','currentClouds');currentClouds=b.currentClouds;
    for variant=1:3
        sources=b.sources;
        for k=1:numel(sources)
            c=sources{k}.components;e=sources{k}.heightEvidence;keep=e.available;
            if variant==1,keep=keep & c.semanticName=="curb";end
            if variant==2,keep=keep & ismember(c.semanticName,["pole","trafficSign"]);end
            c.mean(keep,:)=e.mean(keep,1:2);sources{k}.components=c;
        end
        label="freshBoundary"+variant;file=fullfile(out,label+"_sources.mat");save(file,'sources','currentClouds','-v7.3');cfg=distributionRegistrationConfig();rootCauseReplay(file,label,cfg);
    end
end
