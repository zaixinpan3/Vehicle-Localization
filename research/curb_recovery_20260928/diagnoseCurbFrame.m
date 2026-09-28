function diagnoseCurbFrame(frameIndex)
% diagnoseCurbFrame: Attribute reference misses to proposal and point routing.
    if nargin<1,frameIndex=500;end
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));
    cache=load(fullfile(root,'output','semantic_precision_20260927','mississippi_baseline.mat'),'frames','references','names');
    at=cache.frames==frameIndex;ref=cache.references{at,cache.names=="curb"};
    frame=loadPointCloudFrame(fullfile(root,'data','raw','MissisipiPointClouds.mat'),frameIndex);
    cfg=perceptionConfig('Mississippi');cfg.semanticPrecision.modelFiles=struct();
    cfg.coarseProbabilityCloud.storeDiagnostics=true;p=perceiveFrame(frame,cfg);
    g=p.candidates.geometry;xyz=double([frame.x(ref),frame.y(ref),frame.z(ref)]);xyz=reshape(xyz,[],3);
    bins=floor((xyz(:,1:2)-g.origin)./g.cellSize)+1;ids=sub2ind(g.mapSize,bins(:,2),bins(:,1));
    selected=double(p.candidates.pillarIndices{p.candidates.semanticNames=="curb"});
    inGround=ismember(ref,p.diagnostics.groundPointContext.groundOriginalPointIdx);
    T=table(double(ref(:)),xyz(:,1),xyz(:,2),xyz(:,3),ids,ismember(ids,selected),inGround(:), ...
        'VariableNames',{'sourceIndex','x','y','z','pillar','selected','coarseGround'});
    writetable(T,fullfile(folder,sprintf('frame%d_reference_diagnosis.csv',frameIndex)));
    disp(groupsummary(T,{'selected','coarseGround'}));
end
