function results = benchmarkCoarseProbabilityCloud(dataRoot, frameIndices)
% benchmarkCoarseProbabilityCloud: Measure current coarse and fine execution.
% Warm timeit measurements exclude data loading, rendering and localization.
    root=fileparts(fileparts(mfilename('fullpath'))); run(fullfile(root,'setupVehicleLocalization.m'));
    if nargin<1, dataRoot=fullfile(root,'data'); end
    if nargin<2, frameIndices=[260 300 326]; end
    frameIndices=frameIndices(:);
    coarseSeconds=zeros(size(frameIndices)); fineSeconds=coarseSeconds;
    coarseBytes=coarseSeconds; fineBytes=coarseSeconds; components=coarseSeconds;
    cfg=perceptionConfig(); fineCfg=cfg; fineCfg.executionMode="offline";
    for k=1:numel(frameIndices)
        frame=loadPointCloudFrame(fullfile(dataRoot,'raw','MissisipiPointClouds.mat'),frameIndices(k));
        coarse=perceiveFrame(frame,cfg); fine=perceiveFrame(frame,fineCfg); %#ok<NASGU>
        coarseSeconds(k)=timeit(@() perceiveFrame(frame,cfg));
        fineSeconds(k)=timeit(@() perceiveFrame(frame,fineCfg));
        info=whos('coarse'); coarseBytes(k)=info.bytes;
        info=whos('fine'); fineBytes(k)=info.bytes;
        components(k)=coarse.probabilityCloud.components.numComponents;
    end
    results=table(frameIndices,coarseSeconds,fineSeconds,coarseBytes,fineBytes,components);
    disp(results);
end
