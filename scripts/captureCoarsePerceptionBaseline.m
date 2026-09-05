function captureCoarsePerceptionBaseline(referenceRoot, snapshotFolder, referenceCommit)
% captureCoarsePerceptionBaseline: Freeze behavior and loaded-frame timing.
% referenceRoot must contain the intended source revision (for example a git
% archive export). referenceCommit identifies that export; this function does
% not infer a revision from the caller's current working tree. Files are never
% overwritten. Restore the caller's path after using the reference code.
    root=fileparts(fileparts(mfilename('fullpath')));
    assert(nargin==3 && strlength(string(referenceCommit))>0,'Supply the source revision and output folder.');
    if ~isfolder(snapshotFolder), mkdir(snapshotFolder); end
    indices=[150 260 300 326 370 450 550 600 700 850 900 1000 1100 1150 75 225 475 775 1125 100 200 300 400 500];
    originalPath=path;
    cleanup=onCleanup(@() path(originalPath));
    run(fullfile(referenceRoot,'setupVehicleLocalization.m'));
    assert(startsWith(which('perceiveFrame'),char(referenceRoot)),'Wrong reference code on path.');
    cfg=perceptionConfig(); offlineCfg=cfg; offlineCfg.executionMode="offline";
    for k=1:numel(indices)
        snapshot=struct('frameIndex',indices(k),'dataset',"mississippi",'split',"development");
        filename='MissisipiPointClouds.mat';
        if k>=15, snapshot.split="validation"; end
        if k>=20, snapshot.dataset="downtown"; filename='downTownPointClouds.mat'; end
        output=fullfile(snapshotFolder,sprintf('%s_%04d.mat',snapshot.dataset,indices(k)));
        assert(~isfile(output),'Refusing to overwrite a frozen baseline: %s',output);
        snapshot.frame=loadPointCloudFrame(fullfile(root,'data','raw',filename),indices(k));
        snapshot.coarse=perceiveFrame(snapshot.frame,cfg);
        snapshot.offline=perceiveFrame(snapshot.frame,offlineCfg);
        snapshot.seconds=zeros(1,7);
        for repeat=1:7
            timer=tic; perceiveFrame(snapshot.frame,cfg); snapshot.seconds(repeat)=toc(timer);
        end
        snapshot.config=cfg; snapshot.commit=string(referenceCommit);
        save(output,'snapshot');
    end
end
