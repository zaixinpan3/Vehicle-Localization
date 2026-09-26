function profileShaftSpeed(label,useBaseline,baselineDir)
% profileShaftSpeed: Profile identical frames; timings include profiler overhead.
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));
    originalPath=path;
    clean=onCleanup(@()shaftSpeedUseBackend(originalPath,baselineDir,false));
    shaftSpeedUseBackend(originalPath,baselineDir,useBaseline);
    speedFrames=[1 71 301 501 851 1031];
    source=matfile(fullfile(root,'data','raw','MissisipiPointClouds.mat'));
    block=cell(numel(speedFrames),1);
    for k=1:numel(speedFrames),block{k}=source.pointClouds(1,speedFrames(k));end
    cfg=perceptionConfig();
    for k=1:3,perceiveFrame(block{1},cfg);end
    profile clear;profile on;
    for repeat=1:3
        for k=1:numel(speedFrames),perceiveFrame(block{k},cfg);end
    end
    profile off;speedProfile=profile('info');
    output=fullfile(root,'output','pillar_shaft_speed_20260926');
    save(fullfile(output,char(string(label)+"_profile.mat")),'speedProfile','speedFrames','cfg');
    functions=speedProfile.FunctionTable;
    [~,order]=sort([functions.TotalTime],'descend');
    T=table(string({functions(order).FunctionName}).', ...
        [functions(order).TotalTime].',[functions(order).NumCalls].', ...
        'VariableNames',{'functionName','inclusiveSeconds','calls'});
    writetable(T,fullfile(folder,char(string(label)+"_profile.csv")));
    disp(T(1:min(height(T),16),:));
end
