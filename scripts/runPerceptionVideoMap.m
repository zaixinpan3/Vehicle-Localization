function result = runPerceptionVideoMap(outputFolder, fig, cfg)
% runPerceptionVideoMap: Render fine perception and map its identical outputs.
% Supply a figure from showMississippiPerception after adjusting its camera.
% Its single pcshow cloud retains its native pcviewer tag. All camera, axis,
% palette, marker, and legend settings are retained. Frame cadence is the mean
% recorded LiDAR cadence; an AVI master contains one image per input frame.
% The mapping drive is registered with its matched GNSS/INS poses, then passed
% to the current canonical repeated-observation mapping implementation.
% Set cfg.buildMap=false to export perception and observations without fitting
% a map. The default retains the combined video-and-map workflow.
    arguments
        outputFolder (1,1) string
        fig (1,1) matlab.ui.Figure
        cfg (1,1) struct = featureMapBuildConfig()
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    buildMap=true;if isfield(cfg,'buildMap'),buildMap=cfg.buildMap;end
    assert(islogical(buildMap) && isscalar(buildMap),'cfg.buildMap must be a logical scalar.');
    dataRoot=fullfile(root,'data');
    if ~isfolder(outputFolder),mkdir(outputFolder);end
    videoPath=fullfile(outputFolder,'perception.avi');
    assert(~isfile(videoPath),'Choose a new output folder; video already exists.');
    matPath=fullfile(dataRoot,cfg.pointCloudMatPath);
    posePath=fullfile(dataRoot,cfg.poseMatchCsvPath);
    [~,total]=loadPointCloudFrame(matPath,1);
    frames=cfg.frameIndices;if isempty(frames),frames=1:total;end
    poses=readFramePoseTable(posePath,frames);
    assert(numel(frames)>=2 && all(diff(frames)==1),'Use a contiguous sequence of at least two frames.');
    stamps=double(poses.lidar_stamp_sec);
    assert(all(isfinite(stamps)) && all(diff(stamps)>0),'LiDAR timestamps must increase.');
    frameRate=(numel(frames)-1)/(stamps(end)-stamps(1));
    if isfield(cfg,'videoFrameRate'), frameRate=cfg.videoFrameRate; end
    cloud=findobj(fig,'Type','scatter','Tag','pcviewer');
    assert(isscalar(cloud),'Use a figure with one directly colored pcshow cloud.');
    ax=ancestor(cloud,'axes');
    properties={'XLim','YLim','ZLim','DataAspectRatio','PlotBoxAspectRatio', ...
        'CameraPosition','CameraTarget','CameraUpVector','CameraViewAngle','Projection'};
    viewState=struct();for k=1:numel(properties),viewState.(properties{k})=get(ax,properties{k});end
    set(ax,'XLimMode','manual','YLimMode','manual','ZLimMode','manual', ...
        'DataAspectRatioMode','manual','PlotBoxAspectRatioMode','manual', ...
        'CameraPositionMode','manual','CameraTargetMode','manual', ...
        'CameraUpVectorMode','manual','CameraViewAngleMode','manual');
    datacursormode(fig,'off');
    savefig(fig,fullfile(outputFolder,'initial_view.fig'));
    initialImage=getframe(fig);imageSize=size(initialImage.cdata);
    imwrite(initialImage.cdata,fullfile(outputFolder,'initial_view.png'));
    frameCounter=findall(fig,'Tag','PerceptionFrameCounter');
    if isempty(frameCounter)
        frameCounter=annotation(fig,'textbox',[.02 .94 .38 .045], ...
            'String','','Color','white','BackgroundColor','black','LineStyle','none', ...
            'FontSize',14,'FitBoxToText','on','Margin',6,'Interpreter','none', ...
            'Tag','PerceptionFrameCounter');
    end
    assert(isscalar(frameCounter),'Figure must contain at most one frame counter.');
    frameCounter.Visible='on';
    if isfield(cfg,'videoFrameCounter') && ~cfg.videoFrameCounter,frameCounter.Visible='off';end
    perceptionCfg=perceptionConfig('Mississippi');perceptionCfg.executionMode="offline";
    perceptionCfg.featureNames=cfg.featureNames;
    if isfield(cfg,'frameCalibration'),perceptionCfg.frameCalibration=cfg.frameCalibration;end
    save(fullfile(outputFolder,'run_configuration.mat'),'viewState','cfg','perceptionCfg','frameRate','imageSize','frames');
    writer=VideoWriter(videoPath,'Motion JPEG AVI');writer.FrameRate=frameRate;writer.Quality=95;
    open(writer);writerCleanup=onCleanup(@() close(writer));
    completed=0;timer=tic;
    updateProgress("perception",0);
    featureData=collectFeatureObservations(matPath,frames,poses,perceptionCfg,cfg,@renderFrame);
    close(writer);clear writerCleanup;
    save(fullfile(outputFolder,'feature_observations.mat'),'featureData','-v7.3');
    writetable(featureData.frameSummaryTable,fullfile(outputFolder,'frame_feature_counts.csv'));
    writetable(poses(:,{'frame_index','lidar_stamp_sec'}),fullfile(outputFolder,'frame_timestamps.csv'));
    mapSummary=table();
    if buildMap
        updateProgress("mapping",completed);
        probabilityCloudMap=buildSlidingWindowMap(featureData,cfg);
        probabilityCloudMap.sourceMatPath=string(matPath);probabilityCloudMap.poseMatchCsvPath=string(posePath);
        save(fullfile(outputFolder,'probability_cloud_map.mat'),'probabilityCloudMap','-v7.3');
        mapSummary=probabilityCloudMap.layerSummaryTable;
        writetable(mapSummary,fullfile(outputFolder,'map_layer_summary.csv'));
    end
    result=struct('outputFolder',outputFolder,'frameCount',completed,'frameRate',frameRate, ...
        'videoDurationSeconds',completed/frameRate,'videoImageSize',imageSize, ...
        'elapsedSeconds',toc(timer),'featureCounts',sum(featureData.counts,1), ...
        'featureNames',cfg.featureNames,'mapBuilt',buildMap,'mapSummary',mapSummary);
    save(fullfile(outputFolder,'run_result.mat'),'result');
    updateProgress("completed",completed);

    function renderFrame(frame,perception,index)
        updatePerceptionDisplay(fig,frame,perception.featureMasks,cfg.featureNames,index,total);
        fig.Name=sprintf('Mississippi frame %d - fine perception video',index);
        ax.Title.String=sprintf('Mississippi frame %d - fine perception',index);
        frameCounter.String=sprintf('Frame %d / %d',index,total);
        for j=1:numel(properties),set(ax,properties{j},viewState.(properties{j}));end
        drawnow;
        for j=1:numel(properties)
            assert(isequal(get(ax,properties{j}),viewState.(properties{j})), ...
                'Recording camera or axis geometry changed.');
        end
        assert(numel(cloud.XData)==nnz(isfinite(frame.x(:)) & isfinite(frame.y(:)) & isfinite(frame.z(:))), ...
            'The recording must retain every finite original point.');
        imageFrame=getframe(fig);
        assert(isequal(size(imageFrame.cdata),imageSize),'Keep the recording window size fixed.');
        writeVideo(writer,imageFrame);
        if mod(index,100)==0 || index==frames(1) || index==frames(end)
            imwrite(imageFrame.cdata,fullfile(outputFolder,sprintf('frame_%04d.png',index)));
        end
        completed=completed+1;
        updateProgress("perception",completed);
    end

    function updateProgress(phase,count)
        progress=struct('phase',phase,'completedFrames',count,'requestedFrames',numel(frames), ...
            'elapsedSeconds',toc(timer),'frameRate',frameRate);
        path=fullfile(outputFolder,'progress.json');temporary=path+".tmp";
        fid=fopen(temporary,'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(progress));fclose(fid);
        movefile(temporary,path,'f');
    end
end
