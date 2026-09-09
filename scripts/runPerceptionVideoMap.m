function result = runPerceptionVideoMap(outputFolder, fig, cfg)
% runPerceptionVideoMap: Render fine perception and map its identical outputs.
% Supply the existing pcshow figure after adjusting its camera. Its source and
% semantic scatter layers must carry PointLayerName appdata. All camera, axis,
% palette, marker, and legend settings are retained. Frame cadence is the mean
% recorded LiDAR cadence; an AVI master contains one image per input frame.
% The mapping drive is registered with its matched GNSS/INS poses, then passed
% to the current canonical repeated-observation mapping implementation.
    arguments
        outputFolder (1,1) string
        fig (1,1) matlab.ui.Figure
        cfg (1,1) struct = featureMapBuildConfig()
    end
    root=fileparts(fileparts(mfilename('fullpath')));
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
    objects=findobj(fig,'Type','scatter');
    names=arrayfun(@(h) string(getappdata(h,'PointLayerName')),objects);
    assert(nnz(names=="source")==1 && all(ismember(cfg.featureNames,names)), ...
        'Figure must have tagged source and requested semantic layers.');
    ax=ancestor(objects(1),'axes');
    properties={'XLim','YLim','ZLim','DataAspectRatio','PlotBoxAspectRatio', ...
        'CameraPosition','CameraTarget','CameraUpVector','CameraViewAngle','Projection'};
    viewState=struct();for k=1:numel(properties),viewState.(properties{k})=get(ax,properties{k});end
    set(ax,'XLimMode','manual','YLimMode','manual','ZLimMode','manual', ...
        'DataAspectRatioMode','manual','PlotBoxAspectRatioMode','manual', ...
        'CameraPositionMode','manual','CameraTargetMode','manual', ...
        'CameraUpVectorMode','manual','CameraViewAngleMode','manual');
    legendHandle=findobj(fig,'Type','legend');
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
    updateProgress("mapping",completed);
    probabilityCloudMap=buildSlidingWindowMap(featureData,cfg);
    probabilityCloudMap.sourceMatPath=string(matPath);probabilityCloudMap.poseMatchCsvPath=string(posePath);
    save(fullfile(outputFolder,'probability_cloud_map.mat'),'probabilityCloudMap','-v7.3');
    writetable(probabilityCloudMap.layerSummaryTable,fullfile(outputFolder,'map_layer_summary.csv'));
    result=struct('outputFolder',outputFolder,'frameCount',completed,'frameRate',frameRate, ...
        'videoDurationSeconds',completed/frameRate,'videoImageSize',imageSize, ...
        'elapsedSeconds',toc(timer),'featureCounts',sum(featureData.counts,1), ...
        'featureNames',cfg.featureNames,'mapSummary',probabilityCloudMap.layerSummaryTable);
    save(fullfile(outputFolder,'run_result.mat'),'result');
    updateProgress("completed",completed);

    function renderFrame(frame,perception,index)
        xyz=double([frame.x(:),frame.y(:),frame.z(:)]);
        for j=1:numel(objects)
            name=names(j);
            if name=="source",mask=all(isfinite(xyz),2);else,mask=perception.featureMasks.(name);end
            selected=find(mask);
            set(objects(j),'XData',xyz(selected,1),'YData',xyz(selected,2),'ZData',xyz(selected,3));
            setappdata(objects(j),'OriginalFramePointIndices',selected);
        end
        if ~isempty(legendHandle)
            legendHandle.String=compose('%s: %d points',cfg.featureNames(:), ...
                arrayfun(@(name) nnz(perception.featureMasks.(name)),cfg.featureNames(:)));
        end
        source=struct('xyz',xyz,'featureMasks',perception.featureMasks, ...
            'frameSize',size(frame.x),'frameIndex',index);
        setappdata(fig,'PointTipSource',source);
        fig.Name=sprintf('Mississippi frame %d - fine perception video',index);
        frameCounter.String=sprintf('Frame %d / %d',index,total);
        for j=1:numel(properties),set(ax,properties{j},viewState.(properties{j}));end
        drawnow;
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
