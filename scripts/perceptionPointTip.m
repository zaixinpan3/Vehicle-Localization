function lines = perceptionPointTip(~, event)
% perceptionPointTip: Report original indices for the colored pcshow cloud.
    obj=event.Target;fig=ancestor(obj,'figure');
    source=getappdata(fig,'PointTipSource');
    indices=getappdata(obj,'OriginalFramePointIndices');
    labels=getappdata(obj,'PointDisplayLabels');
    localIndex=double(event.DataIndex);originalIndex=double(indices(localIndex));
    [row,column]=ind2sub(source.frameSize,originalIndex);
    xyz=source.xyz(originalIndex,:);layer=labels(localIndex);
    selection=struct('frameIndex',source.frameIndex,'originalIndex',originalIndex, ...
        'row',row,'column',column,'xyz',xyz,'layer',layer);
    assignin('base','lastPickedPoint',selection);
    history=getappdata(fig,'PickedPointIndices');
    if isempty(history) || history(end)~=originalIndex,history=[history(:);originalIndex];end
    setappdata(fig,'PickedPointIndices',history);assignin('base','pickedPointIndices',history);
    fprintf('[%s frame %d] OriginalIndex=%d | row=%d col=%d | XYZ=[%.6f %.6f %.6f] m | layer=%s\n', ...
        source.dataset,source.frameIndex,originalIndex,row,column,xyz(1),xyz(2),xyz(3),layer);
    lines={sprintf('Original index: %d (1-based)',originalIndex), ...
        sprintf('Row: %d, Column: %d',row,column), ...
        sprintf('X: %.6f m',xyz(1)),sprintf('Y: %.6f m',xyz(2)), ...
        sprintf('Z: %.6f m',xyz(3)),sprintf('Layer: %s',layer)};
end
