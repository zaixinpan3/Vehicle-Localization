function [aligned,lateral,metadata]=align(data,lateralInput,cfg)
% Align synthetic reference observations for validation on their own clock.
    if ~isfield(data,'lidar')
        [aligned,lateral,metadata]=localizationSupport.synchronizeLocalizationInputs(data,lateralInput,cfg);
        return
    end
    a=data.lidar;
    [aligned,lateral,metadata,selected]=observerInternal.alignFrameClock(data,lateralInput,cfg,a.time(:));
    aligned.lidar=a;aligned.lidar.time=aligned.highRate.time;
    for name=["pose","valid"]
        if isfield(a,name),aligned.lidar.(name)=a.(name)(selected,:);end
    end
    aligned.lidar.information=a.information(:,:,selected);
    if isfield(a,'evaluateResidual')
        indices=find(selected);
        aligned.lidar.evaluateResidual=@(k,predicted)a.evaluateResidual(indices(k),predicted);
    end
end
