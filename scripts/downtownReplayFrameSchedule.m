function [ids,native,t]=downtownReplayFrameSchedule(datasetFolder,inputs)
% downtownReplayFrameSchedule Measurement-only query clock and acquisition IDs.
    protocol=downtownReferenceReplayConfig();frames=readtable(fullfile(datasetFolder,'frames.csv'));
    valid=frames.available==1 & mod(frames.frame_index,2)==protocol.queryFrameParity & ...
        frames.native_time_sec>=inputs.nativeOriginSeconds & ...
        frames.native_time_sec<=inputs.nativeOriginSeconds+inputs.highRate.time(end)-.1002;
    ids=frames.frame_index(valid);native=frames.native_time_sec(valid);t=native-inputs.nativeOriginSeconds;
    assert(numel(ids)>=2 && all(diff(t)>0),'VehicleLocalization:ReplayCoverage','Require increasing, measured-motion-covered query frames.');
end
