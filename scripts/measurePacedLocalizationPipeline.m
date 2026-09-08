function calls = measurePacedLocalizationPipeline(folder, period, repetitions)
% measurePacedLocalizationPipeline Measure serial processing under paced input.
% Uses preloaded raw frames/maps from benchmarkLocalizationPipeline. MATLAB
% wall-clock scheduling models regular frame availability; it is not a ROS
% transport measurement. Acquisition timestamps are not relabelled.

    arguments
        folder (1,1) string
        period (1,1) double {mustBePositive} = 0.09169316291809082
        repetitions (1,1) double {mustBeInteger,mustBePositive} = 10
    end
    loaded = load(fullfile(folder,'inputs.mat'),'inputs','cfg');
    starts = [.5,-.4,deg2rad(2);-.5,.4,-deg2rad(2);0,0,0];
    cases = 3*numel(loaded.inputs);
    stream = RandStream('mt19937ar','Seed',20260906);
    order = zeros(cases*repetitions,1);
    for pass = 1:repetitions
        order((pass-1)*cases+(1:cases)) = randperm(stream,cases);
    end
    rows = cell(numel(order),11);
    % One untimed warm call prevents first-execution cost dominating the queue.
    item = loaded.inputs{1}; cfg = loaded.cfg;
    cfg.perception.coarseProbabilityCloud.projectionRotation = item.tilt;
    localizeLidarFrame(item.frame,item.mapCloud,item.pose,0,cfg);
    clockStart = tic;
    for index = 1:numel(order)
        scene = floor((order(index)-1)/3)+1;
        start = mod(order(index)-1,3)+1;
        item = loaded.inputs{scene};
        cfg.perception.coarseProbabilityCloud.projectionRotation = item.tilt;
        available = (index-1)*period;
        remaining = available-toc(clockStart);
        if remaining > 0, pause(remaining); end
        began = toc(clockStart);
        [event,result] = localizeLidarFrame(item.frame,item.mapCloud, ...
            item.pose+starts(start,:),double(item.frameIndex),cfg);
        finished = toc(clockStart);
        assert((result.accepted || result.directionalAccepted) == ~isempty(event),'Acceptance/event mismatch.');
        rows(index,:) = {index,item.frameIndex,start,1000*available, ...
            1000*(began-available),1000*(finished-began), ...
            1000*(finished-available),result.accepted,string(result.reason),period,result.directionalAccepted};
    end
    calls = cell2table(rows,'VariableNames',{'order','frame','start','availableMs', ...
        'waitingMs','computeMs','availabilityToPoseMs','accepted','reason','periodSeconds','directionalAccepted'});
    writetable(calls,fullfile(folder,sprintf('paced_%.3fms.csv',1000*period)));
    fprintf('Paced %.3f ms: %d calls, maximum availability-to-output %.3f ms.\n', ...
        1000*period,height(calls),max(calls.availabilityToPoseMs));
end
