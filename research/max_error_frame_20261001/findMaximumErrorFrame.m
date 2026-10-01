function findMaximumErrorFrame()
% findMaximumErrorFrame Rank per-frame discrepancies of the current replay.
% Scores the raw recursive matching stage and the fused "both" observer of
% runMncavCoarseLocalizationExperiment against the INSPVA evaluation
% trajectory. The first two seconds contain the deliberate initial offset
% and are reported separately using the existing startup convention.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    folder='output/max_error_frame_20261001/run';
    b=load(fullfile(folder,'observer','experiment.mat'),'runs','reference','data');
    m=load(fullfile(folder,'matching','report.mat'),'report');calls=m.report.calls;
    t=b.data.highRate.time;ref=b.reference;run=b.runs{1};r=run.estimate;
    assert(run.scenario=="both" && isequal(r.time,t));
    [distance,index]=min(abs(calls.timeSeconds-t.'),[],1);index=index.';
    assert(max(distance)<1e-7 && numel(unique(index))==numel(t));frame=calls.frame(index);
    fused=bodyError(r.pose,ref);
    matchPose=cell2mat(cellfun(@(x)x.poseXYTheta,r.matchingResults,UniformOutput=false));
    accepted=cellfun(@(x)x.accepted,r.matchingResults);
    directional=cellfun(@(x)x.directionalAccepted,r.matchingResults);
    reason=cellfun(@(x)string(x.reason),r.matchingResults);
    matched=bodyError(matchPose,ref);seed=bodyError(r.matchingSeeds,ref);
    gnss=nan(numel(t),4);valid=all(isfinite(r.diagnostics.gnssPositionAtObserverPoint),2);
    gnssPose=[r.diagnostics.gnssPositionAtObserverPoint,ref(:,3)];
    gnss(valid,:)=bodyError(gnssPose(valid,:),ref(valid,:));
    trace=table(frame,t,fused(:,4),fused(:,1),fused(:,2),fused(:,3),seed(:,4),seed(:,1),seed(:,2),seed(:,3), ...
        accepted,directional,reason,matched(:,4),matched(:,1),matched(:,2),matched(:,3), ...
        gnss(:,4),gnss(:,1),gnss(:,2),r.diagnostics.mode,VariableNames={'frame','time', ...
        'fusedErrorM','fusedLongitudinalM','fusedLateralM','fusedYawDeg', ...
        'seedErrorM','seedLongitudinalM','seedLateralM','seedYawDeg','fullMatch','directionalMatch','reason', ...
        'matchErrorM','matchLongitudinalM','matchLateralM','matchYawDeg', ...
        'gnssErrorM','gnssLongitudinalM','gnssLateralM','mode'});
    writetable(trace,fullfile(dest,'frame_errors.csv'));
    raw=bodyError([calls.x,calls.y,calls.psi],[calls.referenceX,calls.referenceY,calls.referencePsi]);
    rawTrace=table(calls.frame,calls.timeSeconds,raw(:,4),raw(:,1),raw(:,2),raw(:,3),calls.accepted,calls.directionalAccepted,calls.reason, ...
        VariableNames={'frame','time','errorM','longitudinalM','lateralM','yawDeg','accepted','directional','reason'});
    writetable(rawTrace,fullfile(dest,'raw_matching_errors.csv'));
    late=trace(trace.time>=t(1)+2,:);
    disp('Fused "both" observer, ten largest after the 2 s startup:');
    disp(head(sortrows(late,'fusedErrorM','descend'),10));
    disp('Fused "both" observer, largest including startup:');
    disp(head(sortrows(trace,'fusedErrorM','descend'),3));
    lateRaw=rawTrace(rawTrace.time>=2,:);
    disp('Raw recursive matching, ten largest after 2 s:');
    disp(head(sortrows(lateRaw,'errorM','descend'),10));
    [~,at]=max(late.fusedErrorM);[~,rawAt]=max(lateRaw.errorM);
    summary=struct('fusedFrames',numel(t),'fusedRmseM',rms(trace.fusedErrorM),'fusedRmseAfterStartupM',rms(late.fusedErrorM), ...
        'fusedMaximumFrame',late.frame(at),'fusedMaximumTime',late.time(at),'fusedMaximumM',late.fusedErrorM(at), ...
        'fusedMaximumIncludingStartupFrame',trace.frame(1),'fusedMaximumIncludingStartupM',max(trace.fusedErrorM), ...
        'rawMaximumFrame',lateRaw.frame(rawAt),'rawMaximumM',lateRaw.errorM(rawAt), ...
        'acceptedMatches',nnz(accepted),'directionalMatches',nnz(directional), ...
        'evaluation',"Recorded INSPVA trajectory at native LiDAR frame times; not surveyed truth");
    fid=fopen(fullfile(dest,'maximum_summary.json'),'w');fprintf(fid,'%s\n',jsonencode(summary,PrettyPrint=true));fclose(fid);
    disp(summary);
end

function e=bodyError(pose,ref)
% Columns: longitudinal, lateral (reference vehicle axes), yaw in degrees, norm.
    d=pose(:,1:2)-ref(:,1:2);c=cos(ref(:,3));s=sin(ref(:,3));
    yaw=rad2deg(atan2(sin(pose(:,3)-ref(:,3)),cos(pose(:,3)-ref(:,3))));
    e=[d(:,1).*c+d(:,2).*s,-d(:,1).*s+d(:,2).*c,yaw,vecnorm(d,2,2)];
end
