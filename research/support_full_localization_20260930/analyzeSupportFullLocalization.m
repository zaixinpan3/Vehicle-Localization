function analyzeSupportFullLocalization()
% analyzeSupportFullLocalization Score current full observer and paired matches.
% Report startup separately using the existing two-second evaluation convention.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    folder='output/support_full_localization_20260930';
    b=load(fullfile(folder,'observer','experiment.mat'));
    m=load(fullfile(folder,'matching','report.mat'),'report');calls=m.report.calls;
    t=b.data.highRate.time;ref=b.reference;
    [distance,index]=min(abs(calls.timeSeconds-t.'),[],1);index=index.';
    assert(max(distance)<1e-7 && numel(unique(index))==numel(t));frames=calls.frame(index);
    rows=cell(0,14);traces=cell(0,1);peaks=cell(0,11);paired=cell(0,10);
    masks={true(size(t)),t>=t(1)+2,t>=40&t<60,t>=60&t<70};
    labels=["all","after_2_seconds","outage_40_60","recovery_60_70"];
    for k=1:numel(b.runs)
        run=b.runs{k};r=run.estimate;assert(isequal(r.time,t));
        e=r.pose-ref;e(:,3)=wrap(e(:,3));position=vecnorm(e(:,1:2),2,2);yaw=rad2deg(e(:,3));
        full=false(size(t));directional=full;matchError=nan(size(t));matchYaw=matchError;
        if isfield(r,'matchingResults') && ~isempty(r.matchingResults)
            full=cellfun(@(x)x.accepted,r.matchingResults);
            directional=cellfun(@(x)x.directionalAccepted,r.matchingResults);
            pose=cell2mat(cellfun(@(x)x.poseXYTheta,r.matchingResults,UniformOutput=false));
            matchError(full)=vecnorm(pose(full,1:2)-ref(full,1:2),2,2);
            matchYaw(full)=rad2deg(wrap(pose(full,3)-ref(full,3)));
        end
        for j=1:numel(masks)
            ix=find(masks{j});p=position(ix);a=yaw(ix);[peak,at]=max(p);at=ix(at);
            rows(end+1,:)={run.scenario,labels(j),numel(ix),rms(p),median(p),prctile(p,95),peak,frames(at),t(at), ...
                rms(a),max(abs(a)),mean(p<=.1),nnz(full(ix)),nnz(directional(ix))}; %#ok<AGROW>
            peaks(end+1,:)={run.scenario,labels(j),frames(at),t(at),peak,e(at,1),e(at,2),yaw(at),r.diagnostics.mode(at),matchError(at),full(at)}; %#ok<AGROW>
        end
        for j=1:2
            ix=full & masks{j};if ~any(ix),continue;end
            paired(end+1,:)={run.scenario,labels(j),nnz(ix),rms(matchError(ix)),rms(position(ix)), ...
                max(matchError(ix)),max(position(ix)),rms(matchYaw(ix)),rms(yaw(ix)),mean(position(ix)<matchError(ix))}; %#ok<AGROW>
        end
        traces{end+1}=table(frames,t,repmat(run.scenario,numel(t),1),position,yaw,e(:,1),e(:,2), ...
            full,directional,matchError,matchYaw,r.diagnostics.mode,r.diagnostics.lidarLongitudinalVelocityBias, ...
            r.diagnostics.lidarVelocityBias,VariableNames={'frame','time','scenario','positionErrorM','yawErrorDeg','errorX','errorY', ...
            'fullMatch','directionalMatch','matchedPositionErrorM','matchedYawErrorDeg','mode','learnedLongitudinalBiasMps','learnedLateralBiasMps'}); %#ok<AGROW>
    end
    metrics=cell2table(rows,VariableNames={'scenario','population','frames','positionRmseM','medianM','p95M','maximumM','maximumFrame', ...
        'maximumTime','yawRmseDeg','yawMaximumDeg','fractionWithin10cm','fullMatches','directionalMatches'});
    peakTable=cell2table(peaks,VariableNames={'scenario','population','frame','time','errorM','errorX','errorY','yawErrorDeg','mode','matchedErrorM','fullMatch'});
    pairedTable=cell2table(paired,VariableNames={'scenario','population','frames','matchingRmseM','observerRmseM','matchingMaximumM', ...
        'observerMaximumM','matchingYawRmseDeg','observerYawRmseDeg','fractionObserverBetter'});
    trace=vertcat(traces{:});
    writetable(metrics,fullfile(dest,'metrics.csv'));writetable(peakTable,fullfile(dest,'peaks.csv'));
    writetable(pairedTable,fullfile(dest,'paired_matching.csv'));writetable(trace,fullfile(dest,'frame_errors.csv'));
    writetable(trace(trace.frame==895,:),fullfile(dest,'frame895.csv'));
    assertMncavReplayCurrent(b.report.metadata.inputMetadata.parameters);
    summary=struct('rawFrames',height(calls),'observerFrames',numel(t),'firstFrame',frames(1),'lastFrame',frames(end), ...
        'excludedFrames',setdiff(calls.frame,frames),'registrationMethod',m.report.metadata.registrationMethod, ...
        'rawPerceptionRerun',m.report.metadata.perceptionRerun,'closedLoopRematching',b.report.metadata.closedLoopRematching, ...
        'registrationSeedFromFusedState',true,'zeroProcessingDelay',b.report.metadata.zeroLidarProcessingDelay, ...
        'synchronization',b.report.metadata.synchronization,'parametersCurrent',true,'lateralCertified',b.lateralDesign.certified, ...
        'lateralDecayRate',b.lateralDesign.decayRate,'lateralGainBound',b.lateralDesign.gainBound, ...
        'lateralMaximumCertificateMargin',b.lateralDesign.maxCertificateMargin, ...
        'rawMatching',m.report.summary,'fusedDiagnostics',b.runs{1}.estimate.diagnostics, ...
        'headCommit',string(getenv('LOCALIZATION_EXPERIMENT_HEAD')), ...
        'workingTreeInputs','output/support_full_localization_20260930/source_inputs.json');
    summary.fusedDiagnostics=rmfield(summary.fusedDiagnostics,{'mode','headingMode','positionCorrection','gnssPositionAtObserverPoint', ...
        'gnssInformationAtObserverPoint','yawCorrection','lidarVelocityBias','lidarLongitudinalVelocityBias','gnssDerivedHeading', ...
        'minimumWeights','translationDissipationMargin'});
    fid=fopen(fullfile(dest,'summary.json'),'w');fprintf(fid,'%s\n',jsonencode(summary,PrettyPrint=true));fclose(fid);
    fig=figure('Visible','off','Color','w','Position',[100 100 1300 800]);cleanup=onCleanup(@()close(fig));
    colororder(fig,[.10 .55 .25;.10 .45 .80;.85 .30 .08]);tiledlayout(2,2,Padding='compact');
    nexttile;origin=ref(1,1:2);plot(ref(:,1)-origin(1),ref(:,2)-origin(2),'k--');hold on;
    plot(b.runs{1}.estimate.position(:,1)-origin(1),b.runs{1}.estimate.position(:,2)-origin(2));
    axis equal;grid on;xlabel('Easting (m)');ylabel('Northing (m)');title('Mississippi full route');legend('Reference','Fused');
    nexttile;hold on;
    for k=1:3
        mask=trace.scenario==b.runs{k}.scenario & trace.time>=2;
        plot(trace.time(mask),100*trace.positionErrorM(mask),DisplayName=b.runs{k}.scenario);
    end
    grid on;xlabel('Receiver time (s)');ylabel('Position discrepancy (cm)');title('After the common 2 s startup');legend(Interpreter='none');
    nexttile;hold on;
    for k=1:3
        mask=trace.scenario==b.runs{k}.scenario;plot(trace.time(mask),trace.yawErrorDeg(mask),DisplayName=b.runs{k}.scenario);
    end
    grid on;xlabel('Receiver time (s)');ylabel('Heading discrepancy (deg)');title('Heading including startup');
    nexttile;hold on;
    for k=4:6
        mask=trace.scenario==b.runs{k}.scenario;plot(trace.time(mask),100*trace.positionErrorM(mask),DisplayName=b.runs{k}.scenario);
    end
    xline(40,'k:',HandleVisibility='off');xline(60,'k:',HandleVisibility='off');grid on;xlabel('Receiver time (s)');ylabel('Position discrepancy (cm)');
    title('Declared sensor withdrawals, 40-60 s');legend(Interpreter='none');
    set(findall(fig,'Type','axes'),'Color','w','XColor','k','YColor','k');
    set(findall(fig,'Type','text'),'Color','k');set(findall(fig,'Type','legend'),'Color','w','TextColor','k');
    exportgraphics(fig,fullfile(dest,'comparison.png'),Resolution=160);
    exportgraphics(fig,fullfile(dest,'comparison.pdf'),ContentType='vector');
    disp(metrics(metrics.population=="after_2_seconds"|metrics.population=="all",:));disp(pairedTable);
end
function a=wrap(a),a=atan2(sin(a),cos(a));end
