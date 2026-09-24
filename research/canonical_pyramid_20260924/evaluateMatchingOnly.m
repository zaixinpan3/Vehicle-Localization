function T=evaluateMatchingOnly()
% evaluateMatchingOnly LiDAR-only map matching over the whole drive with the current matcher.
% Populations: the recursive stage (odometry seed, no GNSS, no observer), the
% reference-seeded ceiling (evaluation only) and the GNSS-aided matching
% inside the fused run, for the previous and the current production chain.
    setupVehicleLocalization;dest=fileparts(mfilename('fullpath'));
    chains={"before (single level)",'output/mncav_coarse_localization_20260924'; ...
        "after (canonical pyramid)",'output/mncav_coarse_localization_20260924b'};
    C=load('output/mncav_coarse_localization_20260924/sources.mat','fixed','sources','calls');
    ref=[C.calls.referenceX,C.calls.referenceY,C.calls.referencePsi];n=height(C.calls);cfg=distributionRegistrationConfig();
    rows=cell(0,16);traces=struct();
    for j=1:2
        M=load(fullfile(chains{j,2},'matching','report.mat'),'report');c=M.report.calls;
        pose=[c.x,c.y,c.psi];accepted=logical(c.accepted);
        rows(end+1,:)=describe(chains{j,1},"recursive (odometry seed)",pose,accepted,ref); %#ok<AGROW>
        traces.(sprintf('recursive%d',j))=hypot(pose(:,1)-ref(:,1),pose(:,2)-ref(:,2));
        E=load(fullfile(chains{j,2},'observer','experiment.mat'),'runs');r=E.runs{1}.estimate.matchingResults;
        pose=cell2mat(cellfun(@(x)x.poseXYTheta,r,UniformOutput=false));accepted=cellfun(@(x)x.accepted,r);
        rows(end+1,:)=describe(chains{j,1},"GNSS-aided (fused run)",[pose;nan(n-size(pose,1),3)],[accepted;false(n-numel(accepted),1)],ref); %#ok<AGROW>
        traces.(sprintf('aided%d',j))=[hypot(pose(:,1)-ref(1:size(pose,1),1),pose(:,2)-ref(1:size(pose,1),2));nan(n-size(pose,1),1)];
    end
    % Reference-seeded ceiling with the current matcher (evaluation only).
    pose=zeros(n,3);accepted=false(n,1);
    for k=1:n
        r=matchLocalProbabilityCloud(C.fixed,C.sources{k},ref(k,:),cfg,[]);pose(k,:)=r.poseXYTheta;accepted(k)=r.accepted;
    end
    rows(end+1,:)=describe(chains{2,1},"reference-seeded ceiling",pose,accepted,ref);
    T=cell2table(rows,VariableNames={'chain','population','accepted','rmseCm','medianCm','p90Cm','p95Cm','p99Cm','maxCm','maxFrame', ...
        'above20cm','above30cm','above50cm','yawRmseDeg','longitudinalMeanRmsCm','lateralMeanRmsCm'});
    disp(T);writetable(T,fullfile(dest,'matching_only.csv'));
    % Segments that exceeded 30 cm before, and the frames still above 30 cm.
    segments=[158 214;421 426;841 853;875 879;955 959];
    fprintf('\nsegment (frames)      before max/median   after max/median\n');
    for s=1:size(segments,1)
        k=segments(s,1):segments(s,2);a=traces.recursive1(k);b=traces.recursive2(k);
        fprintf('%4d-%4d           %6.1f / %5.1f     %6.1f / %5.1f\n',segments(s,1),segments(s,2),100*max(a),100*median(a),100*max(b),100*median(b));
    end
    M=load(fullfile(chains{2,2},'matching','report.mat'),'report');c=M.report.calls;e=traces.recursive2;
    fprintf('after: frames above 30 cm: %s\n',mat2str(find(e>.3 & logical(c.accepted)).'));
    t=(c.timeSeconds-c.timeSeconds(1));
    fig=figure('Visible','off','Color','w','Position',[100 100 1250 620]);theme(fig,'light');tiledlayout(2,1,TileSpacing='compact');
    nexttile;hold on;plot(t,100*traces.recursive1,Color=[.6 .6 .6],DisplayName='before: single level');
    plot(t,100*traces.recursive2,Color=[.1 .45 .85],DisplayName='after: canonical pyramid');ylim([0 75]);grid on;
    ylabel('Position error (cm)');legend(Location='northeast');title('LiDAR-only recursive map matching, 1170 scans, no GNSS, no observer');
    nexttile;hold on;plot(t,100*traces.recursive2,Color=[.1 .45 .85],DisplayName='LiDAR-only recursive');
    plot(t,100*traces.aided2,Color=[.85 .3 .1],DisplayName='GNSS-aided, inside the fused run');ylim([0 45]);grid on;
    ylabel('Position error (cm)');xlabel('Time since first scan (s)');legend(Location='northeast');title('Current matcher');
    exportgraphics(fig,fullfile(dest,'matching_only_traces.png'),Resolution=140);close(fig);
end

function row=describe(chain,population,pose,accepted,ref)
    e=hypot(pose(:,1)-ref(:,1),pose(:,2)-ref(:,2));a=accepted & isfinite(e);ea=e(a);
    yaw=rad2deg(atan2(sin(pose(:,3)-ref(:,3)),cos(pose(:,3)-ref(:,3))));
    dx=pose(:,1)-ref(:,1);dy=pose(:,2)-ref(:,2);
    lon=dx.*cos(ref(:,3))+dy.*sin(ref(:,3));lat=-dx.*sin(ref(:,3))+dy.*cos(ref(:,3));
    ids=find(a);[mx,i]=max(ea);
    row={chain,population,nnz(a),100*rms(ea),100*median(ea),100*prctile(ea,90),100*prctile(ea,95),100*prctile(ea,99),100*mx,ids(i), ...
        nnz(ea>.2),nnz(ea>.3),nnz(ea>.5),rms(yaw(a)),sprintf('%.1f / %.1f',100*mean(lon(a)),100*rms(lon(a))),sprintf('%.1f / %.1f',100*mean(lat(a)),100*rms(lat(a)))};
end
