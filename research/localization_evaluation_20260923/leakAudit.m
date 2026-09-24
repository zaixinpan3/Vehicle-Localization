function leakAudit()
% Isolation and perturbation audit of reference-derived runtime inputs.
    addpath(pwd);setupVehicleLocalization;
    out='output/localization_evaluation_20260923';
    E=load(fullfile(out,'observer','experiment.mat'),'data','lateral','lateralDesign','cfg','runs','reference');
    C=load(fullfile(out,'sources.mat'),'fixed','sources','calls');
    ref=E.reference;prod=E.runs{1}.estimate.pose;
    base=rmfield(E.data,'lidar');
    fprintf('highRate fields: %s\n',strjoin(string(fieldnames(base.highRate)),', '));
    fprintf('gnss fields: %s\n',strjoin(string(fieldnames(base.gnss)),', '));
    fprintf('lateral fields: %s\n',strjoin(string(fieldnames(E.lateral)),', '));
    % 1) Isolated rerun: only motion, lateral, GNSS, map, source clouds, config.
    est=isolatedRun(base,E.lateral,E.lateralDesign,E.cfg,C.fixed,C.sources);
    fprintf('ISOLATED max|pose - production| = %.3g\n',max(abs(est.pose-prod),[],'all'));
    rows=cell(0,6);rows(end+1,:)=row("production (ref+[0.5,-0.4,2deg] init, INS tilt)",est,ref);
    % 2) Initialization without the reference XY: first valid GNSS, heading perturbed.
    g=find(base.gnss.valid,1);cfg0=E.cfg;
    fprintf('first valid GNSS index %d, init error of GNSS XY vs ref: %.3f m\n',g,norm(base.gnss.position(g,:)-ref(1,1:2)));
    for dyaw=[0 -3 3 -8 8]
        cfg=cfg0;cfg.initialState([1 4])=base.gnss.position(g,:).';cfg.initialState(7)=ref(1,3)+deg2rad(dyaw);
        rows(end+1,:)=row(sprintf("GNSS XY init, ref yaw %+g deg",dyaw),isolatedRun(base,E.lateral,E.lateralDesign,cfg,C.fixed,C.sources),ref); %#ok<AGROW>
    end
    for off=[2 -2;-3 3].'
        cfg=cfg0;cfg.initialState([1 4])=ref(1,1:2).'+off;
        rows(end+1,:)=row(sprintf("ref XY %+g/%+g m init",off(1),off(2)),isolatedRun(base,E.lateral,E.lateralDesign,cfg,C.fixed,C.sources),ref); %#ok<AGROW>
    end
    % 3) No INS roll/pitch: identity projection rotation for every scan.
    Z=zeroTiltSources();
    rows(end+1,:)=row("zero tilt sources (no INS roll/pitch)",isolatedRun(base,E.lateral,E.lateralDesign,cfg0,C.fixed,Z),ref);
    T=cell2table(rows,VariableNames={'variant','fusedRmseCm','fusedAfter2sRmseCm','fusedMaxAfter2sCm','matchRmseCm','accepted'});
    disp(T);writetable(T,fullfile(out,'leak_audit.csv'));
end

function est=isolatedRun(data,lateral,lateralDesign,cfg,map,sources)
    regCfg=distributionRegistrationConfig();
    data.lidarMatcher=@(k,seed,aid) matchLocalProbabilityCloud(map,sources{k},seed,regCfg,aid);
    est=runFullLocalizationObserver(data,lateralDesign,cfg,LateralInputs=lateral);
end

function r=row(name,est,ref)
    e=vecnorm(est.position-ref(:,1:2),2,2);post=est.time>=est.time(1)+2;
    m=cell2mat(cellfun(@(x)x.poseXYTheta,est.matchingResults,UniformOutput=false));
    a=cellfun(@(x)x.accepted,est.matchingResults);em=vecnorm(m(a,1:2)-ref(a,1:2),2,2);
    r={name,100*rms(e),100*rms(e(post)),100*max(e(post)),100*rms(em),nnz(a)};
    fprintf('%-50s fused %.2f cm (after 2 s %.2f, max %.2f), match %.2f cm, accepted %d\n',name,r{2:end});
end

function sources=zeroTiltSources()
    root=setupVehicleLocalization();
    saved=load('output/temporal_perception_20260922/five_frame_matching/report.mat','cfg','report');
    cfg=saved.cfg;cfg.sourceWindow=localizationSourceWindowConfig();calls=saved.report.calls;n=height(calls);
    sources=cell(n,1);history=[];mapCfg=featureMapBuildConfig();
    store=matfile(fullfile(root,'data',mapCfg.pointCloudMatPath));
    cfg.perception.coarseProbabilityCloud.projectionRotation=eye(3);
    for first=1:50:n
        ids=first:min(n,first+49);block=store.pointClouds(1,calls.frame(ids));
        for k=ids
            raw=perceiveCoarseProbabilityCloud(block(k-first+1),cfg.perception);
            motion=saved.report.deadReckoning{k,{'x','y','psi'}};
            [sources{k},history]=updateLocalizationSourceWindow(raw,calls.timeSeconds(k),motion,history,cfg.sourceWindow);
        end
    end
end
