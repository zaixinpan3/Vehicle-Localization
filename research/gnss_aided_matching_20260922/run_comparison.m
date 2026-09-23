function summary=run_comparison()
% run_comparison Paired full-drive candidate selection and closed-loop controls.
% The last raw scan is matched, but native fusion input covers only 1169 scans.
    setupVehicleLocalization;dest=fileparts(mfilename('fullpath'));
    out='output/gnss_aided_matching_20260922';
    cache=load(fullfile(out,'sources.mat'));c=cache.calls;n=height(c);
    input=load('output/temporal_perception_20260922/observer/experiment.mat','data','cfg','lateral','reference');
    cfg=distributionRegistrationConfig();observerCfg=input.cfg;data=input.data;nf=numel(data.highRate.time);
    assert(max(abs(data.highRate.time-c.timeSeconds(1:nf)))<1e-8);
    first=c{1,{'predictedX','predictedY','predictedPsi'}};
    observerCfg.initialState([1 4 7])=first;
    data.lidar.pose=c{1:nf,{'x','y','psi'}};data.lidar.valid=c.accepted(1:nf);
    for k=1:nf
        data.lidar.information(:,:,k)=[c.informationXX(k) c.informationXY(k) c.informationXPsi(k); ...
            c.informationXY(k) c.informationYY(k) c.informationYPsi(k); ...
            c.informationXPsi(k) c.informationYPsi(k) c.informationPsiPsi(k)];
    end
    baseline=runSynchronousLocalizationObserver(data,observerCfg,input.lateral);
    gnssData=rmfield(data,'lidar');gnssOnly=runSynchronousLocalizationObserver(gnssData,observerCfg,input.lateral);
    references=c{:,{'referenceX','referenceY','referencePsi'}};
    results=cell(n,1);frozenPose=zeros(n,3);frozenValid=false(n,1);
    for k=1:n
        seed=c{k,{'predictedX','predictedY','predictedPsi'}};aid=[];
        if k<=nf && data.gnss.valid(k)
            [p,I]=correctGnssOutputPoint(data.gnss.position(k,:),data.gnss.information(:,:,k),seed(3),observerCfg.gnss.outputPoint);
            aid=struct('position',p,'covariance',I\eye(2),'valid',true);
        end
        r=matchLocalProbabilityCloud(cache.fixed,cache.sources{k},seed,cfg,aid);
        results{k}=r;frozenPose(k,:)=r.poseXYTheta;frozenValid(k)=r.accepted;
        if mod(k,100)==0,fprintf('Frozen-seed GNSS hypotheses %d/%d\n',k,n);end
    end
    save(fullfile(out,'frozen.mat'),'results','frozenPose','frozenValid','cfg','-v7.3');
    callbackData=rmfield(data,'lidar');
    callbackData.lidarMatcher=@(k,seed,aid) matchLocalProbabilityCloud(cache.fixed,cache.sources{k},seed,cfg,[]);
    fprintf('Closed-loop fused-seed control\n');
    feedbackOnly=runSynchronousLocalizationObserver(callbackData,observerCfg,input.lateral);
    save(fullfile(out,'feedback.mat'),'feedbackOnly','-v7.3');
    callbackData.lidarMatcher=@(k,seed,aid) matchLocalProbabilityCloud(cache.fixed,cache.sources{k},seed,cfg,aid);
    fprintf('Closed-loop GNSS hypothesis selection\n');
    aided=runSynchronousLocalizationObserver(callbackData,observerCfg,input.lateral);
    save(fullfile(out,'closed_loop.mat'),'baseline','gnssOnly','feedbackOnly','aided','observerCfg','cfg','-v7.3');
    variants=["recursive_original_matching","frozen_seed_aided_matching","fused_seed_matching", ...
        "fused_seed_aided_matching","original_fusion","feedback_fusion","aided_fusion","gnss_only"];
    rfb=feedbackOnly.matchingResults;ra=aided.matchingResults;
    fbPose=cell2mat(cellfun(@(r)r.poseXYTheta,rfb,UniformOutput=false));
    aPose=cell2mat(cellfun(@(r)r.poseXYTheta,ra,UniformOutput=false));
    fbValid=cellfun(@(r)r.accepted,rfb);aValid=cellfun(@(r)r.accepted,ra);
    values={c{:,{'x','y','psi'}},frozenPose,fbPose,aPose,baseline.pose,feedbackOnly.pose,aided.pose,gnssOnly.pose};
    masks={c.accepted,frozenValid,fbValid,aValid,true(nf,1),true(nf,1),true(nf,1),true(nf,1)};
    rows=cell(0,10);
    for j=1:numel(values)
        p=values{j};valid=masks{j};e=vecnorm(p(:,1:2)-references(1:size(p,1),1:2),2,2);
        yaw=rad2deg(wrap(p(:,3)-references(1:size(p,1),3)));
        for population=["all","accepted"]
            use=true(size(valid));if population=="accepted",use=valid;end
            rows(end+1,:)={variants(j),population,nnz(use),nnz(valid),sqrt(mean(e(use).^2)), ...
                median(e(use)),prctile(e(use),95),max(e(use)),nnz(e(use)>.3),sqrt(mean(yaw(use).^2))}; %#ok<AGROW>
        end
    end
    metrics=cell2table(rows,VariableNames={'variant','population','frames','accepted','rmseM','medianM','p95M','maxM','above30cm','yawRmseDeg'});
    writetable(metrics,fullfile(dest,'metrics.csv'));
    frameErrors=table(c.frame,c.positionErrorM,vecnorm(frozenPose(:,1:2)-references(:,1:2),2,2),frozenValid, ...
        VariableNames={'frame','originalM','frozenAidedM','frozenAccepted'});
    frameErrors.feedbackM=[vecnorm(fbPose(:,1:2)-references(1:nf,1:2),2,2);nan(n-nf,1)];
    frameErrors.closedAidedM=[vecnorm(aPose(:,1:2)-references(1:nf,1:2),2,2);nan(n-nf,1)];
    frameErrors.originalFusionM=[vecnorm(baseline.position-references(1:nf,1:2),2,2);nan(n-nf,1)];
    frameErrors.aidedFusionM=[vecnorm(aided.position-references(1:nf,1:2),2,2);nan(n-nf,1)];
    writetable(frameErrors,fullfile(dest,'frame_errors.csv'));
    signRows=cell(0,7);sets={results,rfb,ra};labels=["frozen_aided","feedback_only","closed_aided"];
    for j=1:3
        for k=950:959
            r=sets{j}{k};pairs=r.correspondences;R=rotation(references(k,3));
            for p=find(pairs.semanticName=="trafficSign").'
                target=pairs.globalTarget(p);source=pairs.source(p);
                delta=cache.sources{k}.components.mean(source,:)-(cache.fixed.components.mean(target,:)-references(k,1:2))*R;
                signRows(end+1,:)={labels(j),k,source,target,norm(delta),r.accepted,norm(r.poseXYTheta(1:2)-references(k,1:2))}; %#ok<AGROW>
            end
        end
    end
    signs=cell2table(signRows,VariableNames={'variant','frame','source','globalTarget','referenceDistanceM','accepted','poseErrorM'});
    writetable(signs,fullfile(dest,'sign_associations.csv'));
    summary=struct('rawScans',n,'fusionScans',nf,'sourceWindow',cache.cfg.sourceWindow, ...
        'perceptionChanged',false,'mapChanged',false,'gnssInformationAddedToLidar',false, ...
        'gnssSelectionIndependent',false,'referencePositionOrYawUsedForCandidateSelection',false,'referenceTiltUsed',true, ...
        'sourcePreparationSeconds',sum(cache.seconds),'frozenMatchingSeconds',sum(cellfun(@(r)r.matchingSeconds,results)), ...
        'closedMatchingSeconds',sum(cellfun(@(r)r.matchingSeconds,ra)), ...
        'frame959OriginalM',c.positionErrorM(959),'frame959AidedM',frameErrors.closedAidedM(959), ...
        'limitations',"Same-drive map; shared GNSS/INS reference; declared reference-offset startup and reference tilt; native offline bracket-aligned inputs; geometric information is uncalibrated; source window and GNSS selections are correlated");
    save(fullfile(out,'analysis.mat'),'metrics','frameErrors','signs','summary','references');
    fid=fopen(fullfile(dest,'summary.json'),'w');cleanup=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(summary,PrettyPrint=true));
    disp(metrics);disp(frameErrors(959,:));disp(signs);
end
function x=wrap(x)
    x=atan2(sin(x),cos(x));
end
function R=rotation(yaw)
    R=[cos(yaw) -sin(yaw);sin(yaw) cos(yaw)];
end
