function summary=run_height_experiment(reuseResults)
% run_height_experiment Paired causal observer replays with relative Z evidence.
% Reference coordinates are used only for evaluation; inherited source tilt
% and initial observer alignment retain the baseline's declared limitations.
    if nargin<1,reuseResults=false;end
    setupVehicleLocalization;dest=fileparts(mfilename('fullpath'));
    out='output/height_association_20260923';cache=load(fullfile(out,'sources.mat'));
    previous=load('output/gnss_aided_matching_20260922/closed_loop.mat','aided','observerCfg');
    input=load('output/temporal_perception_20260922/observer/experiment.mat','data','lateral');
    data=rmfield(input.data,'lidar');cfg=distributionRegistrationConfig();
    estimates=cell(6,1);labels=["xy","relative_z_all","relative_z_objects","relative_z_signs", ...
        "marginal_z_objects","marginal_z_signs"];
    for variant=1:6
        cfg.relativeHeight.enabled=variant>1;
        cfg.relativeHeight.candidateModel="conditional";
        if variant>=5,cfg.relativeHeight.candidateModel="marginal";end
        cfg.relativeHeight.semanticNames=["pole","trafficSign"];
        if variant==2,cfg.relativeHeight.semanticNames=["curb","pole","trafficSign","facade"];end
        if ismember(variant,[4 6]),cfg.relativeHeight.semanticNames="trafficSign";end
        if reuseResults && isfile(fullfile(out,labels(variant)+'.mat'))
            saved=load(fullfile(out,labels(variant)+'.mat'),'estimate');estimates{variant}=saved.estimate;
        else
            data.lidarMatcher=@(k,seed,aid) matchFrame(k,seed,aid,cache,cfg,labels(variant));
            estimates{variant}=runSynchronousLocalizationObserver(data,previous.observerCfg,input.lateral);
            estimate=estimates{variant};save(fullfile(out,labels(variant)+'.mat'),'estimate','cfg','-v7.3');
        end
    end
    baselineDifference=max(abs(estimates{1}.pose-previous.aided.pose),[],'all');
    assert(baselineDifference<1e-7,'Height-off replay must reproduce the active baseline.');
    n=numel(estimates{1}.time);ref=cache.calls{1:n,{'referenceX','referenceY','referencePsi'}};
    matchPose=cell(6,1);valid=false(n,6);matchError=zeros(n,6);fusionError=matchError;seconds=matchError;
    for j=1:6
        results=estimates{j}.matchingResults;
        matchPose{j}=cell2mat(cellfun(@(a)a.poseXYTheta,results,UniformOutput=false));
        valid(:,j)=cellfun(@(a)a.accepted,results);
        matchError(:,j)=vecnorm(matchPose{j}(:,1:2)-ref(:,1:2),2,2);
        fusionError(:,j)=vecnorm(estimates{j}.position-ref(:,1:2),2,2);
        seconds(:,j)=cellfun(@(a)a.matchingSeconds,results);
    end
    common=all(valid,2);post=estimates{1}.time>=2;rows=cell(0,10);
    for j=1:6
        for population=["fusion_all","fusion_after_2s","matching_accepted","matching_common"]
            if population=="fusion_all"
                e=fusionError(:,j);mask=true(n,1);
            elseif population=="fusion_after_2s"
                e=fusionError(:,j);mask=post;
            elseif population=="matching_accepted"
                e=matchError(:,j);mask=valid(:,j);
            else
                e=matchError(:,j);mask=common;
            end
            ids=find(mask);[peak,p]=max(e(mask));
            rows(end+1,:)={labels(j),population,nnz(mask),rms(e(mask)),median(e(mask)), ...
                prctile(e(mask),95),prctile(e(mask),99),peak,cache.calls.frame(ids(p)),nnz(e(mask)>.3)}; %#ok<AGROW>
        end
    end
    metrics=cell2table(rows,VariableNames={'variant','population','frames','rmseM','medianM', ...
        'p95M','p99M','maxM','maxFrame','above30cm'});
    frameTables=cell(6,1);variants=struct([]);
    for j=1:6
        relative=cellfun(@heightDetails,estimates{j}.matchingResults);
        frameTables{j}=table(repmat(labels(j),n,1),cache.calls.frame(1:n),fusionError(:,j),matchError(:,j), ...
            valid(:,j),[relative.enabled].',[relative.anchorCount].',[relative.offset].', ...
            [relative.offsetScatter].',seconds(:,j),VariableNames={'variant','frame', ...
            'fusionM','matchingM','accepted','heightEnabled','anchors','heightOffsetM','heightScatterM','seconds'});
        variants(j).name=labels(j);
        variants(j).heightEnabledFrames=nnz(frameTables{j}.heightEnabled);
        variants(j).lostAcceptedFrames=nnz(valid(:,1)&~valid(:,j));
        variants(j).newAcceptedFrames=nnz(~valid(:,1)&valid(:,j));
        variants(j).matchingSeconds=sum(seconds(:,j));
        variants(j).medianSeconds=median(seconds(:,j));
        variants(j).p95Seconds=prctile(seconds(:,j),95);
    end
    frames=vertcat(frameTables{:});changeRows=cell(0,11);
    for variant=2:6
        totalPairs=0;changedPairs=0;
        for k=1:n
            a=estimates{1}.matchingResults{k};b=estimates{variant}.matchingResults{k};
            if ~isfield(a,'correspondences')||~isfield(b,'correspondences'),continue;end
            p=a.correspondences;q=b.correspondences;[~,ia,ib]=intersect(p.source,q.source);totalPairs=totalPairs+numel(ia);
            changed=find(p.globalTarget(ia)~=q.globalTarget(ib));changedPairs=changedPairs+numel(changed);
            yaw=ref(k,3);r=[cos(yaw) -sin(yaw);sin(yaw) cos(yaw)];
            for j=changed.'
                i=ia(j);v=ib(j);s=p.source(i);old=p.globalTarget(i);new=q.globalTarget(v);
                world=cache.sources{k}.components.mean(s,:)*r.'+ref(k,1:2);
                distance=vecnorm(cache.fixed.components.mean([old new],:)-world,2,2);
                changeRows(end+1,:)={labels(variant),cache.calls.frame(k),s,p.semanticName(i),old,new,distance(1),distance(2), ...
                    q.heightResidual(v),q.heightAssociationCost(v),b.accepted}; %#ok<AGROW>
            end
        end
        variants(variant).comparedCorrespondences=totalPairs;
        variants(variant).changedTargetGaussians=changedPairs;
    end
    changes=cell2table(changeRows,VariableNames={'variant','frame','source','class','oldTarget','newTarget', ...
        'oldReferenceDistanceM','newReferenceDistanceM','heightResidualM','heightCost','accepted'});
    oldCache=load('output/robust_pose_graph_20260923/sources.mat','sources');
    same=cellfun(@(a,b)isequaln(a.components,b.components),cache.sources,oldCache.sources);assert(all(same));
    summary=struct('rawScans',numel(cache.sources),'fusionScans',n,'commonMatchingFrames',nnz(common), ...
        'unchangedXyClouds',nnz(same),'baselinePoseDifferenceMaximum',baselineDifference, ...
        'variants',variants, ...
        'referenceAltitudeUsed',false,'referenceTiltRetained',true,'mapWeightsChanged',false, ...
        'heightForceAdded',false,'heightEvidence',"current scan belonging to confirmed five-scan tracks", ...
        'defaultEnabled',distributionRegistrationConfig().relativeHeight.enabled);
    writetable(metrics,fullfile(dest,'metrics.csv'));writetable(frames,fullfile(dest,'frames.csv'));
    writetable(changes,fullfile(dest,'association_changes.csv'));
    fid=fopen(fullfile(dest,'summary.json'),'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(summary,PrettyPrint=true));disp(metrics);disp(summary);
end

function result=matchFrame(k,seed,aid,cache,cfg,label)
    result=matchLocalProbabilityCloud(cache.fixed,cache.sources{k},seed,cfg,aid);
    if mod(k,100)==0,fprintf('%s replay %d\n',label,k);end
end

function d=heightDetails(result)
    d=struct('enabled',false,'anchorCount',0,'offset',NaN,'offsetScatter',NaN);
    if isfield(result.height,'relativeAssociation')
        a=result.height.relativeAssociation;names=fieldnames(d);
        for j=1:numel(names),d.(names{j})=a.(names{j});end
    end
end
