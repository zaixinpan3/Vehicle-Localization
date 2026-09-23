function validation=validate_graph_experiment()
% validate_graph_experiment Verify baseline preservation and association changes.
    setupVehicleLocalization;dest=fileparts(mfilename('fullpath'));out='output/robust_pose_graph_20260923';
    names=["robustPoseGraphTest","positionAidedRegistrationTest","synchronousLocalizationTest", ...
        "geometricRegistrationTest","localizationSourceWindowTest","distributionRegistrationTest", ...
        "registrationInformationTest","repeatabilityRegistrationTest","gnssOutputPointTest","fullLocalizationObserverTest"];
    tests=runtests("tests/"+names+".m");assert(all([tests.Passed]));writetable(table(tests),fullfile(dest,'tests.csv'));
    clouds=load(fullfile(out,'sources.mat'));oldClouds=load('output/gnss_aided_matching_20260922/sources.mat');
    identical=cellfun(@(a,b)isequaln(a,b),clouds.sources,oldClouds.sources);assert(all(identical));
    input=load('output/temporal_perception_20260922/observer/experiment.mat','data','lateral');
    old=load('output/gnss_aided_matching_20260922/closed_loop.mat','aided','observerCfg');
    data=rmfield(input.data,'lidar');reg=distributionRegistrationConfig();
    data.lidarMatcher=@(k,seed,aid)matchLocalProbabilityCloud(clouds.fixed,clouds.sources{k},seed,reg,aid);
    reproduced=runSynchronousLocalizationObserver(data,old.observerCfg,input.lateral);
    baselineDifference=max(abs(reproduced.pose-old.aided.pose),[],'all');assert(baselineDifference<1e-7);
    guided=load(fullfile(out,'guided.mat'),'estimate');n=numel(guided.estimate.time);rows=cell(0,8);pairCount=0;distinctExtra=0;
    for k=1:n
        a=old.aided.matchingResults{k};b=guided.estimate.matchingResults{k};
        distinctExtra=distinctExtra+(b.positionAiding.candidateCount>a.positionAiding.candidateCount);
        if ~isfield(a,'correspondences')||~isfield(b,'correspondences'),continue;end
        p=a.correspondences;q=b.correspondences;
        [~,ia,ib]=intersect(p.source,q.source);pairCount=pairCount+numel(ia);
        changed=find(p.globalTarget(ia)~=q.globalTarget(ib));
        reference=clouds.calls{k,{'referenceX','referenceY','referencePsi'}};
        R=[cos(reference(3)) -sin(reference(3));sin(reference(3)) cos(reference(3))];
        for c=changed.'
            i=ia(c);j=ib(c);s=p.source(i);before=p.globalTarget(i);after=q.globalTarget(j);
            world=clouds.sources{k}.components.mean(s,:)*R.'+reference(1:2);
            distances=vecnorm(clouds.fixed.components.mean([before after],:)-world,2,2);
            rows(end+1,:)={k,s,p.semanticName(i),before,after,distances(1),distances(2),b.accepted}; %#ok<AGROW>
        end
    end
    changes=cell2table(rows,VariableNames={'frame','source','class','oldTarget','newTarget','oldReferenceDistanceM','newReferenceDistanceM','accepted'});
    writetable(changes,fullfile(dest,'association_changes.csv'));
    reportRows=cell(0,5);
    for variant=["baseline","guided","switchable","quadratic"]
        if variant=="baseline"
            r=old.aided.matchingResults;
        elseif variant=="guided"
            r=guided.estimate.matchingResults;
        else
            loaded=load(fullfile(out,variant+'.mat'),'estimate');r=loaded.estimate.results;
        end
        for k=950:959
            p=r{k}.correspondences;
            for j=find(p.semanticName=="trafficSign").'
                influence=NaN;if isstruct(p)&&isfield(p,'graphInfluence'),influence=p.graphInfluence(j);end
                reportRows(end+1,:)={variant,k,p.source(j),p.globalTarget(j),influence}; %#ok<AGROW>
            end
        end
    end
    writetable(cell2table(reportRows,VariableNames={'variant','frame','source','target','graphInfluence'}),fullfile(dest,'sign950_959.csv'));
    files=["config/robustPoseGraphConfig.m","localization/registerSemanticProbabilityCloud.m", ...
        "localization/prepareSemanticRegistrationGeometry.m","localization/updateLocalizationSourceWindow.m", ...
        "localization/updateRobustPoseGraph.m","localization/runRobustGraphLocalization.m", ...
        "localization/createRobustGraphMatcher.m","localization/selectLocalProbabilityCloud.m", ...
        "localization/matchLocalProbabilityCloud.m","localization/selectPositionAidedRegistration.m", ...
        "localization/registrationSupport.m","scripts/prepareMississippiLocalizationClouds.m", ...
        "tests/robustPoseGraphTest.m","tests/positionAidedRegistrationTest.m", ...
        "research/robust_pose_graph_20260923/run_graph_experiment.m", ...
        "research/robust_pose_graph_20260923/run_guided_experiment.m", ...
        "research/robust_pose_graph_20260923/validate_graph_experiment.m"];
    counts=arrayfun(@(p)numel(checkcode(p,'-id','-config=factory')),files);
    writetable(table(files.',counts.',VariableNames={'file','findings'}),fullfile(dest,'code_analysis.csv'));
    validation=struct('testsPassed',nnz([tests.Passed]),'testCount',numel(tests), ...
        'unchangedStackedSources',nnz(identical),'baselinePoseDifferenceMaximum',baselineDifference, ...
        'correspondencesCompared',pairCount,'changedAssociations',height(changes), ...
        'changedPointAssociations',nnz(ismember(changes.class,["pole","trafficSign"])), ...
        'framesWithAdditionalCandidate',distinctExtra,'codeAnalyzerFindings',sum(counts), ...
        'matlabVersion',version,'defaultBackendChanged',false);
    fid=fopen(fullfile(dest,'validation.json'),'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(validation,PrettyPrint=true));disp(validation);
end
