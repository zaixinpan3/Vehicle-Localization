function runExperiment()
% runExperiment Compare fresh fine and coarse perception with frozen matching.
% Each recursive arm feeds back its own accepted matching output. All use the
% same independent wheel/gyro/lateral transport and initial offset. Rejected
% poses fall back to that arm's motion prediction and remain in output metrics.
    setupVehicleLocalization();maxNumCompThreads(4);
    out='output/fine_matching_20260919';
    baseline=load('output/matching_refinement_20260919/validated/recursive/report.mat');
    cached=load('output/matching_refinement_20260919/inputs.mat');
    c=baseline.report.calls; n=height(c);fineClouds=cell(n,1);coarseClouds=fineClouds;
    fineSeconds=zeros(n,1);conversionSeconds=fineSeconds;counts=zeros(n,3);seen=false(n,1);
    for k=1:4
        a=load(fullfile(out,sprintf('inputs_%d.mat',k)));ids=a.frames;
        assert(a.processed==numel(ids) && ~any(seen(ids)),'Incomplete or overlapping input partitions.');
        fineClouds(ids)=a.fineClouds;coarseClouds(ids)=a.coarseClouds;seen(ids)=true;
        fineSeconds(ids)=a.perceptionSeconds;conversionSeconds(ids)=a.conversionSeconds;counts(ids,:)=a.counts;
    end
    assert(all(seen));
    differences=zeros(n,2);
    for k=1:n
        a=coarseClouds{k}.components;b=cached.clouds{k}.components;
        assert(isequal(a.semanticName,b.semanticName) && a.numComponents==b.numComponents,'Coarse semantics changed.');
        differences(k,:)=[max(abs(a.mean-b.mean),[],'all'),max(abs(a.covariance-b.covariance),[],'all')];
    end
    assert(all(differences<1e-8,'all'),'Fresh coarse inputs differ from the frozen baseline.');
    inputAudit=array2table([(1:n).',counts,fineSeconds,conversionSeconds,differences], ...
        VariableNames={'frame','curbPoints','polePoints','trafficSignPoints','finePerceptionSeconds', ...
        'fineConversionSeconds','coarseMeanDifferenceM','coarseCovarianceDifference'});
    writetable(inputAudit,fullfile(out,'input_audit.csv'));
    map=registrationSupport.projectSemanticProbabilityCloud(cached.map,2);
    motion=baseline.report.deadReckoning{:,{'x','y','psi'}};ref=c{:,{'referenceX','referenceY','referencePsi'}};
    cfg=baseline.cfg;variants=["coarse","fine","coarse_binary_quality"];runs=cell(0,1);
    classNames=["curb","pole","trafficSign"];
    for mode=["recursive","commonInitialGuess"]
        for variant=variants
            if mode=="commonInitialGuess" && variant=="coarse_binary_quality",continue;end
            rows=cell(n,width(c));history=[];state=c{1,{'predictedX','predictedY','predictedPsi'}};
            candidate=zeros(n,3);windows=zeros(n,3);classes=zeros(n,3);
            for k=1:n
                cloud=coarseClouds{k};perceptionMs=c.perceptionMs(k);
                if variant=="fine",cloud=fineClouds{k};perceptionMs=1000*(fineSeconds(k)+conversionSeconds(k));end
                if variant=="coarse_binary_quality",cloud.components.semanticProbability(:)=1;end
                initial=state;
                if mode=="commonInitialGuess"
                    initial=c{k,{'predictedX','predictedY','predictedPsi'}};
                elseif k>1
                    initial=compose(state,relative(motion(k-1,:),motion(k,:)));
                end
                timer=tic;local=selectMap(map,initial,100);selectionSeconds=toc(timer);
                timer=tic;
                [source,history,window]=updateLocalizationSourceWindow(cloud,c.timeSeconds(k),motion(k,:),history,cfg.sourceWindow);
                r=registerSemanticProbabilityCloud(local,source,initial,cfg.registration);registrationSeconds=toc(timer);
                candidate(k,:)=r.poseXYTheta;state=initial;
                event=registrationSupport.registrationPoseMeasurement(r,c.timeSeconds(k));
                information=r.information;if ~isempty(event),state=event.pose;information=event.information;end
                if variant=="coarse"
                    assert(norm(state(1:2)-c{k,{'x','y'}})<1e-5,'Coarse replay changed.');
                end
                e=state-ref(k,:);e(3)=wrap(e(3));pairs=0;
                if isfield(r,'correspondences'),pairs=height(r.correspondences);end
                rows(k,:)={k,c.timeSeconds(k),c.rosStamp(k),ref(k,1),ref(k,2),ref(k,3), ...
                    initial(1),initial(2),initial(3),state(1),state(2),state(3),r.accepted,r.reason, ...
                    norm(e(1:2)),rad2deg(e(3)),r.similarity,r.observableRank,r.iterations,pairs, ...
                    perceptionMs+1000*(registrationSeconds+selectionSeconds),perceptionMs, ...
                    1000*registrationSeconds,1000*selectionSeconds,0,information(1,1),information(1,2), ...
                    information(1,3),information(2,2),information(2,3),information(3,3),r.directionalAccepted};
                windows(k,:)=[window.frameCount,window.spanSeconds,window.componentCount];
                for j=1:3,classes(k,j)=nnz(cloud.components.semanticName==classNames(j));end
                if mod(k,200)==0,fprintf('%s %s %d/%d error %.3f m\n',variant,mode,k,n,norm(e(1:2)));end
            end
            destination=fullfile(out,variant+'_'+mode);if ~isfolder(destination),mkdir(destination);end
            report=struct('calls',cell2table(rows,VariableNames=c.Properties.VariableNames), ...
                'candidatePoses',array2table([(1:n).',candidate],VariableNames={'frame','x','y','psi'}), ...
                'sourceWindows',array2table([(1:n).',windows],VariableNames={'frame','scans','spanSeconds','components'}), ...
                'deadReckoning',baseline.report.deadReckoning, ...
                'componentCounts',array2table([(1:n).',classes],VariableNames={'frame','curb','pole','trafficSign'}));
            report.metadata=struct('variant',variant,'mode',mode,'freshRawPerception',true, ...
                'finePerception',variant=="fine",'fineFeatureProbability',1, ...
                'mapUnchanged',true,'registrationParametersUnchanged',true, ...
                'gridResolution',cfg.perception.coarseProbabilityCloud.resolution, ...
                'referenceXYOrYawInSource',false,'recordedKnownTilt',true, ...
                'sourceWindowMotion',"Independent wheel/gyro/lateral integration; no matching pose feedback", ...
                'initialization',"Common first reference plus [0.5,-0.4,2deg]; subsequent recursive motion predictions", ...
                'reference',"Recorded INSPVA; same drive also supplied map observations", ...
                'timing',"Perception partitions ran concurrently with two threads each; coarse time from baseline; not isolated benchmarking");
            if mode=="commonInitialGuess",report.metadata.initialization="Recorded coarse production prediction at each frame; diagnostic only";end
            writetable(report.calls,fullfile(destination,'calls.csv'));
            writetable(report.candidatePoses,fullfile(destination,'candidate_poses.csv'));
            writetable(report.sourceWindows,fullfile(destination,'source_windows.csv'));
            writetable(report.deadReckoning,fullfile(destination,'dead_reckoning.csv'));
            writetable(report.componentCounts,fullfile(destination,'component_counts.csv'));
            save(fullfile(destination,'report.mat'),'report','cfg');runs{end+1}=report; %#ok<AGROW>
        end
    end
    save(fullfile(out,'experiment.mat'),'runs','inputAudit','cfg','-v7.3');
end

function cloud=selectMap(cloud,pose,radius)
    c=cloud.components;keep=sum((c.mean-pose(1:2)).^2,2)<=radius^2;
    c.mean=c.mean(keep,:);c.covariance=c.covariance(:,:,keep);c.numComponents=nnz(keep);
    for name=["semanticName","mixtureWeight","repeatability"]
        if isfield(c,name),c.(name)=c.(name)(keep,:);end
    end
    cloud.components=c;
end

function p=relative(a,b)
    r=[cos(a(3)) -sin(a(3));sin(a(3)) cos(a(3))];p=[(b(1:2)-a(1:2))*r,wrap(b(3)-a(3))];
end

function p=compose(a,b)
    r=[cos(a(3)) -sin(a(3));sin(a(3)) cos(a(3))];p=[a(1:2)+b(1:2)*r.',wrap(a(3)+b(3))];
end

function a=wrap(a)
    a=atan2(sin(a),cos(a));
end
