function runReferenceFree785(repoRoot)
% runReferenceFree785 Reproduce the 7.85 cm runtime and remove reference seeds.
% Run from the unmodified d650208 snapshot prepared under ignored output.
    setupVehicleLocalization();maxNumCompThreads(4);
    out=fullfile(repoRoot,'output','reference_free_785_20260920');
    assert(contains(which('registerSemanticProbabilityCloud'),'reference_free_785_20260920/historical_runtime'));
    assert(contains(which('runSynchronousLocalizationObserver'),'reference_free_785_20260920/historical_runtime'));
    baseline=load(fullfile(repoRoot,'output/mncav_bestpos_alignment_20260917/experiment.mat'));
    saved=load(fullfile(repoRoot,'output/saved_perception_inspva_20260915/experiment.mat'), ...
        'clouds','reference','time','cfg');
    map=load(fullfile(repoRoot,'output/mississippi_mapping_inspva_20260915/probability_cloud.mat'),'cloud');
    [present,index]=ismembertol(baseline.data.highRate.time,saved.time,1e-9,DataScale=1);assert(all(present));
    clouds=saved.clouds(index);n=numel(index);assert(n==1169 && isequal(index(:),(1:1169).'));
    registration=saved.cfg;assert(isequaln(registration,distributionRegistrationConfig()));
    reproduced=runSynchronousLocalizationObserver(baseline.data,baseline.cfg,baseline.lateral);
    observerDifference=max(abs(reproduced.pose-baseline.runs{1}.estimate.pose),[],'all');
    assert(observerDifference<1e-10,'Historical observer reproduction failed.');
    full=false(n,1);pose=nan(n,3);information=zeros(3,3,n);similarity=zeros(n,1);
    for k=1:n
        r=registerSemanticProbabilityCloud(map.cloud,clouds{k},saved.reference(index(k),:),registration);
        full(k)=r.accepted;similarity(k)=r.similarity;
        if r.accepted,pose(k,:)=r.poseXYTheta;information(:,:,k)=r.information;end
    end
    assert(isequal(full,baseline.data.lidar.valid));
    matchDifference=max(abs(pose(full,:)-baseline.data.lidar.pose(full,:)),[],'all');
    infoDifference=max(abs(information(:,:,full)-baseline.data.lidar.information(:,:,full)),[],'all');
    assert(matchDifference<1e-7 && infoDifference<1e-6,'Historical matching reproduction failed.');
    verification=struct('observerMaximumDifference',observerDifference, ...
        'matchingMaximumDifference',matchDifference,'informationMaximumDifference',infoDifference, ...
        'historicalFullMatches',nnz(full),'frames',n,'lastUncoveredRawFrame',1170);
    fprintf('Historical reproduction passed: observer %.3g, matching %.3g; %d matches.\n',observerDifference,matchDifference,nnz(full));
    bootstrap=bootstrapFromGnss785(map.cloud,clouds{1},baseline.data.gnss,baseline.cfg,registration);
    writetable(bootstrap.candidates,fullfile(out,'bootstrap_candidates.csv'));
    fprintf('GNSS/map bootstrap selected heading candidate %d, similarity %.5f.\n', ...
        bootstrap.selected,bootstrap.result.similarity);
    runs=cell(2,1);modes=["matching_prediction","fusion_prediction"];
    for j=1:2
        % Neither reference nor historical LiDAR state is passed into this
        % replay. Clear historical initial state and measurement payloads.
        inputs=baseline.data;inputs.lidar.pose(:)=NaN;inputs.lidar.valid(:)=false;
        inputs.lidar.information(:)=0;configuration=baseline.cfg;configuration.initialState=[];
        result=replayWithoutReference785(map.cloud,clouds,inputs,baseline.lateral, ...
            configuration,registration,bootstrap,modes(j));
        runs{j}=result;folder=fullfile(out,modes(j));if ~isfolder(folder),mkdir(folder);end
        writetable(result.calls,fullfile(folder,'matching.csv'));
        exportPose(folder,'fused.csv',result.estimate,baseline.reference,index);
        exportPose(folder,'gnss_only.csv',result.gnssEstimate,baseline.reference,index);
        save(fullfile(folder,'experiment.mat'),'result','-v7.3');
    end
    exportPose(out,'historical_fused.csv',reproduced,baseline.reference,index);
    exportPose(out,'historical_gnss_only.csv',baseline.runs{3}.estimate,baseline.reference,index);
    reference=baseline.reference;time=baseline.data.highRate.time;motion=baseline.data.highRate;
    motion.lateralVelocity=baseline.lateral.lateralVelocity;
    writetable(struct2table(motion),fullfile(out,'motion.csv'));
    historical=table(index,time,full,pose(:,1),pose(:,2),pose(:,3),similarity, ...
        VariableNames={'frame','time','accepted','x','y','psi','similarity'});
    writetable(historical,fullfile(out,'historical_matching.csv'));
    save(fullfile(out,'experiment.mat'),'runs','bootstrap','verification','reference','time','registration','-v7.3');
    fid=fopen(fullfile(out,'verification.json'),'w');cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(verification,PrettyPrint=true));
end

function exportPose(folder,name,estimate,reference,index)
    p=estimate.pose;e=p-reference;e(:,3)=atan2(sin(e(:,3)),cos(e(:,3)));
    data=table(index,estimate.time,p(:,1),p(:,2),p(:,3),reference(:,1),reference(:,2),reference(:,3), ...
        hypot(e(:,1),e(:,2)),rad2deg(e(:,3)),estimate.diagnostics.mode, ...
        VariableNames={'frame','time','x','y','psi','referenceX','referenceY','referencePsi', ...
        'positionErrorM','yawErrorDeg','mode'});
    writetable(data,fullfile(folder,name));
end
