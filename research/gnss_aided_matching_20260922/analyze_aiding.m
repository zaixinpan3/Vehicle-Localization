function analyze_aiding()
% analyze_aiding Verify paired populations, peak frames and uncertainty export.
    setupVehicleLocalization;dest=fileparts(mfilename('fullpath'));out='output/gnss_aided_matching_20260922';
    saved=load(fullfile(out,'closed_loop.mat'));cache=load(fullfile(out,'sources.mat'));
    raw=load(fullfile(out,'frozen.mat'));n=numel(saved.aided.time);c=cache.calls(1:n,:);
    ref=c{:,{'referenceX','referenceY','referencePsi'}};
    current=saved.aided.matchingResults;valid=cellfun(@(r)r.accepted,current);
    poses=cell2mat(cellfun(@(r)r.poseXYTheta,current,UniformOutput=false));
    error=vecnorm(poses(:,1:2)-ref(:,1:2),2,2);common=valid & c.accepted;
    rows=cell(0,8);
    for scenario=["original_matching","aided_matching","original_fusion","aided_fusion","gnss_only"]
        switch scenario
            case "original_matching",e=c.positionErrorM;
            case "aided_matching",e=error;
            case "original_fusion",e=vecnorm(saved.baseline.position-ref(:,1:2),2,2);
            case "aided_fusion",e=vecnorm(saved.aided.position-ref(:,1:2),2,2);
            otherwise,e=vecnorm(saved.gnssOnly.position-ref(:,1:2),2,2);
        end
        for population=["common_accepted_frames","after_2_seconds"]
            use=common;if population=="after_2_seconds",use=c.timeSeconds>=2;if contains(scenario,"matching"),use=use & common;end,end
            ids=find(use);[peak,ix]=max(e(use));
            rows(end+1,:)={scenario,population,nnz(use),rms(e(use)),prctile(e(use),95),peak,c.frame(ids(ix)),nnz(e(use)>.3)}; %#ok<AGROW>
        end
    end
    paired=cell2table(rows,VariableNames={'variant','population','frames','rmseM','p95M','maxM','maxFrame','above30cm'});
    writetable(paired,fullfile(dest,'paired_metrics.csv'));
    aidCounts=zeros(n,6);maxInformationIncrease=0;reproduction=0;
    for k=1:n
        r=current{k};a=r.positionAiding;
        aidCounts(k,1:3)=[a.used,a.candidateCount,a.selected];
        if a.used
            aidCounts(k,4:6)=[max(a.relativeSupport),sum(a.geometricallyValid),max(eig(a.betweenHypothesisSecondMoment))];
        end
        if valid(k)
            seed=saved.aided.matchingSeeds(k,:);
            if a.used,seed=a.seedPoses(a.selected,:);end
            conditional=matchLocalProbabilityCloud(cache.fixed,cache.sources{k},seed,distributionRegistrationConfig(),[]);
            maxInformationIncrease=max(maxInformationIncrease,max(eig(r.information-conditional.information)));
        end
    end
    % Replay original failed/peak frames without aid to verify the frozen baseline.
    for k=[28 169 384 615 840 847 959 1137 1170]
        row=cache.calls(k,:);r=matchLocalProbabilityCloud(cache.fixed,cache.sources{k}, ...
            [row.predictedX row.predictedY row.predictedPsi],distributionRegistrationConfig(),[]);
        reproduction=max(reproduction,max(abs(r.poseXYTheta-[row.x row.y row.psi])));
    end
    assert(reproduction<1e-7);
    assert(maxInformationIncrease<1e-8,'Aiding must not add information.');
    reject=table(c.frame(~valid),string(cellfun(@(r)r.reason,current(~valid),UniformOutput=false)),error(~valid), ...
        VariableNames={'frame','reason','unpublishedCandidateErrorM'});
    writetable(reject,fullfile(dest,'rejected_frames.csv'));
    writetable(array2table([c.frame,aidCounts],VariableNames={'frame','aidUsed','candidates','selected','maxRelativeSupport','validCandidates','maxModeSecondMoment'}), ...
        fullfile(dest,'hypothesis_diagnostics.csv'));
    summary=struct('commonAcceptedFrames',nnz(common),'baselineReproductionMaximum',reproduction, ...
        'conditionalInformationIncreaseMaximum',maxInformationIncrease, ...
        'closedLoopRejectedFrames',c.frame(~valid).','extraSeedEvaluations',sum(aidCounts(:,2)-1), ...
        'secondCandidateSelected',nnz(aidCounts(:,3)==2),'frozenRejectedFrames',find(~raw.frozenValid).', ...
        'remainingPeakFrame',840,'referenceUsedForSelection',false);
    fid=fopen(fullfile(dest,'audit.json'),'w');cleanup=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(summary,PrettyPrint=true));
    disp(paired);disp(reject);disp(summary);
end
