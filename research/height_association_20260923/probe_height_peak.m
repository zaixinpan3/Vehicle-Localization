function probe_height_peak()
% probe_height_peak Separate association changes from causal prediction drift.
% Every mode uses the same XY baseline-selected seed; GNSS does not rank modes.
    setupVehicleLocalization;dest=fileparts(mfilename('fullpath'));
    cache=load('output/height_association_20260923/sources.mat');
    saved=load('output/height_association_20260923/xy.mat','estimate');
    cfg=distributionRegistrationConfig();rows=cell(0,7);pairRows=cell(0,9);
    labels=["xy","relative_z_objects","relative_z_signs","marginal_z_objects"];
    for k=[166 840]
        seed=saved.estimate.matchingResults{k}.initialPoseXYTheta;
        ref=cache.calls{k,{'referenceX','referenceY','referencePsi'}};
        for mode=1:4
            cfg.relativeHeight.candidateModel="conditional";
            if mode==4,cfg.relativeHeight.candidateModel="marginal";end
            cfg.relativeHeight.enabled=mode>1;cfg.relativeHeight.semanticNames=["pole","trafficSign"];
            if mode==3,cfg.relativeHeight.semanticNames="trafficSign";end
            result=matchLocalProbabilityCloud(cache.fixed,cache.sources{k},seed,cfg,[]);
            rows(end+1,:)={k,labels(mode),result.accepted,norm(result.poseXYTheta(1:2)-ref(1:2)), ...
                seed(1),seed(2),seed(3)}; %#ok<AGROW>
            p=result.correspondences;
            for j=find(ismember(p.semanticName,["pole","trafficSign"])).'
                s=p.source(j);t=p.globalTarget(j);
                offset=NaN;
                if mode>1 && cache.sources{k}.heightEvidence.available(s)
                    offset=result.height.relativeAssociation.offset;
                end
                pairRows(end+1,:)={k,labels(mode),s,t,p.semanticName(j), ...
                    cache.sources{k}.heightEvidence.mean(s,3)+offset, ...
                    cache.fixed.components.meanXYZ(t,3),p.heightResidual(j),p.heightAssociationCost(j)}; %#ok<AGROW>
            end
        end
    end
    summary=cell2table(rows,VariableNames={'frame','variant','accepted','errorM','seedX','seedY','seedPsi'});
    pairs=cell2table(pairRows,VariableNames={'frame','variant','source','target','class', ...
        'sourceZWithOffsetM','targetMeanZM','heightResidualM','heightCost'});
    writetable(summary,fullfile(dest,'peak_control.csv'));writetable(pairs,fullfile(dest,'peak_pairs.csv'));
    disp(summary);disp(pairs);
end
