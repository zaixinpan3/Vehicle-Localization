function probe_height_associations()
% probe_height_associations Hold seeds fixed and inspect historical sign ambiguity.
% Reference distances are evaluation proxies, not physical-object labels.
    setupVehicleLocalization;dest=fileparts(mfilename('fullpath'));
    cache=load('output/height_association_20260923/sources.mat');
    cfg=distributionRegistrationConfig();rows=cell(0,13);pairRows=cell(0,12);
    labels=["xy","conditional","marginal"];
    frames=[91 260 615 840 950:959];
    for k=frames
        seed=cache.calls{k,{'predictedX','predictedY','predictedPsi'}};
        ref=cache.calls{k,{'referenceX','referenceY','referencePsi'}};
        r=[cos(ref(3)) -sin(ref(3));sin(ref(3)) cos(ref(3))];
        for mode=1:3
            enabled=mode>1;cfg.relativeHeight.enabled=enabled;
            cfg.relativeHeight.candidateModel="marginal";
            if mode==2,cfg.relativeHeight.candidateModel="conditional";end
            a=matchLocalProbabilityCloud(cache.fixed,cache.sources{k},seed,cfg,[]);
            d=struct('enabled',false,'anchorCount',0,'offset',NaN,'offsetScatter',NaN);
            if enabled,d=a.height.relativeAssociation;end
            rows(end+1,:)={k,labels(mode),enabled,a.accepted,a.reason,norm(a.poseXYTheta(1:2)-ref(1:2)), ...
                norm(seed(1:2)-ref(1:2)),a.observableRank,d.enabled,d.anchorCount,d.offset,d.offsetScatter,a.matchingSeconds}; %#ok<AGROW>
            p=a.correspondences;
            for j=find(ismember(p.semanticName,["pole","trafficSign"])).'
                target=p.globalTarget(j);source=p.source(j);
                delta=cache.sources{k}.components.mean(source,:)*r.'+ref(1:2)-cache.fixed.components.mean(target,:);
                pairRows(end+1,:)={k,labels(mode),enabled,p.semanticName(j),source,target,norm(delta), ...
                    p.heightResidual(j),p.heightAssociationCost(j),p.squaredStandardizedResidual(j),p.robustWeight(j),a.accepted}; %#ok<AGROW>
            end
        end
    end
    metrics=cell2table(rows,VariableNames={'frame','model','zRequested','accepted','reason','errorM','seedErrorM', ...
        'rank','zEnabled','anchors','offsetM','offsetScatterM','seconds'});
    pairs=cell2table(pairRows,VariableNames={'frame','model','zRequested','class','source','target','referenceDistanceM', ...
        'heightResidualM','heightCost','planarStandardizedSquaredResidual','robustWeight','accepted'});
    writetable(metrics,fullfile(dest,'frozen_probes.csv'));writetable(pairs,fullfile(dest,'frozen_probe_pairs.csv'));
    disp(metrics);disp(pairs(pairs.frame>=950 & pairs.class=="trafficSign",:));
end
