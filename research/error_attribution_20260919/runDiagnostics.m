function runDiagnostics()
% runDiagnostics Controlled offline attribution; never change runtime defaults.
% Reference initialization/transport are diagnostic oracles, not online inputs.
    setupVehicleLocalization(); maxNumCompThreads(8);
    out='output/error_attribution_20260919';
    if ~isfolder(out), mkdir(out); end
    s=load('output/matching_refinement_20260919/inputs.mat');
    s.map=registrationSupport.projectSemanticProbabilityCloud(s.map,2);
    current=load('output/matching_refinement_20260919/validated/recursive/report.mat');
    c=current.report.calls; cfg=current.cfg.registration;
    ref=c{:,{'referenceX','referenceY','referencePsi'}};
    motion=current.report.deadReckoning{:,{'x','y','psi'}};
    variants=["reproduce","exact_initial","exact_window_motion","single_scan", ...
        "without_curb","without_pole","without_sign","uniform_map_prior"];
    rows=cell(height(c)*numel(variants),13); row=0; history=[]; oracleHistory=[];
    details=cell(0,13); snapshots=cell(0,1);
    selected=[30 75 85 90 92 94 100 200 425 600 800 820 830 840 842 900 966 1100];
    for k=1:height(c)
        [source,history]=updateLocalizationSourceWindow(s.clouds{k},c.timeSeconds(k),motion(k,:),history);
        [oracle,oracleHistory]=updateLocalizationSourceWindow(s.clouds{k},c.timeSeconds(k),ref(k,:),oracleHistory);
        initial=c{k,{'predictedX','predictedY','predictedPsi'}};
        map=subsetCloud(s.map,sum((s.map.components.mean-initial(1:2)).^2,2)<=100^2);
        for v=1:numel(variants)
            moving=source; fixed=map; seed=initial;
            switch variants(v)
                case "exact_initial", seed=ref(k,:);
                case "exact_window_motion", moving=oracle;
                case "single_scan", moving=s.clouds{k};
                case "without_curb", moving=subsetCloud(source,source.components.semanticName~="curb");
                case "without_pole", moving=subsetCloud(source,source.components.semanticName~="pole");
                case "without_sign", moving=subsetCloud(source,source.components.semanticName~="trafficSign");
                case "uniform_map_prior", fixed.components.mixtureWeight(:)=1/fixed.components.numComponents;
            end
            r=registerSemanticProbabilityCloud(fixed,moving,seed,cfg);
            e=r.poseXYTheta-ref(k,:); e(3)=atan2(sin(e(3)),cos(e(3)));
            rot=[cos(ref(k,3)) -sin(ref(k,3));sin(ref(k,3)) cos(ref(k,3))]; body=e(1:2)*rot;
            n=0; if isfield(r,'correspondences'), n=height(r.correspondences); end
            row=row+1;
            rows(row,:)={k,variants(v),norm(e(1:2)),body(1),body(2),rad2deg(e(3)), ...
                r.accepted,r.reason,r.observableRank,r.similarity,n, ...
                norm(r.poseXYTheta(1:2)-initial(1:2)),norm(r.poseXYTheta(1:2)-c{k,{'x','y'}})};
            if v==1 && c.accepted(k)
                assert(norm(r.poseXYTheta(1:2)-c{k,{'x','y'}})<1e-5,'Cached replay mismatch.');
            end
        end
        if ismember(k,selected)
            held=cfg; held.stepTolerance=1e6;
            for mode=["reference","estimate"]
                pose=ref(k,:); if mode=="estimate", pose=c{k,{'x','y','psi'}}; end
                r=registerSemanticProbabilityCloud(map,source,pose,held);
                p=r.correspondences; rot=[cos(pose(3)) -sin(pose(3));sin(pose(3)) cos(pose(3))];
                delta=source.components.mean(p.source,:)*rot.'+pose(1:2)-map.components.mean(p.target,:);
                body=delta*[cos(ref(k,3)) -sin(ref(k,3));sin(ref(k,3)) cos(ref(k,3))];
                cost=cfg.geometric.robustStandardizedDistance^2*log1p(p.squaredStandardizedResidual/cfg.geometric.robustStandardizedDistance^2);
                for name=unique(p.semanticName).'
                    keep=p.semanticName==name; w=p.robustWeight(keep); w=w/sum(w);
                    d=r.classDiagnostics(r.classDiagnostics.semanticName==name,:);
                    details(end+1,:)={k,mode,name,nnz(keep),numel(unique(p.target(keep))), ...
                        sum(p.weight(keep)),sum(p.robustWeight(keep)),sum(p.weight(keep).*cost(keep)), ...
                        sum(w.*body(keep,1)),sum(w.*body(keep,2)), ...
                        d.observableCorrection,d.informationWeightedCorrection,min(eig(r.scaledCurvature))/max(eig(r.scaledCurvature))}; %#ok<AGROW>
                end
                snapshots{end+1}=struct('frame',k,'mode',mode,'source',source,'map',map,'result',r,'reference',ref(k,:)); %#ok<AGROW>
            end
        end
        if mod(k,100)==0, fprintf('Attribution %d/%d\n',k,height(c)); end
    end
    results=cell2table(rows,VariableNames={'frame','variant','candidateErrorM','longitudinalErrorM', ...
        'lateralErrorM','yawErrorDeg','accepted','reason','rank','similarity','matches','correctionM','differenceFromProductionM'});
    pairDetails=cell2table(details,VariableNames={'frame','poseMode','semanticName','pairs','uniqueMapTargets', ...
        'classWeight','robustWeight','robustScore','meanLongitudinalResidualM','meanLateralResidualM', ...
        'classCorrection','weightedClassCorrection','eigenRatio'});
    writetable(results,fullfile(out,'matching_controls.csv'));
    writetable(pairDetails,fullfile(out,'pair_details.csv'));
    save(fullfile(out,'diagnostics.mat'),'results','pairDetails','snapshots','-v7.3');
end

function cloud=subsetCloud(cloud,keep)
% Keep component arrays synchronized, including optional map metadata.
    c=cloud.components; n=c.numComponents;
    for field=string(fieldnames(c)).'
        a=c.(field);
        if field=="numComponents", continue; end
        if contains(lower(field),"covariance") && size(a,3)==n
            c.(field)=a(:,:,keep);
        elseif size(a,1)==n
            c.(field)=a(keep,:);
        end
    end
    c.numComponents=nnz(keep); cloud.components=c;
end
