function T=runGatedPyramid()
% runGatedPyramid Fine refinement accepted only within a trust radius of the coarse pose.
% Results on the canonical levels are seed independent, so the recorded seeds
% suffice. For each coarse radius the fine solve on the original clouds is
% gated by |fine - coarse| <= tau; otherwise the coarse pose is kept.
    setupVehicleLocalization;dest=fileparts(mfilename('fullpath'));
    C=load('output/mncav_coarse_localization_20260924/sources.mat','fixed','sources','calls');c=C.calls;n=height(c);
    cfg=distributionRegistrationConfig();ref=[c.referenceX,c.referenceY,c.referencePsi];seed=[c.predictedX,c.predictedY,c.predictedPsi];
    rows=cell(0,10);
    for coarseRadius=[1.5 2.0]
        map=canonicalizeSemanticCloud(C.fixed,coarseRadius);sources=cellfun(@(s)canonicalizeSemanticCloud(s,0.5),C.sources,UniformOutput=false);
        coarseErr=zeros(n,1);fineErr=zeros(n,1);shift=zeros(n,1);ok=false(n,1);
        for k=1:n
            a=matchLocalProbabilityCloud(map,sources{k},seed(k,:),cfg,[]);s=seed(k,:);if a.accepted||a.directionalAccepted,s=a.poseXYTheta;end
            b=matchLocalProbabilityCloud(C.fixed,C.sources{k},s,cfg,[]);
            coarseErr(k)=norm(a.poseXYTheta(1:2)-ref(k,1:2));fineErr(k)=norm(b.poseXYTheta(1:2)-ref(k,1:2));
            shift(k)=norm(b.poseXYTheta(1:2)-a.poseXYTheta(1:2));ok(k)=a.accepted&&b.accepted;
        end
        for tau=[0.15 0.2 0.3 0.4 0.6 Inf]
            e=coarseErr;use=ok&shift<=tau;e(use)=fineErr(use);
            rows(end+1,:)={coarseRadius,tau,100*rms(e(ok)),100*median(e(ok)),100*prctile(e(ok),95),100*max(e(ok)),nnz(e(ok)>.3),nnz(e(ok)>.5),100*e(851),nnz(use)}; %#ok<AGROW>
            fprintf('coarse r=%.1f tau=%.2f: RMSE %.2f median %.2f P95 %.2f max %.2f >30cm %d >50cm %d f851 %.1f cm (fine used in %d frames)\n',rows{end,:});
        end
        fprintf('  |fine-coarse| distribution: median %.2f P90 %.2f P95 %.2f max %.2f m\n',median(shift),prctile(shift,90),prctile(shift,95),max(shift));
    end
    T=cell2table(rows,VariableNames={'coarseRadiusM','tauM','rmseCm','medianCm','p95Cm','maxCm','above30cm','above50cm','frame851Cm','fineUsedFrames'});
    writetable(T,fullfile(dest,'gated_pyramid.csv'));
end
