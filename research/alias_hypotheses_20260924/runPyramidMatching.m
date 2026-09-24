function results=runPyramidMatching()
% runPyramidMatching Coarse-to-fine registration on a canonicalized map pyramid.
% Level 1 solves on the merged clouds (basin selection); level 2 refines on
% the original clouds from the level-1 pose (precision). Frozen recorded
% seeds, the reference-seeded ceiling and the closed-loop replay are reported
% for each level; the closed loop advances on the finest accepted pose.
    setupVehicleLocalization;dest=fileparts(mfilename('fullpath'));out='output/alias_hypotheses_20260924';
    C=load('output/mncav_coarse_localization_20260924/sources.mat','fixed','sources','calls');c=C.calls;n=height(c);
    M=load('output/mncav_coarse_localization_20260924/matching/report.mat','report');motion=M.report.deadReckoning{:,{'x','y','psi'}};
    cfg=distributionRegistrationConfig();ref=[c.referenceX,c.referenceY,c.referencePsi];seed=[c.predictedX,c.predictedY,c.predictedPsi];
    configs={[1.5 0.5];[2.0 0.5];[1.5 0.5;0.8 0.3];[2.0 0.5;0 0];[1.5 0.5;0 0]};rows=cell(0,14);results=struct();
    for q=1:numel(configs)
        levels=configs{q};L=size(levels,1);maps=cell(L,1);sources=cell(L,1);
        for l=1:L
            maps{l}=canonicalizeSemanticCloud(C.fixed,levels(l,1));
            sources{l}=cellfun(@(s)canonicalizeSemanticCloud(s,levels(l,2)),C.sources,UniformOutput=false);
        end
        label=strjoin(arrayfun(@(l)sprintf('%.1f/%.1f',levels(l,1),levels(l,2)),1:L,UniformOutput=false),' -> ');
        frozen=zeros(n,L);fa=false(n,L);ceiling=zeros(n,L);ca=false(n,L);
        for k=1:n
            [e,a]=pyramid(maps,sources,k,seed(k,:),cfg,ref);frozen(k,:)=e;fa(k,:)=a;
            [e,a]=pyramid(maps,sources,k,ref(k,:),cfg,ref);ceiling(k,:)=e;ca(k,:)=a;
        end
        loop=zeros(n,L);la=false(n,L);seconds=zeros(n,1);state=seed(1,:);
        for k=1:n
            timer=tic;delta=[0 0 0];if k>1,delta=relative(motion(k-1,:),motion(k,:));end
            s=compose(state,delta);[e,a,poses]=pyramid(maps,sources,k,s,cfg,ref);seconds(k)=toc(timer);loop(k,:)=e;la(k,:)=a;
            finest=find(a,1,'last');if isempty(finest),state=s;else,state=poses(finest,:);end
        end
        for l=1:L
            rows(end+1,:)={label,l,levels(l,1),levels(l,2),100*rms(frozen(fa(:,l),l)),nnz(frozen(fa(:,l),l)>.3),100*rms(ceiling(ca(:,l),l)), ...
                100*rms(loop(la(:,l),l)),100*prctile(loop(la(:,l),l),95),100*max(loop(la(:,l),l)),nnz(loop(la(:,l),l)>.3),nnz(loop(la(:,l),l)>.5),100*loop(851,l),1e3*mean(seconds)}; %#ok<AGROW>
            fprintf('%-22s level %d (map %.1f source %.1f): frozen %.2f (>30 %d) ceiling %.2f | loop RMSE %.2f P95 %.2f max %.2f >30 %d >50 %d f851 %.1f | %.1f ms/frame total\n',rows{end,:});
        end
        results.(sprintf('config%d',q))=struct('levels',levels,'frozen',frozen,'ceiling',ceiling,'loop',loop,'loopAccepted',la,'seconds',seconds);
    end
    T=cell2table(rows,VariableNames={'pyramid','level','mapRadiusM','sourceRadiusM','frozenRmseCm','frozenAbove30cm','ceilingRmseCm', ...
        'loopRmseCm','loopP95Cm','loopMaxCm','loopAbove30cm','loopAbove50cm','loopFrame851Cm','msPerFrame'});
    writetable(T,fullfile(dest,'pyramid_matching.csv'));save(fullfile(out,'pyramid.mat'),'results','T','-v7.3');
end

function [e,accepted,poses]=pyramid(maps,sources,k,s,cfg,ref)
    L=numel(maps);e=zeros(1,L);accepted=false(1,L);poses=repmat(s,L,1);
    for l=1:L
        r=matchLocalProbabilityCloud(maps{l},sources{l}{k},s,cfg,[]);
        e(l)=norm(r.poseXYTheta(1:2)-ref(k,1:2));accepted(l)=r.accepted;poses(l,:)=r.poseXYTheta;
        if r.accepted || r.directionalAccepted,s=r.poseXYTheta;end
    end
end

function d=relative(a,b)
    R=[cos(a(3)) sin(a(3));-sin(a(3)) cos(a(3))];d=[(b(1:2)-a(1:2))*R.',wrap(b(3)-a(3))];
end

function p=compose(a,b)
    R=[cos(a(3)) -sin(a(3));sin(a(3)) cos(a(3))];p=[a(1:2)+b(1:2)*R.',wrap(a(3)+b(3))];
end

function x=wrap(x)
    x=atan2(sin(x),cos(x));
end
