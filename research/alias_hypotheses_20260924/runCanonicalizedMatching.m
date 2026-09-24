function results=runCanonicalizedMatching()
% runCanonicalizedMatching Current single-seed solver on canonicalized clouds.
% Same-class point components closer than the map/source radius are merged
% by moment matching before registration. Frozen recorded seeds, the
% reference-seeded ceiling and the closed-loop recursive replay are reported.
    setupVehicleLocalization;dest=fileparts(mfilename('fullpath'));out='output/alias_hypotheses_20260924';
    C=load('output/mncav_coarse_localization_20260924/sources.mat','fixed','sources','calls');c=C.calls;n=height(c);
    M=load('output/mncav_coarse_localization_20260924/matching/report.mat','report');motion=M.report.deadReckoning{:,{'x','y','psi'}};
    cfg=distributionRegistrationConfig();ref=[c.referenceX,c.referenceY,c.referencePsi];seed=[c.predictedX,c.predictedY,c.predictedPsi];
    configs=[0 0;0.6 0;1.0 0;1.0 0.5;1.5 0.5;1.0 0.35];rows=cell(size(configs,1),16);
    for q=1:size(configs,1)
        mapRadius=configs(q,1);sourceRadius=configs(q,2);
        map=canonicalizeSemanticCloud(C.fixed,mapRadius);names=string(map.components.semanticName);
        sources=cellfun(@(s)canonicalizeSemanticCloud(s,sourceRadius),C.sources,UniformOutput=false);
        pairs=aliasPairs(map);
        frozen=zeros(n,1);ceiling=zeros(n,1);fa=false(n,1);ca=false(n,1);
        for k=1:n
            r=matchLocalProbabilityCloud(map,sources{k},seed(k,:),cfg,[]);frozen(k)=norm(r.poseXYTheta(1:2)-ref(k,1:2));fa(k)=r.accepted;
            r=matchLocalProbabilityCloud(map,sources{k},ref(k,:),cfg,[]);ceiling(k)=norm(r.poseXYTheta(1:2)-ref(k,1:2));ca(k)=r.accepted;
        end
        [loop,la,seconds]=closedLoop(map,sources,motion,cfg,seed(1,:),ref);
        rows(q,:)={mapRadius,sourceRadius,nnz(names=="pole"),nnz(names=="trafficSign"),pairs, ...
            100*rms(frozen(fa)),nnz(frozen(fa)>.3),100*rms(ceiling(ca)),100*max(ceiling(ca)), ...
            100*rms(loop(la)),100*prctile(loop(la),95),100*max(loop(la)),nnz(loop(la)>.3),nnz(loop(la)>.5),100*loop(851),1e3*mean(seconds)};
        fprintf('map r=%.2f source r=%.2f: poles %d signs %d alias pairs %d | frozen RMSE %.2f (>30cm %d) | ceiling RMSE %.2f max %.2f | closed loop RMSE %.2f P95 %.2f max %.2f >30cm %d >50cm %d frame851 %.1f cm | %.1f ms\n',rows{q,:});
        results.(sprintf('map%03d_source%03d',round(100*mapRadius),round(100*sourceRadius)))=table((1:n).',frozen,fa,ceiling,ca,loop,la,VariableNames={'frame','frozenM','frozenAccepted','ceilingM','ceilingAccepted','loopM','loopAccepted'});
    end
    T=cell2table(rows,VariableNames={'mapRadiusM','sourceRadiusM','mapPoles','mapSigns','aliasPairs','frozenRmseCm','frozenAbove30cm', ...
        'ceilingRmseCm','ceilingMaxCm','loopRmseCm','loopP95Cm','loopMaxCm','loopAbove30cm','loopAbove50cm','loopFrame851Cm','msPerFrame'});
    writetable(T,fullfile(dest,'canonicalized_matching.csv'));save(fullfile(out,'canonicalized.mat'),'results','T','-v7.3');
end

function count=aliasPairs(map)
    c=map.components;names=string(c.semanticName);count=0;
    for nm=["pole","trafficSign"]
        idx=find(names==nm);m=c.mean(idx,1:2);w=c.mixtureWeight(idx);
        for i=1:numel(idx)-1
            r=vecnorm(m(i+1:end,:)-m(i,:),2,2);count=count+nnz(r>0.25 & r<=1.5 & w(i+1:end)>=0.05*max(w) & w(i)>=0.05*max(w));
        end
    end
end

function [e,accepted,seconds]=closedLoop(map,sources,motion,cfg,state,ref)
    n=numel(sources);e=zeros(n,1);accepted=false(n,1);seconds=zeros(n,1);
    for k=1:n
        timer=tic;delta=[0 0 0];if k>1,delta=relative(motion(k-1,:),motion(k,:));end
        s=compose(state,delta);r=matchLocalProbabilityCloud(map,sources{k},s,cfg,[]);seconds(k)=toc(timer);
        e(k)=norm(r.poseXYTheta(1:2)-ref(k,1:2));accepted(k)=r.accepted;
        if r.accepted || r.directionalAccepted,state=r.poseXYTheta;else,state=s;end
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
