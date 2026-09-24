function summary=runClosedLoopHypotheses(modes)
% runClosedLoopHypotheses LiDAR-only recursive replays with alias hypotheses.
% Seeds are the previous accepted pose plus one frame of recorded wheel/gyro/
% lateral odometry, exactly as the production recursive replay. Modes:
%   current      single-seed solve (must reproduce the recorded replay)
%   alias_select alias seeds, per-frame selection by similarity
%   alias_track  alias seeds, two tracked modes with discounted cumulative score
% Reference poses are used for evaluation only.
    if nargin<1,modes=["current","alias_select","alias_track"];end
    setupVehicleLocalization;dest=fileparts(mfilename('fullpath'));out='output/alias_hypotheses_20260924';
    C=load('output/mncav_coarse_localization_20260924/sources.mat','fixed','sources','calls');c=C.calls;n=height(c);
    M=load('output/mncav_coarse_localization_20260924/matching/report.mat','report');motion=M.report.deadReckoning{:,{'x','y','psi'}};
    cfg=distributionRegistrationConfig();ref=[c.referenceX,c.referenceY,c.referencePsi];
    lambda=0.95;maximumModes=2;mergeDistance=0.10;screenRatio=0.7;scale=[1;1;1/cfg.yawLeverArm];
    results=struct();
    for mode=string(modes)
        pose=zeros(n,3);accepted=false(n,1);sim=zeros(n,1);seconds=zeros(n,1);candidates=zeros(n,1);seedErr=zeros(n,1);
        state=[c.predictedX(1),c.predictedY(1),c.predictedPsi(1)];
        tracks=struct('pose',state,'score',0);   % alias_track: leading mode first
        for k=1:n
            timer=tic;
            if k>1,delta=relative(motion(k-1,:),motion(k,:));else,delta=[0 0 0];end
            switch mode
                case "current"
                    seed=compose(state,delta);r=matchLocalProbabilityCloud(C.fixed,C.sources{k},seed,cfg,[]);
                    best=r;candidates(k)=1;
                case "alias_select"
                    seed=compose(state,delta);r=matchLocalProbabilityCloud(C.fixed,C.sources{k},seed,cfg,[]);
                    list={r};aliasSeeds=enumerateMapAliasSeeds(C.fixed,C.sources{k},seed);
                    for j=1:size(aliasSeeds,1),list{end+1}=matchLocalProbabilityCloud(C.fixed,C.sources{k},aliasSeeds(j,:),cfg,[]);end %#ok<AGROW>
                    best=list{1};bestSim=-inf;
                    for j=1:numel(list),if list{j}.accepted && list{j}.similarity>bestSim,bestSim=list{j}.similarity;best=list{j};end,end
                    candidates(k)=numel(list);
                case "alias_screen"
                    % One linearization per alias seed screens the candidates;
                    % at most two survivors are refined by the full solver.
                    seed=compose(state,delta);r=matchLocalProbabilityCloud(C.fixed,C.sources{k},seed,cfg,[]);
                    list={r};aliasSeeds=enumerateMapAliasSeeds(C.fixed,C.sources{k},seed);
                    if ~isempty(aliasSeeds)
                        [fixedLocal,~]=selectLocalProbabilityCloud(C.fixed,seed,cfg.localMapRadius);
                        model=prepareSemanticRegistrationGeometry(fixedLocal,C.sources{k},seed,cfg);
                        screened=zeros(size(aliasSeeds,1),1);
                        for j=1:numel(screened)
                            probe=model.linearize([aliasSeeds(j,1:2)-seed(1:2),aliasSeeds(j,3)],scale);screened(j)=probe.similarity;
                        end
                        [screened,order]=sort(screened,'descend');
                        survivors=order(screened>=screenRatio*r.similarity);
                        for j=survivors(1:min(2,numel(survivors))).'
                            list{end+1}=matchLocalProbabilityCloud(C.fixed,C.sources{k},aliasSeeds(j,:),cfg,[]); %#ok<AGROW>
                        end
                    end
                    best=list{1};bestSim=-inf;
                    for j=1:numel(list),if list{j}.accepted && list{j}.similarity>bestSim,bestSim=list{j}.similarity;best=list{j};end,end
                    candidates(k)=numel(list);
                case "alias_track"
                    % Propagate every tracked mode; alias seeds only around the leader.
                    list={};parent=[];
                    for h=1:numel(tracks)
                        seed=compose(tracks(h).pose,delta);r=matchLocalProbabilityCloud(C.fixed,C.sources{k},seed,cfg,[]);
                        list{end+1}=r;parent(end+1)=h; %#ok<AGROW>
                        if h==1
                            aliasSeeds=enumerateMapAliasSeeds(C.fixed,C.sources{k},seed);
                            for j=1:size(aliasSeeds,1)
                                list{end+1}=matchLocalProbabilityCloud(C.fixed,C.sources{k},aliasSeeds(j,:),cfg,[]);parent(end+1)=h; %#ok<AGROW>
                            end
                        end
                    end
                    candidates(k)=numel(list);seed=compose(tracks(1).pose,delta);
                    % Score accepted candidates, merge coincident poses, keep the best modes.
                    scored=struct('pose',{},'score',{},'result',{});
                    for j=1:numel(list)
                        r=list{j};if ~r.accepted,continue;end
                        s=lambda*tracks(parent(j)).score-2*log(max(r.similarity,realmin));
                        merged=false;
                        for q=1:numel(scored)
                            if norm(scored(q).pose(1:2)-r.poseXYTheta(1:2))<mergeDistance
                                if s<scored(q).score,scored(q)=struct('pose',r.poseXYTheta,'score',s,'result',r);end
                                merged=true;break;
                            end
                        end
                        if ~merged,scored(end+1)=struct('pose',r.poseXYTheta,'score',s,'result',r);end %#ok<AGROW>
                    end
                    if isempty(scored)
                        % No accepted candidate: every mode coasts on odometry.
                        best=list{1};for h=1:numel(tracks),tracks(h).pose=compose(tracks(h).pose,delta);end
                    else
                        [~,order]=sort([scored.score]);scored=scored(order(1:min(maximumModes,numel(scored))));
                        best=scored(1).result;tracks=struct('pose',{scored.pose},'score',{scored.score});
                        shift=tracks(1).score;for h=1:numel(tracks),tracks(h).score=tracks(h).score-shift;end
                    end
            end
            seconds(k)=toc(timer);
            pose(k,:)=best.poseXYTheta;accepted(k)=best.accepted;sim(k)=best.similarity;seedErr(k)=norm(seed(1:2)-ref(k,1:2));
            % The recorded replay also advances on directional acceptance.
            if best.accepted || best.directionalAccepted,state=best.poseXYTheta;else,state=seed;end
            if mod(k,200)==0,fprintf('%s %d/%d\n',mode,k,n);end
        end
        e=vecnorm(pose(:,1:2)-ref(:,1:2),2,2);yaw=rad2deg(atan2(sin(pose(:,3)-ref(:,3)),cos(pose(:,3)-ref(:,3))));
        results.(mode)=table((1:n).',e,accepted,sim,seedErr,candidates,seconds,yaw,VariableNames={'frame','errorM','accepted','similarity','seedErrorM','candidates','seconds','yawErrorDeg'});
        a=accepted;
        fprintf('%-13s accepted %d, RMSE %.2f cm, median %.2f, P95 %.2f, max %.2f (frame %d), >30cm %d, >50cm %d, yaw RMSE %.3f deg, frame 851 %.1f cm, time/frame mean %.1f ms max %.1f ms\n', ...
            mode,nnz(a),100*rms(e(a)),100*median(e(a)),100*prctile(e(a),95),100*max(e(a)),find(e==max(e(a)),1),nnz(e(a)>.3),nnz(e(a)>.5),rms(yaw(a)),100*e(851),1e3*mean(seconds),1e3*max(seconds));
    end
    save(fullfile(out,'closed_loop_'+strjoin(string(modes),'_')+'.mat'),'results','-v7.3');
    names=fieldnames(results);frames=table((1:n).',VariableNames={'frame'});
    for j=1:numel(names),frames.(names{j}+"ErrorM")=results.(names{j}).errorM;frames.(names{j}+"Accepted")=results.(names{j}).accepted;end
    writetable(frames,fullfile(dest,'closed_loop_frames_'+strjoin(string(modes),'_')+'.csv'));summary=results;
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
