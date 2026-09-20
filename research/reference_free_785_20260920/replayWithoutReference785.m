function result=replayWithoutReference785(map,clouds,data,lateral,cfg,registration,bootstrap,mode)
% replayWithoutReference785 Replay the historical matcher without pose truth.
% Only sensor inputs, local feature clouds, map and estimated states enter.
% Observer feedback uses prefix replay of the unmodified historical observer
% to preserve its exact history and equations without adding a second runtime.
    assert(ismember(mode,["matching_prediction","fusion_prediction"]));
    t=data.highRate.time(:);n=numel(t);assert(n==numel(clouds));
    data.lidar.pose=nan(n,3);data.lidar.information=zeros(3,3,n);data.lidar.valid=false(n,1);
    pose=bootstrap.result.poseXYTheta;r=[cos(pose(3)),-sin(pose(3));sin(pose(3)),cos(pose(3))];
    v=r*[data.highRate.longitudinalSpeed(1);lateral.lateralVelocity(1)];
    a=r*[data.highRate.longitudinalAcceleration(1);data.highRate.lateralAcceleration(1)];
    cfg.initialState=[pose(1);v(1);a(1);pose(2);v(2);a(2);pose(3)];
    rows=cell(n,25);state=pose;prefixPose=zeros(n,3);prefixPose(1,:)=pose;
    for k=1:n
        initial=bootstrap.initialPose;
        if k>1,initial=advance(state,data.highRate,lateral,k);end
        r=bootstrap.result;
        if k>1,r=registerSemanticProbabilityCloud(map,clouds{k},initial,registration);end
        output=initial;
        % The historical fusion accepted full-pose events only. Retain that
        % policy for prediction and fusion, and log directional candidates.
        if r.accepted
            output=r.poseXYTheta;data.lidar.pose(k,:)=output;
            data.lidar.information(:,:,k)=r.information;data.lidar.valid(k)=true;
        end
        I=r.information;
        rows(k,:)={k,t(k),initial(1),initial(2),initial(3),output(1),output(2),output(3), ...
            r.poseXYTheta(1),r.poseXYTheta(2),r.poseXYTheta(3),r.accepted,r.directionalAccepted, ...
            string(r.reason),r.similarity,r.observableRank,r.iterations,clouds{k}.components.numComponents, ...
            I(1,1),I(1,2),I(1,3),I(2,2),I(2,3),I(3,3),r.converged};
        state=output;
        if mode=="fusion_prediction"
            if k>1
                [prefix,lat]=prefixInputs(data,lateral,k);
                estimate=runSynchronousLocalizationObserver(prefix,cfg,lat);
                prefixPose(k,:)=estimate.pose(end,:);
            end
            state=prefixPose(k,:);
        end
        if mod(k,100)==0,fprintf('%s %d/%d; accepted %d\n',mode,k,n,nnz(data.lidar.valid));end
    end
    estimate=runSynchronousLocalizationObserver(data,cfg,lateral);
    if mode=="fusion_prediction"
        assert(max(abs(estimate.pose-prefixPose),[],'all')<1e-9,'Prefix observer differs from complete replay.');
    end
    gnssOnly=rmfield(data,'lidar');gnssEstimate=runSynchronousLocalizationObserver(gnssOnly,cfg,lateral);
    calls=cell2table(rows,VariableNames={'frame','time','predictedX','predictedY','predictedPsi', ...
        'x','y','psi','candidateX','candidateY','candidatePsi','accepted','directionalCandidate','reason', ...
        'similarity','rank','iterations','sourceComponents','informationXX','informationXY', ...
        'informationXPsi','informationYY','informationYPsi','informationPsiPsi','converged'});
    result=struct('mode',mode,'calls',calls,'estimate',estimate,'gnssEstimate',gnssEstimate, ...
        'data',data,'cfg',cfg,'prefixPose',prefixPose,'referencePoseArgument',false);
end

function pose=advance(state,h,lateral,k)
    dt=h.time(k)-h.time(k-1);yawStep=dt*(h.yawRate(k-1)+h.yawRate(k))/2;
    velocity=[mean(h.longitudinalSpeed(k-1:k));mean(lateral.lateralVelocity(k-1:k))];
    half=yawStep/2;factor=1;if abs(half)>1e-10,factor=sin(half)/half;end
    yaw=state(3)+half;rotation=[cos(yaw),-sin(yaw);sin(yaw),cos(yaw)];
    pose=[state(1:2)+(dt*factor*rotation*velocity).',atan2(sin(state(3)+yawStep),cos(state(3)+yawStep))];
end

function [prefix,lat]=prefixInputs(data,lateral,k)
    prefix=data;
    for name=string(fieldnames(data.highRate)).'
        prefix.highRate.(name)=data.highRate.(name)(1:k,:);
    end
    for source=["gnss","lidar"]
        s=data.(source);
        for name=string(fieldnames(s)).'
            if name=="information"
                s.(name)=s.(name)(:,:,1:k);
            elseif size(s.(name),1)==numel(data.highRate.time)
                s.(name)=s.(name)(1:k,:);
            end
        end
        prefix.(source)=s;
    end
    lat=lateral;
    for name=string(fieldnames(lat)).',lat.(name)=lat.(name)(1:k,:);end
end
