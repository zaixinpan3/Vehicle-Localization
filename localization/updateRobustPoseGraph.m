function [result,state]=updateRobustPoseGraph(state,packet,map,cfg)
% updateRobustPoseGraph Joint causal pose estimation with revisable map factors.
% packet: time, source (confirmed current acquisition), seed, relativeMotion,
% motionCovariance and positionAid. Relative motion is independent odometry.
% Each acquisition enters once. Marginalization includes only the departing
% node's unary factors, incoming prior and outgoing odometry edge. Retained
% factors are relinearized, never also folded into the marginal prior.
% Output is a fused estimate, NOT an independent LiDAR measurement. Its
% conditional local information is uncalibrated and includes position aiding.
    validateattributes(packet.time,{'numeric'},{'scalar','finite','real'});
    validateattributes(packet.seed,{'numeric'},{'numel',3,'finite','real'});
    assert(cfg.maximumFrames>=2 && cfg.maximumFrames==fix(cfg.maximumFrames) && ...
        cfg.maximumIterations>=1 && cfg.switchPrior>0 && cfg.gnssRobustScale>0 && ...
        cfg.lidarInformationScale>0 && ismember(cfg.featureLoss,["switchable","cauchy","quadratic"]), ...
        'VehicleLocalization:InvalidGraphConfig','Invalid graph bounds or loss.');
    assert(isfield(packet.source,'observationScope') && ...
        packet.source.observationScope=="confirmedCurrentAcquisition" && ...
        abs(packet.source.acquisitionTime-packet.time)<1e-7, ...
        'VehicleLocalization:GraphNeedsCurrentScan','Do not insert overlapping stacked observations.');
    if isempty(state)
        state=struct('poses',zeros(0,3),'nodes',{{}},'time',zeros(0,1), ...
            'origin',packet.seed(1:2),'totalAcquisitions',0,'totalObservations',0, ...
            'marginalizedAcquisitions',0,'containsGnss',false);
        state.prior=struct('center',[0 0 packet.seed(3)], ...
            'information',diag(1./cfg.initialStandardDeviation.^2),'gradient',zeros(3,1));
    end
    assert(isempty(state.time)||packet.time>state.time(end), ...
        'VehicleLocalization:DuplicateGraphAcquisition','Graph timestamps must strictly increase.');
    if ~isempty(state.time)
        validateattributes(packet.relativeMotion,{'numeric'},{'numel',3,'finite','real'});
        requireCovariance(packet.motionCovariance,3);
    end
    aid=packet.positionAid;
    if ~isempty(aid) && aid.valid
        requireCovariance(aid.covariance,2);
        validateattributes(aid.position,{'numeric'},{'numel',2,'finite','real'});
        assert(abs(aid.timestamp-packet.time)<1e-7,'VehicleLocalization:PositionAidTimeMismatch','Position aid needs the same acquisition time.');
        aid.position=aid.position(:).'-state.origin;
        state.containsGnss=true;
    end
    [local,indices]=selectLocalProbabilityCloud(map,packet.seed,cfg.registration.localMapRadius);
    model=prepareSemanticRegistrationGeometry(local,packet.source,[state.origin 0],cfg.registration);
    node=struct('model',model,'globalIndices',indices,'aid',aid, ...
        'motion',packet.relativeMotion,'motionCovariance',packet.motionCovariance, ...
        'observationCount',packet.source.components.numComponents);
    state.nodes{end+1,1}=node;state.time(end+1,1)=packet.time;
    state.poses(end+1,:)=packet.seed-[state.origin 0];
    state.totalAcquisitions=state.totalAcquisitions+1;
    state.totalObservations=state.totalObservations+node.observationCount;
    iterations=0;converged=true;
    % Keep the declared common initial pose as the first online output.
    if numel(state.time)>1
        [state.poses,iterations,converged]=optimize(state,cfg);
    end
    [H,~,~,systems]=assemble(state,state.poses,cfg);
    last=3*numel(state.time)-2:3*numel(state.time);
    E=zeros(size(H,1),3);E(last,:)=eye(3);
    covariance=E.'*(H\E);information=covariance\eye(3);
    system=systems{end};q=system.pairs.squaredStandardizedResidual;
    [~,influence]=featureLoss(q,cfg);
    pairs=system.pairs;pairs.graphInfluence=influence;
    pairs.globalTarget=node.globalIndices(pairs.target);
    result=struct('poseXYTheta',state.poses(end,:)+[state.origin 0], ...
        'information',(information+information.')/2,'informationCalibrated',false, ...
        'containsGnss',state.containsGnss,'measurementType',"fusedGraphPose", ...
        'independentLidarMeasurement',false,'iterations',iterations,'converged',converged, ...
        'activeAcquisitions',numel(state.time),'totalAcquisitions',state.totalAcquisitions, ...
        'uniqueObservationCount',state.totalObservations,'correspondences',pairs);
    result.poseXYTheta(3)=wrap(result.poseXYTheta(3));
    if numel(state.time)>cfg.maximumFrames
        [h,g]=unary(state.nodes{1},state.poses(1,:),cfg);
        delta=state.poses(1,:)-state.prior.center;delta(3)=wrap(delta(3));
        h=h+state.prior.information;g=g+state.prior.information*delta.'+state.prior.gradient;
        [he,ge]=edge(state.poses(1,:),state.poses(2,:),state.nodes{2});
        he(1:3,1:3)=he(1:3,1:3)+h;ge(1:3)=ge(1:3)+g;
        A=he(1:3,1:3);B=he(1:3,4:6);
        reduced=he(4:6,4:6)-B.'*(A\B);
        state.prior=struct('center',state.poses(2,:), ...
            'information',(reduced+reduced.')/2,'gradient',ge(4:6)-B.'*(A\ge(1:3)));
        state.poses(1,:)=[];state.nodes(1)=[];state.time(1)=[];
        state.marginalizedAcquisitions=state.marginalizedAcquisitions+1;
    end
end

function [poses,iterations,converged]=optimize(state,cfg)
    poses=state.poses;converged=false;
    for iterations=1:cfg.maximumIterations
        [H,g,cost,systems]=assemble(state,poses,cfg);
        step=-(H+1e-8*diag(max(diag(H),1)))\g;
        delta=reshape(step,3,[]).';
        scale=max([1;vecnorm(delta(:,1:2),2,2)/.5;abs(delta(:,3))/.1]);delta=delta/scale;
        if max(abs(delta),[],'all')<cfg.stepTolerance,converged=true;break;end
        accepted=false;
        for j=0:8
            trial=poses+delta*2^(-j);
            value=frozenCost(state,trial,systems,cfg);
            if value<cost-1e-10,poses=trial;accepted=true;break;end
        end
        if ~accepted,break;end
    end
end

function [H,g,cost,systems]=assemble(state,poses,cfg)
    n=size(poses,1);H=zeros(3*n);g=zeros(3*n,1);cost=0;systems=cell(n,1);
    for k=1:n
        ids=3*k-2:3*k;
        [h,b,c,systems{k}]=unary(state.nodes{k},poses(k,:),cfg);
        H(ids,ids)=H(ids,ids)+h;g(ids)=g(ids)+b;cost=cost+c;
        if k>1
            ids=3*k-5:3*k;[h,b,c]=edge(poses(k-1,:),poses(k,:),state.nodes{k});
            H(ids,ids)=H(ids,ids)+h;g(ids)=g(ids)+b;cost=cost+c;
        end
    end
    delta=poses(1,:)-state.prior.center;delta(3)=wrap(delta(3));
    H(1:3,1:3)=H(1:3,1:3)+state.prior.information;
    g(1:3)=g(1:3)+state.prior.information*delta.'+state.prior.gradient;
    cost=cost+delta*state.prior.information*delta.'+2*delta*state.prior.gradient;
end

function [H,g,cost,system]=unary(node,pose,cfg)
    system=node.model.linearize(pose,ones(3,1));
    [rho,w]=featureLoss(system.pairs.squaredStandardizedResidual,cfg);
    weights=cfg.lidarInformationScale*system.weights;
    A=reshape(permute(system.J,[1 3 2]),[],3);W=repelem(weights.*w,2,1);
    H=A.'*(A.*W);g=A.'*(system.residual(:).*W);cost=sum(weights.*rho);
    [h,b,c]=gnss(node.aid,pose,cfg);
    H=H+h;g=g+b;cost=cost+c;
end

function value=frozenCost(state,poses,systems,cfg)
    value=0;
    for k=1:size(poses,1)
        system=systems{k};q=state.nodes{k}.model.squaredResidual(system,poses(k,:));
        rho=featureLoss(q,cfg);[~,~,c]=gnss(state.nodes{k}.aid,poses(k,:),cfg);
        value=value+cfg.lidarInformationScale*sum(system.weights.*rho)+c;
        if k>1,[~,~,c]=edge(poses(k-1,:),poses(k,:),state.nodes{k});value=value+c;end
    end
    delta=poses(1,:)-state.prior.center;delta(3)=wrap(delta(3));
    value=value+delta*state.prior.information*delta.'+2*delta*state.prior.gradient;
end

function [rho,w]=featureLoss(q,cfg)
    a=cfg.switchPrior;
    switch cfg.featureLoss
        case "switchable",s=a./(a+q);rho=a*q./(a+q);w=s.^2;
        case "cauchy",rho=a*log1p(q/a);w=1./(1+q/a);
        otherwise,rho=q;w=ones(size(q));
    end
end

function [H,g,cost]=gnss(aid,pose,cfg)
    H=zeros(3);g=zeros(3,1);cost=0;
    if isempty(aid)||~aid.valid,return;end
    residual=pose(1:2)-aid.position;
    I=aid.covariance\eye(2);q=residual*I*residual.';w=1/(1+q/cfg.gnssRobustScale);
    H(1:2,1:2)=w*I;g(1:2)=w*I*residual.';
    cost=cfg.gnssRobustScale*log1p(q/cfg.gnssRobustScale);
end

function [H,g,cost]=edge(a,b,node)
    R=[cos(a(3)) sin(a(3));-sin(a(3)) cos(a(3))];
    v=R*(b(1:2)-a(1:2)).';residual=[v-node.motion(1:2).';wrap(b(3)-a(3)-node.motion(3))];
    A=[-R,[v(2);-v(1)];0 0 -1];B=[R,zeros(2,1);0 0 1];J=[A B];
    I=node.motionCovariance\eye(3);H=J.'*I*J;g=J.'*I*residual;cost=residual.'*I*residual;
end

function requireCovariance(C,n)
    assert(isequal(size(C),[n n]) && isreal(C) && all(isfinite(C),'all') && ...
        norm(C-C.','fro')<1e-9 && min(eig(C))>0, ...
        'VehicleLocalization:InvalidGraphCovariance','Graph covariance must be symmetric positive definite.');
end

function x=wrap(x)
    x=atan2(sin(x),cos(x));
end
