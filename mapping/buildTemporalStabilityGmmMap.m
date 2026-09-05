function map = buildTemporalStabilityGmmMap(points, labels, observations, cfg)
% buildTemporalStabilityGmmMap: Fit a semantic repeated-observation field.
% observations is an ID vector or a struct with frameId, observationBlockId,
% and optional sourceId vectors. Numeric frames can be grouped using
% cfg.observationBlockSize and cfg.blockOriginFrame. Explicit blocks win.
% q(A) prod_k q(H_k,mu_k,U_k) retains hypothesis/offset/mean dependence.
% Tiles fit unique deterministic XY representatives including their halos;
% only owned support cells contribute reference mass. See the design document
% research/repeated_observation_map_design.md for the objective and limits.
    if nargin < 4, cfg = temporalStabilityMapConfig(); end
    assert(isfield(cfg,'schemaVersion') && cfg.schemaVersion==2, ...
        'buildTemporalStabilityGmmMap:UnsupportedConfiguration','Rebuild configuration with temporalStabilityMapConfig.');
    validateattributes(points,{'numeric'},{'real','2d'},mfilename,'points');
    assert(ismember(size(points,2),[2 3]) && all(isfinite(points(:,1:2)),'all'), ...
        'buildTemporalStabilityGmmMap:InvalidPoints','Finite XY coordinates are required.');
    assert(size(points,2)==2 || all(isfinite(points(:,3)) | isnan(points(:,3))), ...
        'buildTemporalStabilityGmmMap:InvalidHeight','Height must be finite or missing (NaN).');
    points=double(points); labels=string(labels(:));
    assert(numel(labels)==size(points,1) && all(~ismissing(labels) & strlength(labels)>0), ...
        'buildTemporalStabilityGmmMap:InvalidLabels','One semantic label is required per point.');
    [frameIds,blockIds,sourceIds]=canonicalIds(observations,size(points,1),cfg);
    classes=string(cfg.classes(:));
    if isempty(classes), classes=unique(labels); end
    assert(numel(unique(classes))==numel(classes),'Duplicate requested classes.');
    layers=cell(numel(classes),1);
    for c=1:numel(classes)
        p=classParameters(cfg,classes(c)); rows=find(labels==classes(c));
        layers{c}=buildLayer(points(rows,:),frameIds(rows),blockIds(rows), ...
            sourceIds(rows),rows,classes(c),p,cfg);
    end
    map=struct('mapType',"semanticRepeatedObservationGaussianField", ...
        'schemaVersion',2,'classLabels',classes,'sourcePointCount',size(points,1), ...
        'layers',[],'spatialDimension',size(points,2), ...
        'heightModel',"conditionalGaussianGivenXY",'coordinateFrame',"mapXY", ...
        'fieldSemantics',"referenceMassTimesRepeatabilityGaussianIntensity", ...
        'coverageSemantics',"ownedFittingRegionsNotVisibility", ...
        'observationBlockPolicy',"explicitIdsOrFixedFrameGroups",'config',cfg);
    if ~isempty(layers), map.layers=vertcat(layers{:}); end
    mappingSupport.validateRepeatedObservationMap(map);
end

function [frames,blocks,sources]=canonicalIds(obs,n,cfg)
    if isstruct(obs)
        assert(isscalar(obs) && isfield(obs,'frameId') && isfield(obs,'observationBlockId'), ...
            'Observation structs require frameId and observationBlockId.');
        frames=validIds(obs.frameId,n); blocks=validIds(obs.observationBlockId,n);
        if isfield(obs,'sourceId'), sources=validIds(obs.sourceId,n); else, sources=string((1:n).'); end
    else
        frames=validIds(obs,n); sources=string((1:n).');
        validateattributes(cfg.observationBlockSize,{'numeric'},{'scalar','integer','positive','finite'});
        if cfg.observationBlockSize==1
            blocks=frames;
        else
            validateattributes(obs,{'numeric'},{'real','finite','integer'});
            validateattributes(cfg.blockOriginFrame,{'numeric'},{'scalar','integer','finite'});
            blocks=string(floor((double(obs(:))-cfg.blockOriginFrame)/cfg.observationBlockSize));
        end
    end
    [~,~,f]=unique(frames);
    for k=1:max([f;0])
        assert(isscalar(unique(blocks(f==k))), ...
            'buildTemporalStabilityGmmMap:ConflictingBlocks','A source frame must belong to one block.');
    end
end

function ids=validIds(ids,n)
    if isnumeric(ids), assert(isreal(ids) && all(isfinite(ids(:))),'IDs must be finite.'); end
    ids=string(ids(:));
    assert(numel(ids)==n && all(~ismissing(ids) & strlength(ids)>0),'One valid ID is required per point.');
end

function p=classParameters(cfg,name)
    p=cfg.defaultParams;
    matches=find(string({cfg.classParams.classLabel})==name);
    assert(numel(matches)<=1,'Duplicate class configuration.');
    if ~isempty(matches), p=cfg.classParams(matches); end
    template=temporalStabilityMapConfig();
    assert(isempty(setdiff(fieldnames(p),fieldnames(template.defaultParams))), ...
        'buildTemporalStabilityGmmMap:RemovedParameter', ...
        'Unknown parameters include removed reliability, priorSupport, pruning or nearest-component settings.');
    positive=["representativeResolution","referenceResolution","tileSize", ...
        "stableStandardDeviation","variableStandardDeviation","tangentStandardDeviation", ...
        "meanPriorStandardDeviation","minCovarianceEigenvalue","maxCovarianceEigenvalue", ...
        "mixturePseudocount","clutterIntensity","publishFalsePositiveCost", ...
        "publishFalseNegativeCost","emTolerance","parameterTolerance", ...
        "minBlockEffectiveCount","heightModeSeparation","heightModeVarianceRatio", ...
        "elongatedAnisotropyThreshold","queryIndexCellSize"];
    for key=positive, validateattributes(p.(key),{'numeric'},{'real','scalar','positive','finite'},mfilename,key); end
    integers=["maxComponentsPerTile","minComponentPoints","minObservedBlocks", ...
        "emMaxIterations","selectionMinBlocks","selectionHoldoutEvery"];
    for key=integers, validateattributes(p.(key),{'numeric'},{'scalar','integer','positive','finite'},mfilename,key); end
    assert(p.minObservedBlocks>=2 && p.selectionMinBlocks>=3 && p.selectionHoldoutEvery>=2, ...
        'Repeatability needs two blocks; selection needs training and held-out blocks.');
    validateattributes(p.tileOrigin,{'numeric'},{'real','size',[1 2],'finite'});
    validateattributes(p.backgroundBetaPrior,{'numeric'},{'real','size',[1 2],'finite','>',1});
    validateattributes(p.repeatabilityPrior,{'numeric'},{'scalar','>',0,'<',1});
    for key=["contextHalo","queryIntensityTolerance","selectionMinGainPerPoint","classImportance"]
        validateattributes(p.(key),{'numeric'},{'scalar','nonnegative','finite'});
    end
    assert(p.maxCovarianceEigenvalue>=p.minCovarianceEigenvalue,'Invalid covariance bounds.');
    assert(abs(p.tileSize/p.referenceResolution-round(p.tileSize/p.referenceResolution))<1e-10, ...
        'Tile size must be an integer multiple of reference resolution.');
    validateattributes(cfg.minimumConditionalHeightVariance,{'numeric'},{'scalar','positive','finite'});
end

function layer=buildLayer(raw,frames,blocks,sources,sourceRows,name,p,cfg)
    [xy,block,provenance]=representatives(raw,frames,blocks,sources,sourceRows,p);
    n=size(xy,1);
    layer=struct('classLabel',name,'schemaVersion',2,'params',p, ...
        'pointCount',size(raw,1),'representativePoints',xy, ...
        'representativeBlockIds',provenance.blockIds,'provenance',provenance, ...
        'components',repmat(emptyComponent(),0,1),'tiles',struct([]), ...
        'coverageRegions',zeros(0,4),'clutterIntensity',p.clutterIntensity, ...
        'classImportance',p.classImportance,'totalMass',0,'referenceMass',0,'backgroundReferenceMass',0, ...
        'gmmTrainingModel',"structuredVariationalRepeatedObservationMixture", ...
        'repeatabilitySemantics',"variationalProbabilityConditionalOnModel", ...
        'gmmTrainingUsesTimestamps',true,'gmmTrainingUsesUniformBackground',true, ...
        'covarianceSemantics',"withinBlockPlusStableOffsetPlusReferenceMeanUncertainty");
    if n>0
        [tiles,~,owner]=unique(floor((xy-p.tileOrigin)/p.tileSize),'rows');
        tileResults=cell(size(tiles,1),1);
        for t=1:size(tiles,1)
            low=p.tileOrigin+tiles(t,:)*p.tileSize; high=low+p.tileSize;
            bounds=[low-p.contextHalo,high+p.contextHalo];
            use=all(xy>=bounds(1:2) & xy<=bounds(3:4),2);
            localRows=find(use); core=owner(use)==t; center=(low+high)/2;
            [localBlocks,~,b]=unique(block(use));
            [fit,selection]=selectAndFit(xy(use,:)-center,b,bounds-[center center],p);
            [components,massCells,backgroundMass]=publishTile(fit,xy(use,:),core,center,tiles(t,:), ...
                p,raw,provenance.localRows(localRows),cfg);
            layer.components=[layer.components;components];
            tileResults{t}=struct('ownerTile',tiles(t,:),'coverage',[low high], ...
                'observationBlockIds',provenance.uniqueBlockIds(localBlocks), ...
                'representativeIndices',localRows,'selection',selection, ...
                'fit',fitDiagnostics(fit,p),'referenceCells',massCells,'backgroundReferenceCellMass',backgroundMass, ...
                'referenceMass',size(massCells,1)*p.referenceResolution^2);
            mappingSupport.printLog(mappingSupport.isLogEnabled(cfg,"logEnabled"), ...
                "repeatability-map","tile","class=%s tile=[%d %d] representatives=%d K=%d ELBO=%.6g", ...
                name,tiles(t,1),tiles(t,2),nnz(use),size(fit.C,3),fit.trace(end));
        end
        layer.tiles=vertcat(tileResults{:});
        layer.coverageRegions=vertcat(layer.tiles.coverage);
        layer.referenceMass=sum([layer.tiles.referenceMass]);
        layer.backgroundReferenceMass=sum(arrayfun(@(tile) sum(tile.backgroundReferenceCellMass),layer.tiles));
    end
    k=numel(layer.components);
    layer.componentMeans=reshape([layer.components.mean],2,k).';
    layer.componentCovariances=reshape([layer.components.covariance],2,2,k);
    layer.componentMeansXYZ=reshape([layer.components.meanXYZ],3,k).';
    layer.componentCovariancesXYZ=reshape([layer.components.covarianceXYZ],3,3,k);
    layer.componentMasses=reshape([layer.components.mass],k,1);
    layer.componentReferenceMasses=reshape([layer.components.referenceMass],k,1);
    layer.componentRepeatability=reshape([layer.components.repeatability],k,1);
    layer.componentPublished=logical(reshape([layer.components.published],k,1));
    layer.componentHeightAvailable=logical(reshape([layer.components.heightAvailable],k,1));
    layer.componentIds=reshape(string({layer.components.id}),k,1);
    layer.totalMass=sum(layer.componentMasses);
    layer.componentMixtureWeights=zeros(k,1);
    if layer.totalMass>0, layer.componentMixtureWeights=layer.componentMasses/layer.totalMass; end
    layer.queryIndex=mappingSupport.compileFieldIndex(layer);
end

function [xy,block,prov]=representatives(raw,frames,blocks,sources,sourceRows,p)
% One location closest to each block's XY voxel center; lexical tie breaking.
% Height and multiplicity cannot change XY selection. Retain all provenance.
    [uniqueBlocks,~,allBlocks]=unique(blocks);
    if isempty(raw)
        xy=zeros(0,2); block=zeros(0,1); groups=cell(0,1);
    else
        [~,~,s]=unique(sources);
        sourceCounts=accumarray(s,1);
        for j=find(sourceCounts>1).'
            rows=find(s==j);
            assert(all(frames(rows)==frames(rows(1))) && all(blocks(rows)==blocks(rows(1))) && ...
                isequaln(raw(rows,:),repmat(raw(rows(1),:),numel(rows),1)), ...
                'buildTemporalStabilityGmmMap:ConflictingSource','A source ID identifies conflicting observations.');
        end
        voxel=floor((raw(:,1:2)-p.tileOrigin)/p.representativeResolution);
        [~,~,group]=unique([allBlocks voxel],'rows');
        centers=p.tileOrigin+(voxel+0.5)*p.representativeResolution;
        [~,order]=sortrows([group sum((raw(:,1:2)-centers).^2,2) raw(:,1:2)],[1 2 3 4]);
        chosen=order([true;diff(group(order))~=0]);
        xy=raw(chosen,1:2); block=allBlocks(chosen);
        groups=accumarray(group,(1:size(raw,1)).',[],@(v){v});
    end
    prov=struct('uniqueBlockIds',uniqueBlocks,'blockIds',uniqueBlocks(block), ...
        'sourceFrameIds',{cellfun(@(v) unique(frames(v)),groups,'UniformOutput',false)}, ...
        'sourceObservationIds',{cellfun(@(v) unique(sources(v)),groups,'UniformOutput',false)}, ...
        'sourceRows',{cellfun(@(v) sourceRows(v),groups,'UniformOutput',false)}, ...
        'localRows',{groups},'representativeResolution',p.representativeResolution);
end

function [fit,selection]=selectAndFit(x,b,bounds,p)
    B=max(b); budget=min(p.maxComponentsPerTile,max(1,floor(size(x,1)/p.minComponentPoints)));
    selected=1; scores=nan(budget,1); held=zeros(0,1); status="insufficientBlocks";
    candidateConverged=false(budget,1); candidateIterations=zeros(budget,1);
    if B>=p.selectionMinBlocks && budget>1
        held=(p.selectionHoldoutEvery:p.selectionHoldoutEvery:B).';
        test=ismember(b,held); train=~test; [~,~,trainBlocks]=unique(b(train));
        if ~isempty(held) && max(trainBlocks)>=2
            status="heldOutBlockPredictiveCompositeLogScore";
            for k=1:budget
                candidate=fitMixture(x(train,:),trainBlocks,bounds,k,p);
                candidateConverged(k)=candidate.converged;
                candidateIterations(k)=numel(candidate.trace)-1;
                scores(k)=predictiveScore(candidate,x(test,:),b(test));
                if k>1 && scores(k)>scores(selected)+p.selectionMinGainPerPoint, selected=k; end
            end
        end
    end
    fit=fitMixture(x,b,bounds,selected,p);
    selection=struct('status',status,'selectedComponentCount',selected, ...
        'candidateCounts',(1:budget).','heldOutBlocks',held,'scorePerPoint',scores, ...
        'candidateConverged',candidateConverged,'candidateIterations',candidateIterations, ...
        'finalRefitUsesSelectionBlocks',true,'evaluationDataUsed',false);
end

function fit=fitMixture(x,b,bounds,K,p)
    B=max(b); area=prod(bounds(3:4)-bounds(1:2));
    [assignment,C,QS,QV]=initializeGeometry(x,b,K,p);
    gamma=zeros(size(x,1),K+1);
    gamma(sub2ind(size(gamma),(1:size(x,1)).',assignment+1))=0.9; gamma(:,1)=0.1;
    [pi,epsilon]=observationWeights(gamma,b,B,p);
    posterior=inferGeometry(x,b,gamma(:,2:end),C,QS,QV,p);
    objectiveTrace=lowerBound(posterior,gamma,b,pi,epsilon,area,p);
    shifts=zeros(0,1); converged=false;
    for iteration=1:p.emMaxIterations
        oldParameters=parameterVector(C,pi,epsilon,posterior,gamma);
        logAssignment=expectedLogAssignments(x,b,C,posterior,pi,epsilon,area);
        gamma=exp(logAssignment-logSumExp(logAssignment,2));
        posterior=inferGeometry(x,b,gamma(:,2:end),C,QS,QV,p);
        C=updateCovariance(x,b,gamma(:,2:end),posterior,C,p);
        [pi,epsilon]=observationWeights(gamma,b,B,p);
        posterior=inferGeometry(x,b,gamma(:,2:end),C,QS,QV,p);
        value=lowerBound(posterior,gamma,b,pi,epsilon,area,p);
        assert(isfinite(value) && value>=objectiveTrace(end)-1e-7*max(1,abs(objectiveTrace(end))), ...
            'buildTemporalStabilityGmmMap:ObjectiveDecrease','Structured variational lower bound decreased.');
        change=max(abs(parameterVector(C,pi,epsilon,posterior,gamma)-oldParameters)./(1+abs(oldParameters)));
        shifts(end+1,1)=change; %#ok<AGROW>
        progress=abs(value-objectiveTrace(end))/max(1,abs(objectiveTrace(end)));
        objectiveTrace(end+1,1)=value; %#ok<AGROW>
        if progress<=p.emTolerance && change<=p.parameterTolerance, converged=true; break; end
    end
    fit=struct('C',C,'QS',QS,'QV',QV,'posterior',{posterior},'gamma',gamma, ...
        'pi',pi,'epsilon',epsilon,'trace',objectiveTrace,'parameterChange',shifts, ...
        'converged',converged,'area',area,'bounds',bounds);
end

function [assignment,C,QS,QV]=initializeGeometry(x,b,K,p)
% Deterministic farthest-point proposals and Lloyd updates. Normals come from
% observed within-block scatter and remain fixed during each fit's objective.
    centers=zeros(K,2); centers(1,:)=mean(x,1); distances=inf(size(x,1),1);
    for k=2:K
        distances=min(distances,sum((x-centers(k-1,:)).^2,2));
        [~,i]=max(distances); centers(k,:)=x(i,:);
    end
    assignment=ones(size(x,1),1);
    for iteration=1:20
        distances=zeros(size(x,1),K);
        for k=1:K, distances(:,k)=sum((x-centers(k,:)).^2,2); end
        [~,next]=min(distances,[],2);
        for k=1:K
            if any(next==k), centers(k,:)=mean(x(next==k,:),1); end
        end
        if isequal(next,assignment) && iteration>1, break; end
        assignment=next;
    end
    C=zeros(2,2,K); QS=C; QV=C;
    for k=1:K
        scatter=zeros(2); count=0;
        for j=unique(b(assignment==k)).'
            z=x(assignment==k & b==j,:); residual=z-mean(z,1);
            scatter=scatter+residual.'*residual; count=count+size(z,1);
        end
        C(:,:,k)=boundCovariance(scatter/max(count,1),p);
        [vectors,values]=eig(C(:,:,k),'vector'); [values,order]=sort(values);
        normal=vectors(:,order(1)); tangent=vectors(:,order(2));
        projector=eye(2); QS(:,:,k)=p.stableStandardDeviation^2*eye(2);
        if values(2)/values(1)>=p.elongatedAnisotropyThreshold
            projector=normal*normal.';
            QS(:,:,k)=p.stableStandardDeviation^2*projector + p.tangentStandardDeviation^2*(tangent*tangent.');
        end
        QV(:,:,k)=QS(:,:,k)+p.variableStandardDeviation^2*projector;
    end
end

function posterior=inferGeometry(x,b,weights,C,QS,QV,p)
% Integrate offsets and the proper mean prior for fractional assignments.
% Zero-mass blocks contribute exactly zero evidence under both hypotheses.
    K=size(C,3); B=max(b); posterior=cell(K,1);
    for k=1:K
        count=accumarray(b,weights(:,k),[B 1]); means=zeros(B,2); scatter=zeros(2,2,B);
        for j=find(count>0).'
            use=b==j; w=weights(use,k); z=x(use,:);
            means(j,:)=sum(z.*w,1)/count(j);
            residual=z-means(j,:); scatter(:,:,j)=residual.'*(residual.*w);
        end
        P0=p.meanPriorStandardDeviation^2*eye(2);
        stable=gaussianHierarchy(count,means,scatter,C(:,:,k),QS(:,:,k),P0);
        variable=gaussianHierarchy(count,means,scatter,C(:,:,k),QV(:,:,k),P0);
        logEvidence=[log(p.repeatabilityPrior)+stable.logEvidence,log1p(-p.repeatabilityPrior)+variable.logEvidence];
        normalizer=logSumExp(logEvidence,2);
        posterior{k}=struct('stable',stable,'variable',variable, ...
            'r',exp(logEvidence(1)-normalizer),'logNormalizer',normalizer,'count',count);
    end
end

function q=gaussianHierarchy(count,means,scatter,C,Q,P0)
    B=numel(count); invC=C\eye(2); invQ=Q\eye(2);
    precision=P0\eye(2); linear=zeros(2,1); constant=0; quadratic=0;
    for j=find(count>0).'
        n=count(j); y=means(j,:).'; invV=n*((C+n*Q)\eye(2));
        precision=precision+invV; linear=linear+invV*y; quadratic=quadratic+y.'*invV*y;
        constant=constant-0.5*(n*(2*log(2*pi)+logDet(C)) + ...
            trace(invC*scatter(:,:,j))+logDet(C+n*Q)-logDet(C));
    end
    covariance=precision\eye(2); mean=covariance*linear;
    logEvidence=constant-0.5*(logDet(P0)+logDet(precision)+quadratic-linear.'*covariance*linear);
    locationMean=zeros(B,2); locationCovariance=zeros(2,2,B);
    for j=1:B
        W=(invQ+count(j)*invC)\eye(2); gain=W*(count(j)*invC);
        locationMean(j,:)=(mean+gain*(means(j,:).'-mean)).';
        locationCovariance(:,:,j)=(eye(2)-gain)*covariance*(eye(2)-gain).'+W;
    end
    q=struct('mean',mean,'covariance',covariance,'locationMean',locationMean, ...
        'locationCovariance',locationCovariance,'logEvidence',logEvidence);
end

function logs=expectedLogAssignments(x,b,C,post,mixture,epsilon,area)
    logs=zeros(size(x,1),size(C,3)+1); logs(:,1)=log(epsilon(b))-log(area);
    for k=1:size(C,3)
        invC=C(:,:,k)\eye(2); q=post{k}; expected=zeros(size(x,1),1);
        hypotheses={q.stable,q.variable}; probabilities=[q.r 1-q.r];
        for h=1:2
            state=hypotheses{h}; delta=x-state.locationMean(b,:); variance=zeros(size(state.locationMean,1),1);
            for j=1:numel(variance), variance(j)=trace(invC*state.locationCovariance(:,:,j)); end
            expected=expected+probabilities(h)*(sum((delta*invC).*delta,2)+variance(b));
        end
        logs(:,k+1)=log1p(-epsilon(b))+log(mixture(b,k))-0.5*(2*log(2*pi)+logDet(C(:,:,k))+expected);
    end
end

function C=updateCovariance(x,b,weights,post,C,p)
    for k=1:size(C,3)
        mass=sum(weights(:,k));
        if mass==0, continue; end
        q=post{k}; hypotheses={q.stable,q.variable}; probabilities=[q.r 1-q.r]; scatter=zeros(2);
        for h=1:2
            state=hypotheses{h}; delta=x-state.locationMean(b,:); current=delta.'*(delta.*weights(:,k));
            for j=1:numel(q.count), current=current+q.count(j)*state.locationCovariance(:,:,j); end
            scatter=scatter+probabilities(h)*current;
        end
        C(:,:,k)=boundCovariance(scatter/mass,p);
    end
end

function [pi,epsilon]=observationWeights(gamma,b,B,p)
    K=size(gamma,2)-1; mass=zeros(B,K+1);
    for k=1:K+1, mass(:,k)=accumarray(b,gamma(:,k),[B 1]); end
    foreground=sum(mass(:,2:end),2);
    pi=(mass(:,2:end)+p.mixturePseudocount/K)./(foreground+p.mixturePseudocount);
    beta=p.backgroundBetaPrior; epsilon=(mass(:,1)+beta(1)-1)./(sum(mass,2)+sum(beta)-2);
end

function value=lowerBound(post,gamma,b,pi,epsilon,area,p)
    K=numel(post); B=size(pi,1); alpha=1+p.mixturePseudocount/K;
    logWeights=[log(epsilon(b))-log(area),log1p(-epsilon(b))+log(pi(b,:))];
    entropy=-sum(gamma.*log(max(gamma,realmin)),'all'); beta=p.backgroundBetaPrior;
    prior=B*(gammaln(K*alpha)-K*gammaln(alpha))+(alpha-1)*sum(log(pi),'all') + ...
        sum((beta(1)-1)*log(epsilon)+(beta(2)-1)*log1p(-epsilon)-betaln(beta(1),beta(2)));
    value=sum(cellfun(@(q) q.logNormalizer,post))+sum(gamma.*logWeights,'all')+entropy+prior;
end

function score=predictiveScore(fit,x,b)
% Composite marginal prediction: average within held-out blocks, then blocks.
% No held-out offsets or weights are fitted. This is not a joint block density.
    K=numel(fit.posterior); weights=mean(fit.pi,1); background=mean(fit.epsilon);
    logs=zeros(size(x,1),2*K+1); logs(:,1)=log(background)-log(fit.area);
    for k=1:K
        q=fit.posterior{k}; hypotheses={q.stable,q.variable}; probabilities=[q.r 1-q.r];
        offsets={fit.QS(:,:,k),fit.QV(:,:,k)};
        for h=1:2
            state=hypotheses{h}; covariance=fit.C(:,:,k)+offsets{h}+state.covariance;
            logs(:,2*k+h-1)=log1p(-background)+log(weights(k))+log(max(probabilities(h),realmin)) + ...
                logGaussian(x,state.mean.',covariance);
        end
    end
    likelihood=logSumExp(logs,2); [~,~,group]=unique(b);
    score=mean(accumarray(group,likelihood,[],@mean));
end

function [components,cells,backgroundMass]=publishTile(fit,xy,core,center,tile,p,raw,groups,cfg)
    K=numel(fit.posterior); components=repmat(emptyComponent(),K,1);
    [cells,~,cellGroup]=unique(floor((xy(core,:)-p.tileOrigin)/p.referenceResolution),'rows');
    assignment=fit.gamma(core,:); allocation=zeros(size(cells,1),K+1);
    for j=1:size(cells,1)
        evidence=mean(assignment(cellGroup==j,:),1);
        allocation(j,:)=evidence/sum(evidence);
    end
    backgroundMass=allocation(:,1)*p.referenceResolution^2;
    allocation=allocation(:,2:end);
    referenceMass=sum(allocation,1)*p.referenceResolution^2;
    threshold=p.publishFalsePositiveCost/(p.publishFalsePositiveCost+p.publishFalseNegativeCost);
    for k=1:K
        q=fit.posterior{k}; c=emptyComponent();
        c.id=string(tile(1))+":"+string(tile(2))+":"+string(k); c.ownerTile=tile;
        c.mean=(q.stable.mean+center.').';
        c.withinBlockCovariance=fit.C(:,:,k); c.stableOffsetCovariance=fit.QS(:,:,k);
        c.referenceMeanCovariance=q.stable.covariance;
        c.covariance=fit.C(:,:,k)+fit.QS(:,:,k)+q.stable.covariance;
        c.repeatability=q.r; c.observedBlockCount=nnz(q.count>=p.minBlockEffectiveCount);
        c.referenceMass=referenceMass(k); c.status="variable";
        if c.observedBlockCount<p.minObservedBlocks, c.status="unconfirmed";
        elseif q.r>threshold && sum(q.count)>=p.minComponentPoints, c.status="repeatable"; end
        c.published=c.status=="repeatable" && c.referenceMass>0;
        c.mass=c.referenceMass*c.repeatability*double(c.published);
        [c.meanXYZ,c.covarianceXYZ,c.heightAvailable,c.heightStatus]=conditionalHeight( ...
            raw,groups,fit.gamma(:,k+1),c.mean,c.covariance,p,cfg);
        c.blockEffectiveCounts=q.count; c.blockOffsetsStable=q.stable.locationMean-q.stable.mean.';
        c.referenceCellMass=allocation(:,k)*p.referenceResolution^2; components(k)=c;
    end
end

function [meanXYZ,covXYZ,available,status]=conditionalHeight(raw,groups,w,mu,Sigma,p,cfg)
    meanXYZ=[mu NaN]; covXYZ=nan(3); available=false; status="missing";
    if size(raw,2)<3, return; end
    coordinates=zeros(0,2); height=zeros(0,1); weights=zeros(0,1);
    for j=1:numel(groups)
        values=unique(raw(groups{j},:),'rows'); values=values(isfinite(values(:,3)),:);
        if isempty(values) || w(j)<1e-8, continue; end
        coordinates=[coordinates;values(:,1:2)]; %#ok<AGROW>
        height=[height;values(:,3)]; %#ok<AGROW>
        weights=[weights;repmat(w(j)/size(values,1),size(values,1),1)]; %#ok<AGROW>
    end
    if numel(height)<3, return; end
    design=[ones(size(height)),coordinates-mu]; root=sqrt(weights);
    coefficient=lsqminnorm(design.*root,height.*root);
    residual=height-design*coefficient; variance=sum(weights.*residual.^2)/sum(weights);
    lower=residual<=0; upper=~lower;
    if any(lower) && any(upper)
        a=sum(weights(lower).*residual(lower))/sum(weights(lower));
        z=sum(weights(upper).*residual(upper))/sum(weights(upper));
        within=(sum(weights(lower).*(residual(lower)-a).^2)+ ...
            sum(weights(upper).*(residual(upper)-z).^2))/sum(weights);
        balanced=min(sum(weights(lower)),sum(weights(upper)))/sum(weights)>0.15;
        if balanced && z-a>p.heightModeSeparation && within<p.heightModeVarianceRatio*variance
            status="inadequateMultipleModes"; return;
        end
    end
    slope=coefficient(2:3); cross=Sigma*slope; meanXYZ=[mu coefficient(1)];
    covXYZ=[Sigma cross;cross.' max(variance,cfg.minimumConditionalHeightVariance)+slope.'*cross];
    available=true; status="conditionalGaussian";
end

function c=emptyComponent()
    c=struct('id',"",'ownerTile',[0 0],'mean',[0 0],'covariance',eye(2), ...
        'withinBlockCovariance',eye(2),'stableOffsetCovariance',eye(2), ...
        'referenceMeanCovariance',eye(2),'repeatability',0,'observedBlockCount',0, ...
        'referenceMass',0,'mass',0,'status',"unconfirmed",'published',false, ...
        'meanXYZ',[0 0 NaN],'covarianceXYZ',nan(3),'heightAvailable',false,'heightStatus',"missing", ...
        'blockEffectiveCounts',zeros(0,1),'blockOffsetsStable',zeros(0,2),'referenceCellMass',zeros(0,1));
end

function d=fitDiagnostics(fit,p)
    d=struct('objective',"structuredVariationalLowerBoundWithParameterPriors", ...
        'objectiveTrace',fit.trace,'parameterChange',fit.parameterChange, ...
        'converged',fit.converged,'iterationCount',numel(fit.trace)-1, ...
        'blockMixtureWeights',fit.pi,'backgroundFractions',fit.epsilon, ...
        'backgroundBoundsLocal',fit.bounds,'backgroundDensity',1/fit.area, ...
        'responsibilityRowSums',sum(fit.gamma,2),'responsibilities',zeros(0,0));
    if p.storeEmAssignments, d.responsibilities=fit.gamma; end
end

function vector=parameterVector(C,pi,epsilon,post,gamma)
    vector=[C(:);pi(:);epsilon(:);gamma(:)];
    for k=1:numel(post)
        q=post{k}; vector=[vector;q.r;q.stable.mean;q.variable.mean; ...
            q.stable.covariance(:);q.variable.covariance(:); ...
            q.stable.locationMean(:);q.variable.locationMean(:); ...
            q.stable.locationCovariance(:);q.variable.locationCovariance(:)]; %#ok<AGROW>
    end
end

function C=boundCovariance(S,p)
    [vectors,values]=eig((S+S.')/2,'vector');
    C=vectors*diag(min(max(values,p.minCovarianceEigenvalue),p.maxCovarianceEigenvalue))*vectors.'; C=(C+C.')/2;
end

function value=logGaussian(x,mu,C)
    delta=x-mu; value=-log(2*pi)-0.5*logDet(C)-0.5*sum((delta/C).*delta,2);
end

function value=logDet(C)
    factor=chol((C+C.')/2); value=2*sum(log(diag(factor)));
end

function value=logSumExp(x,dimension)
    peak=max(x,[],dimension); value=peak+log(sum(exp(x-peak),dimension));
end
