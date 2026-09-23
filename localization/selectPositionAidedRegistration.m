function result=selectPositionAidedRegistration(solve,initialPose,aid,cfg,additionalSeeds)
% selectPositionAidedRegistration Resolve local LiDAR modes with position aid.
% aid.position/covariance refer to the map/observer point at scan acquisition.
% Source/map stability remains inside solve. No GNSS term is added to its
% objective or Hessian. Selection still creates statistical dependence on
% GNSS; the exported matrix is conditional geometry, not independent evidence.
% Hypothesis scores are engineering compatibility scores, not calibrated
% posterior probabilities. Between-mode disagreement can only reduce information.
    initialPose=double(initialPose(:).');
    if nargin<5,additionalSeeds=zeros(0,3);end
    assert(size(additionalSeeds,2)==3 && isreal(additionalSeeds) && all(isfinite(additionalSeeds),'all'), ...
        'VehicleLocalization:InvalidRegistrationSeed','Additional seeds must be finite SE(2) poses.');
    result=solve(initialPose);
    result.positionAiding=struct('used',false,'reason',"unavailable", ...
        'selected',1,'candidateCount',1,'gnssInformationAdded',false, ...
        'selectionDependsOnGnss',false);
    if isempty(aid) || (isfield(aid,'valid') && ~aid.valid),return;end
    a=cfg.positionAid;
    parameters=[a.maximumStandardDeviation,a.standardDeviationFloor,a.minimumSeedSeparation, ...
        a.hypothesisSeparation,a.maximumSquaredInnovation];
    assert(all(isfinite(parameters) & parameters>0), ...
        'VehicleLocalization:InvalidPositionAidConfig','Position-aid thresholds must be finite and positive.');
    validateattributes(aid.position,{'numeric'},{'real','finite','numel',2});
    C=double(aid.covariance);
    assert(isequal(size(C),[2,2]) && all(isfinite(C),'all') && ...
        norm(C-C.','fro')<1e-9 && min(eig(C))>0, ...
        'VehicleLocalization:InvalidPositionAid','Require positive definite position covariance.');
    if sqrt(max(eig(C)))>a.maximumStandardDeviation
        result.positionAiding.reason="uncertainPosition";return;
    end
    C=C+a.standardDeviationFloor^2*eye(2);
    seeds=initialPose;gnssSeed=[double(aid.position(:).'),initialPose(3)];
    candidates={result};
    if norm(gnssSeed(1:2)-initialPose(1:2))>=a.minimumSeedSeparation
        seeds(end+1,:)=gnssSeed;candidates{end+1}=solve(gnssSeed);
    end
    for k=1:size(additionalSeeds,1)
        delta=seeds-additionalSeeds(k,:);delta(:,3)=wrap(delta(:,3));
        if all(vecnorm(delta.*[1 1 cfg.yawLeverArm],2,2)>=a.minimumSeedSeparation)
            seeds(end+1,:)=additionalSeeds(k,:);candidates{end+1}=solve(additionalSeeds(k,:)); %#ok<AGROW>
        end
    end
    count=numel(candidates);scores=inf(count,1);innovation=scores;
    poses=zeros(count,3);similarity=zeros(count,1);valid=false(count,1);
    for k=1:count
        r=candidates{k};poses(k,:)=r.poseXYTheta;similarity(k)=r.similarity;
        delta=r.poseXYTheta(1:2)-aid.position(:).';innovation(k)=delta/C*delta.';
        valid(k)=r.accepted || r.directionalAccepted;
        if valid(k)
            scores(k)=-2*log(max(r.similarity,realmin))+innovation(k);
        end
    end
    [~,selected]=min(scores);
    result=candidates{selected};
    distinct=false(count,1);representatives=zeros(0,1);[~,order]=sort(scores);
    for k=order(:).'
        if ~valid(k),continue;end
        delta=poses(representatives,:)-poses(k,:);delta(:,3)=wrap(delta(:,3));
        if all(vecnorm(delta.*[1 1 cfg.yawLeverArm],2,2)>=a.hypothesisSeparation)
            distinct(k)=true;representatives(end+1,1)=k; %#ok<AGROW>
        end
    end
    probabilities=zeros(count,1);eligible=valid & distinct;
    spread=zeros(3);
    if any(eligible)
        probabilities(eligible)=exp(-.5*(scores(eligible)-min(scores(eligible))));
        probabilities=probabilities/sum(probabilities);
        for k=find(eligible).'
            delta=poses(k,:)-poses(selected,:);delta(3)=wrap(delta(3));
            spread=spread+probabilities(k)*(delta.'*delta);
        end
        % Preserve genuine null directions and never increase geometry information.
        result.information=reduceInformation(result.information,spread);
        result.directionalInformation=reduceInformation(result.directionalInformation,spread);
    end
    result.positionAiding=struct('used',true,'reason',"hypothesisSelection", ...
        'selected',selected,'candidateCount',count,'seedPoses',seeds(1:count,:), ...
        'candidatePoses',poses,'geometricallyValid',valid,'similarity',similarity, ...
        'scores',scores,'squaredInnovation',innovation,'relativeSupport',probabilities, ...
        'betweenHypothesisSecondMoment',spread,'gnssInformationAdded',false, ...
        'selectionDependsOnGnss',true,'informationSemantics',"conditionalLiDARGeometryWithModeDisagreement");
    if valid(selected) && innovation(selected)>a.maximumSquaredInnovation
        result.accepted=false;result.directionalAccepted=false;
        result.reason="positionAidConflict";result.positionAiding.reason="selectedOptimumConflictsWithPosition";
    end
end

function information=reduceInformation(information,spread)
    I=(information+information.')/2;
    [V,D]=eig(I);root=V*diag(sqrt(max(0,diag(D))))*V.';
    information=root*((eye(3)+root*spread*root)\root);
    information=(information+information.')/2;
end

function x=wrap(x)
    x=atan2(sin(x),cos(x));
end
