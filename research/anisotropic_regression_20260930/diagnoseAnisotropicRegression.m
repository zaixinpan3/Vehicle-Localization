function diagnoseAnisotropicRegression()
% diagnoseAnisotropicRegression Isolate the changed geometry on identical data.
% Reference-seeded association anchors are offline diagnostics, not deployment.
% Every intervention preserves source membership and class-balanced weights.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    out='output/anisotropic_regression_20260930';
    if ~isfolder(out),mkdir(out);end
    a=load('output/source_shape_matching_20260929/shape50.mat','sources');
    map=load(featureMapBuildConfig().probabilityCloudPath,'cloud');
    old=load('output/line_direction_matching_20260928/production/report.mat','report');
    baseline=readtable('research/source_shape_matching_20260929/final_raw.csv');
    cfg=distributionRegistrationConfig();ncfg=anisotropicRegistrationConfig();scale=[1;1;.1];
    summary=cell(0,14);forceRows=cell(0,13);pairRows=cell(0,24);associationRows=cell(0,23);ratioRows=cell(0,8);contexts=cell(3,1);
    for index=1:3
        frame=[178 894 932];k=frame(index);source=a.sources{k};
        seed=baseline{k,{'x','y','psi'}};ref=old.report.calls{k,{'referenceX','referenceY','referencePsi'}};
        [fixed,globalIds]=selectLocalProbabilityCloud(map.cloud,seed,cfg.localMapRadius);
        fixed=conditionSemanticMapOnView(fixed,seed);fixed=rmfield(fixed,'landmarkViews');
        c=cfg;c.pyramid.mapMergeRadius=0;c.pyramid.sourceMergeRadius=0;
        n=ncfg;n.pyramid.mapMergeRadius=0;n.pyramid.sourceMergeRadius=0;
        legacy=prepareSemanticRegistrationGeometry(fixed,source,seed,c);
        modern=prepareSemanticRegistrationGeometry(fixed,source,seed,n);
        poseRef=[ref(1:2)-seed(1:2),ref(3)];
        oldAnchor=legacy.linearize(poseRef,scale);newAnchor=modern.linearize(poseRef,scale);
        ctx=struct('frame',k,'seed',seed,'reference',ref,'fixed',fixed,'source',source, ...
            'legacy',legacy,'modern',modern,'oldAnchor',oldAnchor,'newAnchor',newAnchor,'globalIds',globalIds,'cfg',c);
        modes=["legacy_dynamic","full_dynamic","full_dynamic_split","full_fixed_new","full_fixed_new_split","position_fixed_new", ...
            "frozen_cov_position_fixed_new","full_fixed_old","legacy_fixed_old", ...
            "normal_fixed_old","normal_direction_fixed_old","full_direction_fixed_new", ...
            "partial_fixed_new","partial_direction_fixed_new","curb_normal_fixed_new", ...
            "curb_normal_direction_fixed_new","curb_and_partial_direction_fixed_new"];
        controls=cell(numel(modes),1);
        for j=1:numel(modes)
            mode=modes(j);model=controlModel(ctx,mode);
            result=solve(model,seed,c);controls{j}=result;
            delta=result.pose-ref;body=delta(1:2)*rotation(ref(3));
            summary(end+1,:)={k,mode,norm(delta(1:2)),body(1),body(2),rad2deg(wrap(delta(3))), ...
                result.rank,result.converged,result.iterations,result.system.cost, ...
                result.system.numPairs,result.eigenvalues(1),result.eigenvalues(2),result.eigenvalues(3)}; %#ok<AGROW>
            % Verify each actual control's gradient against its own frozen
            % cost. For dynamic legacy this checks its frozen metric contract.
            testPose=poseRef+[.01 -.015 .002];s=model.linearize(testPose,scale);numeric=zeros(3,1);
            for axis=1:3
                step=zeros(1,3);step(axis)=1e-6*scale(axis);
                numeric(axis)=(model.frozenCost(s,testPose+step)-model.frozenCost(s,testPose-step))/(2e-6);
            end
            assert(max(abs(numeric-2*s.gradient))<2e-5,'Control gradient mismatch: %s frame %d',mode,k);
        end
        liveOld=registerSemanticProbabilityCloud(fixed,source,seed,c);
        liveNew=registerSemanticProbabilityCloud(fixed,source,seed,n);
        assert(max(abs(controls{1}.pose-liveOld.poseXYTheta))<1e-7);
        assert(max(abs(controls{2}.pose-liveNew.poseXYTheta))<1e-7);
        forceRows=[forceRows;forceDecomposition(ctx,poseRef,scale)]; %#ok<AGROW>
        pairRows=[pairRows;pairGeometry(ctx,poseRef)]; %#ok<AGROW>
        associationRows=[associationRows;associationAnalysis(ctx,poseRef)]; %#ok<AGROW>
        for mode=["full_fixed_new","full_direction_fixed_new"]
            for ratio=[.001 .005 .01 .02 .05]
                rc=c;rc.geometric.minimumObservabilityRatio=ratio;
                rr=solve(controlModel(ctx,mode),seed,rc);difference=rr.pose-ref;
                ratioRows(end+1,:)={k,mode,ratio,norm(difference(1:2)),rad2deg(wrap(difference(3))),rr.rank,rr.eigenvalues(1)/rr.eigenvalues(3),rr.converged}; %#ok<AGROW>
            end
        end
        ctx=rmfield(ctx,{'legacy','modern'});ctx.controls=controls;ctx.modes=modes;
        contexts{index}=ctx;
        fprintf('Frame %d: production fine-solver parity and %d control gradients passed\n',k,numel(modes));
    end
    controls=cell2table(summary,VariableNames={'frame','variant','errorM','longitudinalM','lateralM','yawErrorDeg','rank','converged','iterations','cost','pairs','lambda1','lambda2','lambda3'});
    forces=cell2table(forceRows,VariableNames={'frame','class','part','gradientLongitudinal','gradientLateral','gradientScaledYaw','weightedPositionQ','weightedShapeQ','robustWeightSum','rawWeightSum','informationTrace','pairs','cost'});
    geometry=cell2table(pairRows,VariableNames={'frame','source','globalTarget','class','sourceAnisotropy','targetAnisotropy','sumAnisotropy','sourceMapAxisDifferenceDeg','neighborMapAxisDifferenceDeg','neighborUsed','oldTargetEligible','oldPartial','normalOffsetM','tangentOffsetM','positionQ','shapeQ','weight','robustWeight','sourceMinor','sourceMajor','targetMinor','targetMajor','intrinsicAnisotropy','intrinsicTotalAxisDifferenceDeg'});
    writetable(controls,fullfile(dest,'controls.csv'));writetable(forces,fullfile(dest,'reference_forces.csv'));
    writetable(geometry,fullfile(dest,'reference_geometry.csv'));
    associations=cell2table(associationRows,VariableNames={'frame','source','class','oldGlobalTarget','newGlobalTarget','changed', ...
        'oldNeighborQ','newNeighborQ','oldPositionQ','newPositionQ','oldShapeQ','newShapeQ', ...
        'oldPriorQ','newPriorQ','oldNewMeanDifferenceM','newOldMeanDifferenceLongitudinalM','newOldMeanDifferenceLateralM','oldWeight','newWeight','legacyScoreOldTarget','legacyScoreNewTarget','modernScoreOldTarget','modernScoreNewTarget'});
    ratios=cell2table(ratioRows,VariableNames={'frame','variant','observabilityRatio','errorM','yawErrorDeg','rank','actualEigenvalueRatio','converged'});
    writetable(associations,fullfile(dest,'associations.csv'));writetable(ratios,fullfile(dest,'observability_controls.csv'));
    synthetic=syntheticCurvature();writetable(synthetic,fullfile(dest,'synthetic_curvature.csv'));
    objectives=crossObjectives(contexts);writetable(objectives,fullfile(dest,'cross_objectives.csv'));
    curvature=curvatureAnalysis(contexts);writetable(curvature,fullfile(dest,'shape_curvature.csv'));
    save(fullfile(out,'diagnosis.mat'),'controls','forces','geometry','curvature','synthetic','objectives','associations','ratios','contexts','-v7.3');
    disp(controls);disp(forces);
end

function model=controlModel(ctx,mode)
    if mode=="legacy_dynamic",model=ctx.legacy;return;end
    if mode=="full_dynamic",model=ctx.modern;return;end
    if mode=="full_dynamic_split"
        model=ctx.modern;
        model.linearize=@(pose,scale)splitDynamic(ctx,pose,scale);
        return;
    end
    anchor=ctx.newAnchor;
    if any(mode==["full_fixed_old","legacy_fixed_old","normal_fixed_old","normal_direction_fixed_old"]),anchor=ctx.oldAnchor;end
    model=struct('linearize',@(pose,scale)factors(ctx,anchor,pose,scale,mode), ...
        'frozenCost',@(system,pose)cost(ctx,anchor,pose,mode,system));
end

function s=factors(ctx,anchor,pose,scale,mode)
    ids=anchor.pairs.source;cm=ctx.source.components.covariance(:,:,ids);
    if isfield(anchor,'targetCovariance')
        cf=anchor.targetCovariance;
    else
        x=anchor.pairs.targetCovarianceXY;cf=zeros(2,2,numel(ids));
        cf(1,1,:)=x(:,1);cf(1,2,:)=x(:,2);cf(2,1,:)=x(:,2);cf(2,2,:)=x(:,3);
    end
    mean=ctx.source.components.mean(ids,1:2);target=anchor.targetMean;noise=ctx.cfg.geometric.noiseStandardDeviation;
    [residual,J,~]=gaussianRegistrationResiduals(mean,cm,target,cf,pose,noise);
    J=J.*reshape(scale,1,3,1);r=rotation(pose(3));delta=mean*r.'+pose(1:2)-target;
    if mode=="legacy_fixed_old" || mode=="frozen_cov_position_fixed_new"
        precision=anchor.precision;
        residual(1:2,:)=reshape(pagemtimes(precision,permute(delta,[2 3 1])),2,[]);
        J(1:2,:,:)=0;J(1:2,1:2,:)=precision.*reshape(scale(1:2),1,2,1);
        derivative=mean*[r(:,2),-r(:,1)].';
        J(1:2,3,:)=pagemtimes(precision,permute(derivative*scale(3),[2 3 1]));
    end
    onlyPosition=any(mode==["position_fixed_new","frozen_cov_position_fixed_new","normal_fixed_old"]);
    direction=any(mode==["legacy_fixed_old","normal_direction_fixed_old","full_direction_fixed_new", ...
        "partial_direction_fixed_new","curb_normal_direction_fixed_new","curb_and_partial_direction_fixed_new"]);
    if onlyPosition || direction
        residual(3,:)=0;J(3,:,:)=0;
    end
    normalOnly=false(numel(ids),1);normals=ctx.legacy.fixed.normal(anchor.pairs.target,:);
    if any(mode==["normal_fixed_old","normal_direction_fixed_old"])
        normalOnly=any(anchor.pairs.semanticName==["curb","facade"],2)|anchor.pairs.partialSignSurface;
    end
    if any(mode==["partial_fixed_new","partial_direction_fixed_new","curb_and_partial_direction_fixed_new"])
        oldPartial=ctx.oldAnchor.pairs.source(ctx.oldAnchor.pairs.partialSignSurface);
        normalOnly=normalOnly|ismember(ids,oldPartial);
        [present,oldIndex]=ismember(ids,ctx.oldAnchor.pairs.source);
        for j=find(present & ismember(ids,oldPartial)).'
            normals(j,:)=ctx.legacy.fixed.normal(ctx.oldAnchor.pairs.target(oldIndex(j)),:);
        end
    end
    if any(mode==["curb_normal_fixed_new","curb_normal_direction_fixed_new","curb_and_partial_direction_fixed_new"])
        normalOnly=normalOnly|any(anchor.pairs.semanticName==["curb","facade"],2);
    end
    rotated=pagemtimes(pagemtimes(r,cm),r.');sumCov=rotated+cf+noise^2*eye(2);
    derivative=mean*[r(:,2),-r(:,1)].';
    for j=find(normalOnly).'
        normal=normals(j,:);variance=normal*sumCov(:,:,j)*normal.';
        rc=rotated(:,:,j);covDerivative=[-2*rc(1,2),rc(1,1)-rc(2,2);rc(1,1)-rc(2,2),2*rc(1,2)];
        residual(1,j)=dot(normal,delta(j,:))/sqrt(variance);residual(2,j)=0;
        J(1,:,j)=[normal/sqrt(variance),dot(normal,derivative(j,:))/sqrt(variance)- ...
            .5*residual(1,j)*(normal*covDerivative*normal.')/variance].*scale.';
        J(2,:,j)=0;
    end
    if direction
        tangent=ctx.legacy.moving.lineTangent(ids,:);sigma=ctx.legacy.moving.lineDirectionSigma(ids);
        used=ctx.legacy.moving.lineDirectionValid(ids) & any(anchor.pairs.semanticName==["curb","facade"],2);
        directionNormal=ctx.legacy.fixed.normal(anchor.pairs.target,:);
        residual(3,:)=(sum((tangent*r.').*directionNormal,2).*used./sigma).';
        J(3,3,:)=sum((tangent*[r(:,2),-r(:,1)].').*directionNormal,2).*used./sigma*scale(3);
    end
    if mode=="full_fixed_new_split"
        [residual,J]=splitResidual(residual,J,cm,cf,pose(3),noise,scale);
    end
    q=sum(residual.^2,1).';weights=anchor.weights;robust=1./(1+q/ctx.cfg.geometric.robustStandardizedDistance^2);
    stacked=reshape(permute(J,[1 3 2]),[],3);rowWeights=repelem(weights.*robust,size(residual,1),1);
    s=struct('J',J,'residual',residual,'H',stacked.'*(stacked.*rowWeights),'gradient',stacked.'*(residual(:).*rowWeights), ...
        'weights',weights,'robust',robust,'numPairs',numel(ids),'cost',sum(weights.*ctx.cfg.geometric.robustStandardizedDistance^2.*log1p(q/ctx.cfg.geometric.robustStandardizedDistance^2)));
end
function value=cost(ctx,anchor,pose,mode,~)
    s=factors(ctx,anchor,pose,ones(3,1),mode);value=s.cost;
end

function result=solve(model,seed,cfg)
    scale=[1;1;1/cfg.yawLeverArm];bounds=cfg.maximumPoseCorrection(:)./scale;q=zeros(3,1);converged=false;
    for iteration=1:cfg.maximumIterationsPerScale
        pose=[q(1:2).',seed(3)+q(3)*scale(3)];s=model.linearize(pose,scale);
        if s.numPairs<cfg.minimumComponents,break;end
        [step,~,rank]=observable(s.H,s.gradient,cfg.geometric.minimumObservabilityRatio);
        if rank==0,break;end
        if norm(step,inf)<cfg.stepTolerance,converged=true;break;end
        step=step/max(1,norm(step));accepted=false;
        for lineSearch=0:15
            trial=max(-bounds,min(bounds,q+step*2^(-lineSearch)));
            trialPose=[trial(1:2).',seed(3)+trial(3)*scale(3)];
            if model.frozenCost(s,trialPose)<s.cost-1e-12,q=trial;accepted=true;break;end
        end
        if ~accepted,converged=norm(step,inf)<10*cfg.stepTolerance;break;end
    end
    pose=[q(1:2).',seed(3)+q(3)*scale(3)];s=model.linearize(pose,scale);
    [~,projector,rank,e]=observable(s.H,s.gradient,cfg.geometric.minimumObservabilityRatio);
    if rank<3
        q=projector*q;pose=[q(1:2).',seed(3)+q(3)*scale(3)];s=model.linearize(pose,scale);
        [~,~,rank,e]=observable(s.H,s.gradient,cfg.geometric.minimumObservabilityRatio);
    end
    result=struct('pose',seed+(q.*scale).','rank',rank,'eigenvalues',e,'converged',converged,'iterations',iteration,'system',s);
    result.pose(3)=wrap(result.pose(3));
end
function [step,P,rank,e]=observable(H,g,ratio)
    [v,e]=eig((H+H.')/2,'vector');keep=e>max(1e-8,ratio*max(e));rank=nnz(keep);
    P=v(:,keep)*v(:,keep).';step=-v(:,keep)*((v(:,keep).'*g)./e(keep));
end

function rows=forceDecomposition(ctx,pose,scale)
    s=ctx.modern.linearize(pose,scale);r=rotation(pose(3));ids=s.pairs.source;means=ctx.source.components.mean(ids,1:2);
    derivative=means*[r(:,2),-r(:,1)].';center=s.J(1:2,:,:);center(:,3,:)=pagemtimes(s.precision,permute(derivative*scale(3),[2 3 1]));
    covariance=s.J(1:2,:,:)-center;rows=cell(0,13);
    for name=unique(s.pairs.semanticName).'
        selected=find(s.pairs.semanticName==name);
        for part=["mean_position","covariance_rotation","shape_rotation","total"]
            g=zeros(3,1);H=zeros(3);qp=0;qs=0;value=0;
            for j=selected.'
                switch part
                    case "mean_position",J=center(:,:,j);residual=s.residual(1:2,j);
                    case "covariance_rotation",J=covariance(:,:,j);residual=s.residual(1:2,j);
                    case "shape_rotation",J=s.J(3,:,j);residual=s.residual(3,j);
                    otherwise,J=s.J(:,:,j);residual=s.residual(:,j);
                end
                w=s.weights(j)*s.robust(j);g=g+w*J.'*residual;H=H+w*(J.'*J);
                qp=qp+s.weights(j)*sum(s.residual(1:2,j).^2);qs=qs+s.weights(j)*s.residual(3,j)^2;
                value=value+s.weights(j)*ctx.cfg.geometric.robustStandardizedDistance^2*log1p(sum(s.residual(:,j).^2)/ctx.cfg.geometric.robustStandardizedDistance^2);
            end
            body=g(1:2).'*rotation(ctx.reference(3));
            rows(end+1,:)={ctx.frame,name,part,body(1),body(2),g(3),qp,qs,sum(s.weights(selected).*s.robust(selected)),sum(s.weights(selected)),trace(H),numel(selected),value}; %#ok<AGROW>
        end
    end
end
function rows=pairGeometry(ctx,pose)
    s=ctx.newAnchor;rows=cell(s.numPairs,24);r=rotation(pose(3));
    old=ctx.oldAnchor;[present,oldIndex]=ismember(s.pairs.source,old.pairs.source);
    for j=1:s.numPairs
        id=s.pairs.source(j);target=s.pairs.target(j);cm=r*ctx.source.components.covariance(:,:,id)*r.';cf=s.targetCovariance(:,:,j);
        [sm,sM,sa]=axes(cm);[tm,tM,ta]=axes(cf);[sumMinor,sumMajor,~]=axes(cm+cf+ctx.cfg.geometric.noiseStandardDeviation^2*eye(2));
        normal=[-sin(ta),cos(ta)];tangent=[cos(ta),sin(ta)];
        delta=ctx.source.components.mean(id,1:2)*r.'+pose(1:2)-s.targetMean(j,:);
        neighbor=ctx.legacy.moving.lineTangent(id,:)*r.';used=ctx.legacy.moving.lineDirectionValid(id);
        neighborAngle=atan2(neighbor(2),neighbor(1));intrinsicRatio=NaN;intrinsicDifference=NaN;
        if isfield(ctx.fixed.components,'intrinsicCovariance') && any(ctx.fixed.components.intrinsicCovariance(:,:,target),'all')
            [im,iM,ia]=axes(ctx.fixed.components.intrinsicCovariance(:,:,target));intrinsicRatio=iM/max(im,eps);intrinsicDifference=rad2deg(axisDifference(ta,ia));
        end
        partial=false;if present(j),partial=old.pairs.partialSignSurface(oldIndex(j));end
        rows(j,:)={ctx.frame,id,ctx.globalIds(target),s.pairs.semanticName(j),sM/sm,tM/tm,sumMajor/sumMinor, ...
            rad2deg(axisDifference(sa,ta)),rad2deg(axisDifference(neighborAngle,ta)),used,ctx.legacy.fixed.lineEligible(target),partial, ...
            dot(delta,normal),dot(delta,tangent),sum(s.residual(1:2,j).^2),s.residual(3,j)^2,s.weights(j),s.weights(j)*s.robust(j),sm,sM,tm,tM,intrinsicRatio,intrinsicDifference};
    end
end
function [minor,major,angle]=axes(C)
    [v,e]=eig((C+C.')/2,'vector');minor=min(e);[major,j]=max(e);angle=atan2(v(2,j),v(1,j));
end
function a=axisDifference(a,b),a=.5*atan2(sin(2*(a-b)),cos(2*(a-b)));end
function r=rotation(a),r=[cos(a) -sin(a);sin(a) cos(a)];end
function a=wrap(a),a=atan2(sin(a),cos(a));end


function s=splitDynamic(ctx,pose,scale)
    s=ctx.modern.linearize(pose,scale);original=s;
    ids=s.pairs.source;
    [s.residual,s.J]=splitResidual(s.residual,s.J,ctx.source.components.covariance(:,:,ids), ...
        s.targetCovariance,pose(3),ctx.cfg.geometric.noiseStandardDeviation,scale);
    A=reshape(permute(s.J,[1 3 2]),[],3);w=repelem(s.weights.*s.robust,size(s.residual,1),1);
    s.H=A.'*(A.*w);s.gradient=A.'*(s.residual(:).*w);
    assert(max(abs(s.gradient-original.gradient))<1e-9);
    assert(max(abs(sum(s.residual.^2,1)-sum(original.residual.^2,1)))<1e-10);
end
function [residual,J]=splitResidual(residual,J,sourceCov,targetCov,yaw,noise,scale)
% Separate scale mismatch from orientation before Gauss-Newton linearization.
% The cost and gradient remain exactly the same; positive angular curvature
% is no longer discarded into the residual-times-second-derivative term.
    r=rotation(yaw);sourceCov=pagemtimes(pagemtimes(r,sourceCov),r.');
    a=reshape(sourceCov(1,1,:),[],1);b=reshape(sourceCov(1,2,:),[],1);d=reshape(sourceCov(2,2,:),[],1);
    x=reshape(targetCov(1,1,:),[],1);y=reshape(targetCov(1,2,:),[],1);z=reshape(targetCov(2,2,:),[],1);
    gapA=hypot(a-d,2*b);gapB=hypot(x-z,2*y);trace=a+d+x+z+2*noise^2;
    alignedDet=(trace+gapA+gapB).*(trace-gapA-gapB)/4;k=gapA.*gapB./alignedDet;
    angle=.5*(atan2(2*b,a-d)-atan2(2*y,x-z));v=sin(angle);u=k.*v.^2;penalty=log1p(u);
    shape=residual(3,:).'.^2;constant=max(0,shape-penalty);
    ratio=ones(size(u));nonzero=u>1e-12;ratio(nonzero)=log1p(u(nonzero))./u(nonzero);
    orientation=v.*sqrt(k.*ratio);
    derivative=sqrt(k).*cos(angle);active=penalty>1e-12;
    derivative(active)=sign(v(active)).*k(active).*v(active).*cos(angle(active))./((1+u(active)).*sqrt(penalty(active)));
    residual(3,:)=orientation.';residual(4,:)=sqrt(constant).';
    J(3,:,:)=0;J(3,3,:)=derivative*scale(3);J(4,:,:)=0;
end
function report=curvatureAnalysis(contexts)
    rows=cell(0,8);
    for index=1:numel(contexts)
        ctx=contexts{index};s=ctx.newAnchor;c=ctx.source.components;ids=s.pairs.source;
        cf=s.targetCovariance;cm=c.covariance(:,:,ids);means=zeros(numel(ids),2);pose=[0 0 ctx.reference(3)];h=1e-4;
        r0=gaussianRegistrationResiduals(means,cm,means,cf,pose,ctx.cfg.geometric.noiseStandardDeviation);
        rp=gaussianRegistrationResiduals(means,cm,means,cf,pose+[0 0 h],ctx.cfg.geometric.noiseStandardDeviation);
        rm=gaussianRegistrationResiduals(means,cm,means,cf,pose-[0 0 h],ctx.cfg.geometric.noiseStandardDeviation);
        rawGN=reshape(s.J(3,3,:),[],1).^2/.01;
        trueInformation=(sum(rp.^2,1).'-2*sum(r0.^2,1).'+sum(rm.^2,1).')/(2*h^2);
        [~,splitJ]=splitResidual(s.residual,s.J,cm,cf,pose(3),ctx.cfg.geometric.noiseStandardDeviation,[1;1;.1]);
        splitGN=reshape(splitJ(3,3,:),[],1).^2/.01;
        old=ctx.oldAnchor;
        for name=unique(s.pairs.semanticName).'
            keep=s.pairs.semanticName==name;w=s.weights(keep).*s.robust(keep);
            oldKeep=old.pairs.semanticName==name;
            oldInfo=sum(reshape(old.J(3,3,oldKeep),[],1).^2.*old.weights(oldKeep).*old.robust(oldKeep))/.01;
            rows(end+1,:)={ctx.frame,name,sum(w.*rawGN(keep)),sum(w.*splitGN(keep)), ...
                sum(w.*trueInformation(keep)),oldInfo,sum(s.weights(keep).*s.residual(3,keep).'.^2),nnz(keep)}; %#ok<AGROW>
        end
    end
    report=cell2table(rows,VariableNames={'frame','class','originalShapeGnInformation','splitShapeGnInformation', ...
        'trueShapeHalfCurvature','legacyNeighborhoodInformation','weightedShapeQ','pairs'});
end

function rows=associationAnalysis(ctx,pose)
    old=ctx.oldAnchor;new=ctx.newAnchor;rows=cell(0,23);
    [present,index]=ismember(new.pairs.source,old.pairs.source);assert(all(present));
    assert(max(abs(new.weights-old.weights(index)))<1e-12);
    for j=1:new.numPairs
        id=new.pairs.source(j);ot=old.pairs.target(index(j));nt=new.pairs.target(j);
        sourceCov=ctx.source.components.covariance(:,:,id);sourceMean=ctx.source.components.mean(id,1:2);
        oldCov=ctx.fixed.components.covariance(:,:,ot);newCov=ctx.fixed.components.covariance(:,:,nt);
        oldMean=ctx.legacy.fixed.mean(ot,1:2);newMean=ctx.legacy.fixed.mean(nt,1:2);
        ro=gaussianRegistrationResiduals(sourceMean,sourceCov,oldMean,oldCov,pose,ctx.cfg.geometric.noiseStandardDeviation);
        rn=gaussianRegistrationResiduals(sourceMean,sourceCov,newMean,newCov,pose,ctx.cfg.geometric.noiseStandardDeviation);
        tangent=ctx.legacy.moving.lineTangent(id,:)*rotation(pose(3)).';
        no=ctx.legacy.fixed.normal(ot,:);nn=ctx.legacy.fixed.normal(nt,:);sigma=ctx.legacy.moving.lineDirectionSigma(id);
        qo=(dot(tangent,no)/sigma)^2;qn=(dot(tangent,nn)/sigma)^2;
        difference=(newMean-oldMean)*rotation(ctx.reference(3));
        mixture=ctx.fixed.components.mixtureWeight;po=-2*log(mixture(ot));pn=-2*log(mixture(nt));
        r=rotation(pose(3));deltaOld=sourceMean*r.'+pose(1:2)-oldMean;deltaNew=sourceMean*r.'+pose(1:2)-newMean;
        cm=r*sourceCov*r.';vo=no*(cm+oldCov+ctx.cfg.geometric.noiseStandardDeviation^2*eye(2))*no.';
        vn=nn*(cm+newCov+ctx.cfg.geometric.noiseStandardDeviation^2*eye(2))*nn.';
        to=[-no(2),no(1)];tn=[-nn(2),nn(1)];
        oldScoreOld=dot(no,deltaOld)^2/vo+dot(to,deltaOld)^2/(ctx.legacy.fixed.majorVariance(ot)+ctx.cfg.geometric.maximumMatchDistance^2)+qo+po;
        oldScoreNew=dot(nn,deltaNew)^2/vn+dot(tn,deltaNew)^2/(ctx.legacy.fixed.majorVariance(nt)+ctx.cfg.geometric.maximumMatchDistance^2)+qn+pn;
        newScoreOld=sum(ro.^2)+po;newScoreNew=sum(rn.^2)+pn;
        if ~any(new.pairs.semanticName(j)==["curb","facade"])
            sumOld=cm+oldCov+ctx.cfg.geometric.noiseStandardDeviation^2*eye(2);
            sumNew=cm+newCov+ctx.cfg.geometric.noiseStandardDeviation^2*eye(2);
            oldScoreOld=sum(ro(1:2).^2)+max(0,log(det(sumOld)/(4*sqrt(det(cm)*det(oldCov)))))+po;
            oldScoreNew=sum(rn(1:2).^2)+max(0,log(det(sumNew)/(4*sqrt(det(cm)*det(newCov)))))+pn;
        end
        rows(end+1,:)={ctx.frame,id,new.pairs.semanticName(j),ctx.globalIds(ot),ctx.globalIds(nt),ot~=nt,qo,qn, ...
            sum(ro(1:2).^2),sum(rn(1:2).^2),ro(3)^2,rn(3)^2,po,pn,norm(newMean-oldMean),difference(1),difference(2),old.weights(index(j)),new.weights(j),oldScoreOld,oldScoreNew,newScoreOld,newScoreNew}; %#ok<AGROW>
    end
end

function report=syntheticCurvature()
    source=diag([.16 .01]);target=diag([3 .03]);noise=.1;
    a=eig(source)+noise^2/2;b=eig(target)+noise^2/2;
    k=diff(a)*diff(b)/((a(1)+b(1))*(a(2)+b(2)));rows=cell(4,6);
    angles=[0 1e-4 .01 .1];
    for j=1:numel(angles)
        angle=angles(j);[residual,J]=gaussianRegistrationResiduals([0 0],source,[0 0],target,[0 0 angle],noise);
        [split,splitJ]=splitResidual(residual,J,source,target,angle,noise,ones(3,1));
        assert(abs(sum(residual.^2)-sum(split.^2))<1e-12);
        assert(max(abs(J.'*residual-splitJ.'*split))<1e-12);
        curvature=k*cos(2*angle)/(1+k*sin(angle)^2)-.5*k^2*sin(2*angle)^2/(1+k*sin(angle)^2)^2;
        rows(j,:)={angle,sum(residual.^2),J(3,3)^2,splitJ(3,3)^2,curvature,k};
    end
    assert(rows{1,3}==0 && abs(rows{1,4}-rows{1,5})<1e-12 && rows{1,5}>2.8);
    report=cell2table(rows,VariableNames={'yawRad','shapeCost','originalShapeGnInformation','splitShapeGnInformation','trueShapeHalfCurvature','kappa'});
end
function report=crossObjectives(contexts)
    rows=cell(0,7);
    for k=1:numel(contexts)
        ctx=contexts{k};legacy=prepareSemanticRegistrationGeometry(ctx.fixed,ctx.source,ctx.seed,ctx.cfg);
        modern=prepareSemanticRegistrationGeometry(ctx.fixed,ctx.source,ctx.seed,anisotropicRegistrationConfig());
        oldFit=ctx.controls{ctx.modes=="legacy_dynamic"}.pose;newFit=ctx.controls{ctx.modes=="full_dynamic"}.pose;
        poses=[ctx.reference;oldFit;newFit];labels=["reference","legacy_fit","modern_fit"];
        for j=1:3
            pose=[poses(j,1:2)-ctx.seed(1:2),poses(j,3)];a=legacy.linearize(pose,[1;1;.1]);b=modern.linearize(pose,[1;1;.1]);
            rows(end+1,:)={ctx.frame,labels(j),a.cost,b.cost,a.numPairs,b.numPairs,norm(poses(j,1:2)-ctx.reference(1:2))}; %#ok<AGROW>
        end
    end
    report=cell2table(rows,VariableNames={'frame','pose','legacyCost','modernCost','legacyPairs','modernPairs','errorM'});
end
