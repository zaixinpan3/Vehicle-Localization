function measurement=evaluateLidarRegistrationResidual(frozen,pose)
% evaluateLidarRegistrationResidual Reevaluate admitted scan geometry at prediction.
% FROZEN is the serializable lidarResidualModel exported by registration.
% Associations, robust influence, latent support covariance and admitted pose
% subspace stay fixed; residual and physical Jacobian are reevaluated together.
% A directional model evaluates anchor+projector*(pose-anchor), preserving its
% rejected directions in both innovation and information. Gaussian source
% scatter still rotates, using the same analytic residuals as registration.
% Between-hypothesis pose uncertainty is marginalized by a low-rank residual
% whitening. It only reduces information and applies to BOTH r and J. The
% resulting surrogate is conditional on associations and any position aiding.
    arguments
        frozen (1,1) struct
        pose (3,1) double {mustBeReal,mustBeFinite}
    end
    assert(isfield(frozen,'kind') && string(frozen.kind)=="frozen-semantic-lidar-v1", ...
        'VehicleLocalization:InvalidLidarResidualModel','Unknown frozen registration model.');
    anchor=frozen.anchorPose(:);projector=frozen.poseProjector;
    difference=pose-anchor;difference(3)=atan2(sin(difference(3)),cos(difference(3)));
    effective=anchor+projector*difference;
    local=[effective(1:2).'-frozen.origin(:).',effective(3)];
    method=string(frozen.method);
    switch method
        case "supportD2D"
            [r,J]=supportRegistrationResiduals(frozen.sourceMean,frozen.sourceCovariance, ...
                frozen.targetMean,frozen.targetCovariance,frozen.sourceAxis, ...
                frozen.targetNormal,frozen.directionScale,local,frozen.noiseStandardDeviation);
        case "anisotropicD2D"
            [r,J]=gaussianRegistrationResiduals(frozen.sourceMean,frozen.sourceCovariance, ...
                frozen.targetMean,frozen.targetCovariance,local,frozen.noiseStandardDeviation);
        case "geometricD2D"
            [r,J]=lineAndPointResiduals(frozen,local);
        otherwise
            error('VehicleLocalization:InvalidLidarResidualModel','Unsupported frozen registration method.');
    end
    J=reshape(permute(J,[1,3,2]),[],3)*projector;
    influence=repelem(frozen.rowInfluence(:),size(r,1),1);
    assert(numel(influence)==numel(r) && isreal(influence) && all(isfinite(influence) & influence>=0), ...
        'VehicleLocalization:InvalidLidarResidualModel','Frozen influence must be finite and nonnegative.');
    root=sqrt(influence);r=root.*r(:);J=root.*J;
    spread=frozen.poseSpread;
    assert(isequal(size(spread),[3,3]) && isreal(spread) && all(isfinite(spread),'all') && ...
        norm(spread-spread.','fro')<=1e-10*max(1,norm(spread,'fro')), ...
        'VehicleLocalization:InvalidLidarResidualModel','Pose uncertainty must be symmetric PSD.');
    [V,E]=eig((spread+spread.')/2);values=diag(E);
    assert(min(values)>=-1e-10*max(1,norm(spread,2)), ...
        'VehicleLocalization:InvalidLidarResidualModel','Pose uncertainty must be PSD.');
    if any(values>0)
        root=V*diag(sqrt(max(values,0)))*V.';
        [U,S,~]=svd(J*root,'econ');attenuation=1-1./sqrt(1+diag(S).^2);
        r=r-U*(attenuation.*(U.'*r));J=J-U*(attenuation.*(U.'*J));
    end
    measurement=struct('residual',r,'jacobian',J,'weights',ones(size(r)), ...
        'linearizationPose',pose,'conditionedOnPositionAid',frozen.conditionedOnPositionAid, ...
        'informationCalibrated',false,'geometrySource',"Frozen accepted semantic registration");
end

function [r,J]=lineAndPointResiduals(f,pose)
    R=[cos(pose(3)),-sin(pose(3));sin(pose(3)),cos(pose(3))];
    dR=R*[0,-1;1,0];n=size(f.sourceMean,1);
    delta=f.sourceMean*R.'+pose(1:2)-f.targetMean;
    derivative=f.sourceMean*dR.';
    r=zeros(3,n);J=zeros(3,3,n);B=f.precisionRoot;
    r(1:2,:)=reshape(pagemtimes(B,reshape(delta.',2,1,n)),2,n);
    J(1:2,1:2,:)=B;
    J(1:2,3,:)=pagemtimes(B,reshape(derivative.',2,1,n));
    r(3,:)=(sum((f.sourceAxis*R.').*f.targetNormal,2).*f.directionScale).';
    J(3,3,:)=sum((f.sourceAxis*dR.').*f.targetNormal,2).*f.directionScale;
end
