function filter = filterLidarPoseInformation(information,cfg,frameReliability,directionReliability,poseErrorVariance)
% filterLidarPoseInformation Bounded weighting of supported LiDAR directions.
% INFORMATION is an unregularized physical [X,Y,psi] information surrogate.
% Direction reliabilities refer to ascending normalized information eigenvalues.
% Repeated eigenspaces receive their minimum requested reliability, avoiding
% dependence on the arbitrary eigenvector basis. No count or eigenvalue floor
% is added to geometric information. PSD rank loss and complete rejection are
% valid. Reliability must come from association/geometry checks upstream.
% A calibrated channel replaces the geometric regularizer with predicted
% pose-error second moments [horizontal per axis; heading], in m^2/rad^2.
% Its F need not be symmetric: F*H=S is the required affine identity.
    arguments
        information (3,3) double {mustBeReal,mustBeFinite}
        cfg (1,1) struct
        frameReliability (1,1) double {mustBeReal,mustBeFinite} = 1
        directionReliability (3,1) double {mustBeReal,mustBeFinite} = ones(3,1)
        poseErrorVariance double = []
    end
    scales=cfg.lidar.poseScales(:);calibrated=isfield(cfg.lidar,'errorCalibration');
    validateattributes(scales,{'double'},{'real','finite','positive','numel',3});
    if calibrated
        assert(~isfield(cfg.lidar,'gainInformationScale'),'VehicleLocalization:AmbiguousLidarCalibration', ...
            'A calibrated channel has no geometric information-scale parameter.');
        assert(isequal(scales,ones(3,1)),'VehicleLocalization:UnsupportedFullIssScaling', ...
            'The calibrated channel uses physical metres and radians.');
        reference=cfg.lidar.errorCalibration.referenceVariance(:);
        validateattributes(reference,{'double'},{'real','finite','positive','numel',2});
        predictLidarPoseErrorVariance(zeros(2,1),cfg.lidar.errorCalibration);
        lambda=NaN;
    else
        lambda=cfg.lidar.gainInformationScale;
        validateattributes(lambda,{'numeric'},{'real','finite','positive','scalar'});
    end
    assert(frameReliability>=0 && frameReliability<=1 && ...
        all(directionReliability>=0 & directionReliability<=1), ...
        'VehicleLocalization:InvalidLidarReliability','Reliabilities must lie in [0,1].');
    assert(norm(information-information.','fro')<=1e-10*max(1,norm(information,'fro')), ...
        'VehicleLocalization:InvalidLidarInformation','Information must be symmetric.');
    D=diag(scales);I=D*((information+information.')/2)*D;
    assert(all(isfinite(I),'all'),'VehicleLocalization:InvalidLidarInformation', ...
        'Normalized information must be finite.');
    [U,E]=eig(I);[values,order]=sort(diag(E));U=U(:,order);
    tolerance=1e-10*max(1,max(abs(values)));
    assert(min(values)>=-tolerance,'VehicleLocalization:InvalidLidarInformation', ...
        'Information must be positive semidefinite.');
    values=max(values,0); % Remove negative roundoff only; do not fill nullspaces.
    if calibrated,tolerance=3*eps(max(values));end % Numerical rank, independent of loss units.
    first=1;
    while first<=3
        last=first;
        while last<3 && values(last+1)-values(first)<=tolerance,last=last+1;end
        directionReliability(first:last)=min(directionReliability(first:last));
        first=last+1;
    end
    rho=frameReliability*directionReliability;
    if calibrated
        active=values>tolerance;
        if any(active)
            validateattributes(poseErrorVariance,{'double'},{'real','finite','positive','numel',2});
            authority=min(1,reference./poseErrorVariance(:));
            % Covariance is expressed per position axis and in yaw radians.
            % The empirical mean second moment is the unit of reliability.
            % Project both sides so no correction enters an unobserved mode.
            V=U(:,active);B=V*diag(sqrt(rho(active)))*V.';
            S=B*diag(authority([1,1,2]))*B;
            inverse=V*diag(1./values(active))*V.';F=S*inverse;
        else
            S=zeros(3);F=zeros(3);
        end
        s=eig((S+S.')/2);
        interpretation="Empirically calibrated conditional pose-error second moment; geometry supplies the admitted subspace";
    else
        f=rho./(values+lambda);s=rho.*(values./(values+lambda));
        F=U*diag(f)*U.';S=U*diag(s)*U.';
        F=(F+F.')/2;
        interpretation="Weighted geometric surrogate; not calibrated covariance";
    end
    informationTrace=sum(values);dimension=0;condition=Inf;
    if values(end)>0
        relative=values/values(end);
        dimension=sum(relative)^2/sum(relative.^2);
        if values(1)>0,condition=values(end)/values(1);end
    end
    filter=struct('F',F,'S',(S+S.')/2,'D',D, ...
        'normalizedInformation',U*diag(values)*U.', ...
        'eigenvectors',U,'eigenvalues',values,'reliability',rho, ...
        'strengths',s,'rank',nnz(values>tolerance), ...
        'trace',informationTrace,'minimumEigenvalue',values(1), ...
        'conditionNumber',condition,'effectiveDimension',dimension, ...
        'regularizer',lambda,'informationInterpretation',interpretation, ...
        'poseErrorVariance',poseErrorVariance,'calibrated',calibrated);
end
