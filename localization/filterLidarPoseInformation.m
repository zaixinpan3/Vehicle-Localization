function filter = filterLidarPoseInformation(information,cfg,frameReliability,directionReliability)
% filterLidarPoseInformation Bounded Route A filter in normalized pose units.
% INFORMATION is an unregularized physical [X,Y,psi] information surrogate.
% Direction reliabilities refer to ascending normalized information eigenvalues.
% Repeated eigenspaces receive their minimum requested reliability, avoiding
% dependence on the arbitrary eigenvector basis. No count or eigenvalue floor
% is added to geometric information. PSD rank loss and complete rejection are
% valid. Reliability must come from association/geometry checks upstream.
    arguments
        information (3,3) double {mustBeReal,mustBeFinite}
        cfg (1,1) struct
        frameReliability (1,1) double {mustBeReal,mustBeFinite} = 1
        directionReliability (3,1) double {mustBeReal,mustBeFinite} = ones(3,1)
    end
    scales=cfg.lidar.poseScales(:);lambda=cfg.lidar.gainInformationScale;
    validateattributes(scales,{'double'},{'real','finite','positive','numel',3});
    validateattributes(lambda,{'numeric'},{'real','finite','positive','scalar'});
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
    first=1;
    while first<=3
        last=first;
        while last<3 && values(last+1)-values(first)<=tolerance,last=last+1;end
        directionReliability(first:last)=min(directionReliability(first:last));
        first=last+1;
    end
    rho=frameReliability*directionReliability;
    f=rho./(values+lambda);s=rho.*(values./(values+lambda));
    F=U*diag(f)*U.';S=U*diag(s)*U.';
    informationTrace=sum(values);dimension=0;condition=Inf;
    if values(end)>0
        relative=values/values(end);
        dimension=sum(relative)^2/sum(relative.^2);
        if values(1)>0,condition=values(end)/values(1);end
    end
    filter=struct('F',(F+F.')/2,'S',(S+S.')/2,'D',D, ...
        'normalizedInformation',U*diag(values)*U.', ...
        'eigenvectors',U,'eigenvalues',values,'reliability',rho, ...
        'strengths',s,'rank',nnz(values>tolerance), ...
        'trace',informationTrace,'minimumEigenvalue',values(1), ...
        'conditionNumber',condition,'effectiveDimension',dimension, ...
        'regularizer',lambda,'informationInterpretation',"Weighted geometric surrogate; not calibrated covariance");
end
