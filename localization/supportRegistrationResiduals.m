function [residual,J,precision]=supportRegistrationResiduals(sourceMean,sourceCov,targetMean,targetCov,sourceAxis,targetNormal,directionScale,pose,noise)
% supportRegistrationResiduals Partial-support position and shape orientation.
% The latent sliding covariance is frozen in the map frame; source scatter
% rotates analytically. Angular information vanishes for round distributions.
    [residual,J,precision]=gaussianRegistrationResiduals(sourceMean,sourceCov,targetMean,targetCov,pose,noise);
    residual=residual(1:3,:);J=J(1:3,:,:);
    r=[cos(pose(3)) -sin(pose(3));sin(pose(3)) cos(pose(3))];
    residual(3,:)=(sum((sourceAxis*r.').*targetNormal,2).*directionScale).';
    J(3,:,:)=0;
    J(3,3,:)=sum((sourceAxis*[r(:,2),-r(:,1)].').*targetNormal,2).*directionScale;
end
