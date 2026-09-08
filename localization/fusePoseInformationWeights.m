function [lidarWeight,gpsWeight,totalNormalizedWeight] = fusePoseInformationWeights(J,useGps,cfg)
% fusePoseInformationWeights Combine information without discarding cross terms.
% The correction is L_L*r_L+L_G*r_G. Its homogeneous normalized weight is
% symmetric in [0,I]. GPS has XY information only; its yaw residual is zero.
% cfg.gps.positionInformation is a design reference, not a calibrated claim.
    arguments
        J (3,3) double
        useGps (1,1) logical
        cfg (1,1) struct
    end
    scale=cfg.lidar.gainInformationScale;
    assert(isscalar(scale) && isfinite(scale) && scale>0);
    D=diag(cfg.lidar.poseScales(:));
    G=double(useGps)*D*diag([cfg.gps.positionInformation(:);0])*D;
    assert(isequal(size(G),[3,3]) && all(isfinite(G),'all') && min(eig(G))>=0);
    system=scale*eye(3)+J+G;
    lidarWeight=D*(system\J)/D;
    gpsWeight=D*(system\G)/D;
    totalNormalizedWeight=system\(J+G);
    totalNormalizedWeight=(totalNormalizedWeight+totalNormalizedWeight.')/2;
end
