function [translationWeight,headingWeight,diagnostics,poseWeight] = computeLidarInformationWeights(informationMatrix,cfg)
% computeLidarInformationWeights Turn full information into directional gain.
% In normalized pose coordinates J=D*I*D, W=J/(scale*I+J). Eigenvectors and
% cross terms are retained, weak directions receive smaller gains, and zero
% information gives zero gain in that direction. Rank deficiency is usable.
% This shaping is not an assertion that D2D curvature is calibrated precision.
    arguments
        informationMatrix double
        cfg (1,1) struct
    end
    poseWeight=zeros(3);translationWeight=zeros(2);headingWeight=0;
    diagnostics=struct('hadInformation',false,'qualified',false,'reason',"missingInformation", ...
        'minimumNormalizedEigenvalue',0,'informationMargin',NaN,'headingInformation',0, ...
        'translationInformationEigenvalues',zeros(2,1),'normalizedInformation',zeros(3), ...
        'normalizedWeight',zeros(3),'weightEigenvalues',zeros(3,1),'rank',0, ...
        'gpsFusedLidarWeight',zeros(3),'gpsFusedGpsWeight',zeros(3), ...
        'gpsFusedNormalizedWeight',zeros(3));
    if isempty(informationMatrix),return;end
    assert(isequal(size(informationMatrix),[3,3]),'Information must be 3-by-3.');
    if any(~isfinite(informationMatrix),'all'),return;end
    diagnostics.hadInformation=true;
    D=diag(cfg.lidar.poseScales(:));
    assert(all(isfinite(diag(D)) & diag(D)>0) && size(D,1)==3);
    J=D*((informationMatrix+informationMatrix.')/2)*D;
    [U,S]=eig(J);values=diag(S);tol=1e-10*max(1,max(abs(values)));
    if min(values)<-tol
        diagnostics.reason="invalidIndefiniteInformation";return;
    end
    values=max(values,0);J=U*diag(values)*U.';J=(J+J.')/2;
    diagnostics.normalizedInformation=J;
    diagnostics.minimumNormalizedEigenvalue=min(values);
    diagnostics.rank=nnz(values>tol);
    diagnostics.qualified=any(values>tol);
    if ~diagnostics.qualified,diagnostics.reason="zeroInformation";return;end
    [poseWeight,~,W]=fusePoseInformationWeights(J,false,cfg);
    [gpsL,gpsG,gpsW]=fusePoseInformationWeights(J,true,cfg);
    diagnostics.reason="directionalInformation";
    diagnostics.normalizedWeight=W;diagnostics.weightEigenvalues=sort(eig(W));
    diagnostics.gpsFusedLidarWeight=gpsL;diagnostics.gpsFusedGpsWeight=gpsG;
    diagnostics.gpsFusedNormalizedWeight=gpsW;
    diagnostics.headingInformation=informationMatrix(3,3);
    diagnostics.translationInformationEigenvalues=sort(eig(informationMatrix(1:2,1:2)),'descend');
    translationWeight=poseWeight(1:2,1:2);headingWeight=poseWeight(3,3);
end
