function [translationWeight,headingWeight,diagnostics,poseWeight] = computeLidarInformationWeights(informationMatrix,cfg)
% computeLidarInformationWeights Normalize full pose information and shape gain.
% W=J/(lambda*I+J), J=S'*information*S. All returned weights use normalized
% pose coordinates; the observer multiplies them by a normalized residual.
% No eigenvalue floor or source fusion is applied. The continuous LiDAR mode
% requires the configured uniformly positive weight sector.
    arguments
        informationMatrix double
        cfg (1,1) struct
    end
    poseWeight=zeros(3);translationWeight=zeros(2);headingWeight=0;
    diagnostics=struct('qualified',false,'reason',"missingInformation", ...
        'rank',0,'minimumNormalizedEigenvalue',0,'normalizedWeight',zeros(3), ...
        'normalizedInformation',zeros(3),'weightEigenvalues',zeros(3,1));
    if isempty(informationMatrix),return;end
    if ~isreal(informationMatrix) || ~isequal(size(informationMatrix),[3,3]) ...
            || any(~isfinite(informationMatrix),'all')
        diagnostics.reason="invalidInformation";return;
    end
    tolerance=1e-10*max(1,norm(informationMatrix,2));
    if norm(informationMatrix-informationMatrix.','fro')>tolerance
        diagnostics.reason="asymmetricInformation";return;
    end
    S=diag(cfg.lidar.poseScales(:));J=S*((informationMatrix+informationMatrix.')/2)*S;
    [U,D]=eig(J);values=diag(D);
    if min(values)<0
        diagnostics.reason="nonpositiveInformation";return;
    end
    weights=values./(values+cfg.lidar.gainInformationScale);
    poseWeight=U*diag(weights)*U.';poseWeight=(poseWeight+poseWeight.')/2;
    diagnostics.rank=nnz(values>1e-12*max(1,max(values)));
    diagnostics.minimumNormalizedEigenvalue=min(values);
    diagnostics.normalizedInformation=J;diagnostics.normalizedWeight=poseWeight;
    diagnostics.weightEigenvalues=sort(weights);
    diagnostics.qualified=min(weights)>=cfg.lidar.minimumPoseWeight-1e-12;
    diagnostics.reason="insufficientInformation";
    if diagnostics.qualified,diagnostics.reason="uniformlyInformative";end
    translationWeight=poseWeight(1:2,1:2);headingWeight=poseWeight(3,3);
end
