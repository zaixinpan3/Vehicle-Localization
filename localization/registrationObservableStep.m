function [step,projector,rank,eigenvalues]=registrationObservableStep(h,gradient,ratio,partitionTranslation)
% registrationObservableStep Remove unsupported spatial directions before yaw.
% For finite elongated clouds, tiny center forces can tilt a full-matrix null
% eigenvector into yaw. Spatial projection keeps a rejected road tangent from
% leaking into the accepted pose event through those cross terms.
    h=(h+h.')/2;
    if partitionTranslation
        [axes,values]=eig(h(1:2,1:2),'vector');
        keep=values>max(1e-8,ratio*max(values));
        translation=axes(:,keep)*axes(:,keep).';
        subspace=blkdiag(translation,1);h=subspace*h*subspace;
    end
    [v,eigenvalues]=eig((h+h.')/2,'vector');
    keep=eigenvalues>max(1e-8,ratio*max(eigenvalues));rank=nnz(keep);
    projector=v(:,keep)*v(:,keep).';
    step=-v(:,keep)*((v(:,keep).'*gradient)./eigenvalues(keep));
end
