function [fixed, moving] = balanceSemanticDistributions(fixed, moving)
% balanceSemanticDistributions: Equalize shared class L2 energies for D2D.
% Each shared semantic field has unit self energy, then receives 1/C of the
% objective. The resulting overlap is the mean class-conditional normalized
% overlap. This prevents long curb support from overwhelming sparse landmarks.
% These internal weights are Hilbert-space scaling, not probability masses.
    names=intersect(unique(fixed.semanticName(fixed.mixtureWeight>0)), ...
        unique(moving.semanticName(moving.mixtureWeight>0)));
    fixedWeights=zeros(size(fixed.mixtureWeight));
    movingWeights=zeros(size(moving.mixtureWeight));
    for name=names.'
        f=fixed; m=moving;
        f.mixtureWeight(f.semanticName~=name)=0;
        m.mixtureWeight(m.semanticName~=name)=0;
        fe=semanticGaussianOverlap(f,f,[0 0 0]);
        me=semanticGaussianOverlap(m,m,[0 0 0]);
        fixedWeights=fixedWeights+f.mixtureWeight/sqrt(max(fe*numel(names),realmin));
        movingWeights=movingWeights+m.mixtureWeight/sqrt(max(me*numel(names),realmin));
    end
    fixed.mixtureWeight=fixedWeights;
    moving.mixtureWeight=movingWeights;
end
