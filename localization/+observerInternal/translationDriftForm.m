function H=translationDriftForm(P,x)
% Hermitian part of the complex translation drift at signed scaled yaw rate x.
% P is the same real symmetric 3-by-3 matrix in both Cartesian chains.
    A=[0,1,0;0,0,1;0,x^2,2i*x];
    product=P*A;
    H=product+product';
end
