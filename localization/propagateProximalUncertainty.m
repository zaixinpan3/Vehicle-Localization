function [prior,F]=propagateProximalUncertainty(previous,dt,A,R,bodyVelocity,bodyAcceleration,K,gnssInformation,gains,cfg,gnssYawDerivative)
% propagateProximalUncertainty Linearize the actual implicit motion step.
% Bias states are body-frame true-minus-measured velocity offsets. Their
% random-walk uncertainty persists across pose outages; no pose is invented.
% Process standard deviations are continuous white-noise design intensities.
    if nargin<11,gnssYawDerivative=zeros(2,1);end
    n=9;F=eye(n);J=[0,-1;1,0];indices=[2,5,3,6];Dva=zeros(4,n);
    Dva(:,indices)=A\eye(4);
    Dva(:,7)=A\(dt*[gains(2)*J*R*bodyVelocity;gains(3)*J*R*bodyAcceleration]);
    if cfg.estimateMotionBias,Dva(:,8:9)=A\(dt*[gains(2)*R;zeros(2)]);end
    F(indices,:)=Dva;Dp=zeros(2,n);Dp(:,[1,4])=eye(2);
    if cfg.positionIntegration=="trapezoidal"
        Dp=Dp+dt/2*Dva(1:2,:);Dp(:,[2,5])=Dp(:,[2,5])+dt/2*eye(2);
    else
        assert(cfg.positionIntegration=="endpoint");Dp=Dp+dt*Dva(1:2,:);
    end
    T=(eye(2)+dt*K)\eye(2);F([1,4],:)=T*Dp;F([1,4],7)=F([1,4],7)+gnssYawDerivative;
    q=cfg.processStd(:);validateattributes(q,{'double'},{'nonnegative','finite','numel',n});
    Q=dt*diag(q.^2);
    if any(K,'all')
        B=T*(dt*K);Q([1,4],[1,4])=Q([1,4],[1,4])+B*(gnssInformation\eye(2))*B.';
    end
    prior=F*previous*F.'+Q;prior=(prior+prior.')/2;
end
