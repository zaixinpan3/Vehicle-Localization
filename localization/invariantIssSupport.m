classdef invariantIssSupport
% Exact vector increments and a conditional finite-error ISS certificate.
% Physical state order: [X,VX,AX,Y,VY,AY,psi,r]. Angles use one unwrapped chart.
% Vector-increment methodology: Sharma et al. (2026), doi:10.1002/rnc.70732.
% The finite identities below accommodate this observer's estimated-state
% Jacobian gain; a constant-gain theorem is not applied to it verbatim.
    methods (Static)
        function [F,H,Hspeed,Hgnss]=secants(x,xhat,beta,betaRate,cfg)
        % f(x)-f(xhat)=F*(x-xhat), likewise for all nonlinear outputs.
        % These identities are finite differences, including at equal states.
            arguments
                x (8,1) double {mustBeReal,mustBeFinite}
                xhat (8,1) double {mustBeReal,mustBeFinite}
                beta (1,1) double {mustBeReal,mustBeFinite}
                betaRate (1,1) double {mustBeReal,mustBeFinite}
                cfg (1,1) struct
            end
            e=x-xhat;m=(x+xhat)/2;v=m([2,5]);a=m([3,6]);
            J=[0,-1;1,0];q=m(8)+betaRate;s=cfg.iss.maximumSpeed;
            ell=cfg.invariantObserver.headingLengthM;half=e(7)/2;
            cardinal=1;if half~=0,cardinal=sin(half)/half;end
            c=cos(half);d=[cos(m(7));sin(m(7))];n=J*d;
            dc=[cos(m(7)+beta);sin(m(7)+beta)];nc=J*dc;
            F=zeros(8);F(1,2)=1;F(2,3)=1;F(4,5)=1;F(5,6)=1;F(7,8)=1;
            F([3,6],[2,5])=(q^2+e(8)^2/4)*eye(2);
            F([3,6],[3,6])=2*q*J;F([3,6],8)=2*q*v+2*J*a;
            H=zeros(8);H(1,[2,5])=v.'/s;
            H(2,[2,5])=a.'/s;H(2,[3,6])=v.'/s;
            H(3,[2,5])=(-J*a).'/s;H(3,[3,6])=(J*v).'/s;
            H(4,8)=ell;H(5,:)=H(3,:);H(5,[2,5])=H(5,[2,5])-2*q*v.'/s;
            H(5,8)=-(v.'*v+norm(e([2,5]))^2/4)/s;
            H(6,[2,5])=c*nc.';H(6,7)=-cardinal*dc.'*v;
            H(7,[3,6])=c*d.';H(7,7)=cardinal*n.'*a;
            H(8,[3,6])=c*n.';H(8,7)=-cardinal*d.'*a;
            Hspeed=zeros(1,8);Hspeed([2,5])=c*dc.';Hspeed(7)=cardinal*nc.'*v;
            Hgnss=zeros(2,8);Hgnss(:,[1,4])=eye(2);
            Hgnss(:,7)=cardinal*[d,n]*J*cfg.gnss.outputPoint.bodyOffset(:);
        end

        function N=measurementSecant(v,a,vm,am,cfg)
        % Packet increment from ONE primitive [dv(2); da(2); dgyro] vector.
        % The zero-valued kinematic outputs have no independent sensor noise.
            arguments
                v (2,1) double {mustBeReal,mustBeFinite}
                a (2,1) double {mustBeReal,mustBeFinite}
                vm (2,1) double {mustBeReal,mustBeFinite}
                am (2,1) double {mustBeReal,mustBeFinite}
                cfg (1,1) struct
            end
            v=(v+vm)/2;a=(a+am)/2;s=cfg.iss.maximumSpeed;J=[0,-1;1,0];
            N=zeros(8,5);N(1,1:2)=v.'/s;
            N(2,1:2)=a.'/s;N(2,3:4)=v.'/s;
            N(3,1:2)=(-J*a).'/s;N(3,3:4)=(J*v).'/s;
            N(4,5)=cfg.invariantObserver.headingLengthM;N(7:8,3:4)=eye(2);
        end

        function c=regionalCertificate(design,cfg,joint)
        % Vdot <= -2*alpha*V + gamma*||d||^2 for additive normalized input d.
        % Uniform over the common L/G/M families, subject to their contracts.
        % No actual-map, sampled-system or upstream LPV-domain assertion.
        % With B=diag(theta.^degrees./[1,...,1,ell,ell]), eps=B\(x-xhat),
        % true |v|<=V, |a|<=A, |r+betaRate|<=Q, and v=s*d(psi+beta), s>=0:
        % He(P*A0)<=-mu*I. The exact nonlinear remainder obeys
        % 2*eps'*P*remainder <= c1*|eps|^3+c2*|eps|^4 on |eps|<=outer.
        % For each normalized invariant row, |Htrue|<=h and |Hessian|<=L;
        % |Hhat-Htrue|<=L*|eps| and |Hsec-Htrue|<=L*|eps|/2.
        % Hence the matched-output remainder is bounded by
        % theta*k*(3*h*L*|eps|^3+L^2*|eps|^4), without changing Hhat to Htrue.
        % The actual finite LiDAR operator obeys sym(W)>=S-delta*I on
        % the WHOLE error tube; a nominal PSD Hessian alone is insufficient.
        % In M, S is PSD with the configured heading diagonal and coupling
        % bounds. Its cross block remains in the joint GNSS/LiDAR LMIs;
        % S>=diag(0,0,hmin) is neither inferred nor required.
        % A common P allows switching between valid families without jumps.
            arguments
                design (1,1) struct
                cfg (1,1) struct
                joint (1,1) struct
            end
            P=(design.P+design.P.')/2;z=cfg.invariantObserver;t=z.theta;ell=z.headingLengthM;
            V=cfg.iss.maximumSpeed;A=cfg.iss.maximumAcceleration;Q=cfg.maximumTrackAngleRate;
            outer=min(z.regionalRadiusCap,pi*ell/(2*t));
            validateattributes(outer,{'double'},{'real','finite','positive','scalar'});
            hp=t/ell;hr=t^2/ell;vt=V+t^2*outer;at=A+t^3*outer;qt=Q+hr*outer;
            % Bounds on normalized Jacobian rows at the true state, and on
            % their Hessians throughout the declared outer error ball.
            h=[1;hypot(A/(V*t),1);hypot(A/(V*t),1);1; ...
                norm([(A+2*Q*V)/(V*t),1,V/(t*ell)]); ...
                hypot(1,V/(t*ell));hypot(1,A/(t^2*ell));hypot(1,A/(t^2*ell))];
            L=[t^2/V;t^2/V;t^2/V;0; ...
                t^2/V+2*t*qt/V+2*t*vt/(ell*V); ...
                hp+vt/ell^2;hp+at/(t*ell^2);hp+at/(t*ell^2)];
            k=design.auxiliaryWeights([1,2,2,3,4,5,2,2]);k=k(:);k(4)=0;
            active=[z.velocityAvailable;z.velocityAvailable&&z.accelerationAvailable; ...
                z.velocityAvailable&&z.accelerationAvailable;false;z.velocityAvailable; ...
                z.velocityAvailable;z.accelerationAvailable;z.accelerationAvailable];
            k=k.*active;E=eye(8);pa=norm(P*E(:,[3,6]),2);pv=norm(P*E(:,[2,5]),2);
            c1=2*pa*(t*V/ell^2+2*Q*t/ell+2*t^2/ell) ...
                +design.kappaG*norm(cfg.gnss.outputPoint.bodyOffset)*hp^2 ...
                +3*t*sum(k.*h.*L);
            c2=2*pa*t^3/ell^2+t*sum(k.*L.^2);
            if z.velocityAvailable
                c1=c1+2*t*design.speedGainNormalized*pv*(hp+(hp+vt/ell^2)/2);
            end
            worst=max([joint.worstLidarEigenvalue,joint.worstGnssEigenvalue,joint.worstMixedEigenvalue]);
            mu=t*(design.lambda-worst);ev=eig(P);pmin=min(ev);pmax=max(ev);
            valid=joint.certified && pmin>0 && mu>0 && all(isfinite([c1,c2,mu,pmin,pmax]));
            radius=0;alpha=0;gamma=Inf;level=0;inputLimit=0;
            if valid
                radius=min(outer,mu/(c1+sqrt(c1^2+2*c2*mu)));
                alpha=mu/(8*pmax);gamma=4*pmax^2/mu;level=pmin*radius^2;
                inputLimit=sqrt(2*alpha*level/gamma);
            end
            c=struct('certified',valid && radius>0,'normalizedRadius',radius, ...
                'outerRadius',outer,'lyapunovLevel',level,'alpha',alpha,'gamma',gamma, ...
                'normalizedAdditiveInputLimit',inputLimit,'nominalDissipation',mu, ...
                'cubicRemainderBound',c1,'quarticRemainderBound',c2, ...
                'minimumMetricEigenvalue',pmin,'maximumMetricEigenvalue',pmax, ...
                'scope',"Conditional continuous eight-state ISS: V(0)<level and ||d||_infinity<inputLimit; true motion, gyro availability and a common L/G/M finite-overlap contract must hold throughout. Input d includes upstream interface error, sensor error and model residual. This does not certify actual maps, the full lateral cascade or sampling.");
        end
    end
end
