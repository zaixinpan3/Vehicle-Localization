classdef invariantIssSupport
% Exact vector increments and a conditional finite-error ISS certificate.
% Physical state order: [X,VX,AX,Y,VY,AY,psi,r]. Angles use one unwrapped chart.
% Vector-increment methodology: Sharma et al. (2026), doi:10.1002/rnc.70732.
% The finite identities below accommodate this observer's estimated-state
% Jacobian gain; a constant-gain theorem is not applied to it verbatim.
    methods (Static)
        function [F,H,Hgnss]=secants(x,xhat,beta,betaRate,cfg)
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
            J=[0,-1;1,0];q=m(8)+betaRate;half=e(7)/2;
            cardinal=1;if half~=0,cardinal=sin(half)/half;end
            c=cos(half);d=[cos(m(7));sin(m(7))];n=J*d;
            dc=[cos(m(7)+beta);sin(m(7)+beta)];nc=J*dc;
            F=zeros(8);F(1,2)=1;F(2,3)=1;F(4,5)=1;F(5,6)=1;F(7,8)=1;
            F([3,6],[2,5])=(q^2+e(8)^2/4)*eye(2);
            F([3,6],[3,6])=2*q*J;F([3,6],8)=2*q*v+2*J*a;
            H=zeros(5,8);H(1,[2,5])=2*v.';
            H(2,[2,5])=a.';H(2,[3,6])=v.';
            H(3,[2,5])=(-J*a).';H(3,[3,6])=(J*v).';
            H(4,[2,5])=c*nc.';H(4,7)=-cardinal*dc.'*v;
            H(5,8)=1;
            Hgnss=zeros(2,8);Hgnss(:,[1,4])=eye(2);
            Hgnss(:,7)=cardinal*[d,n]*J*cfg.gnss.outputPoint.bodyOffset(:);
        end

        function N=measurementSecant(v,a,vm,am)
        % Packet increment from ONE primitive [dv(2); da(2); dgyro] vector.
        % The zero-valued kinematic outputs have no independent sensor noise.
            arguments
                v (2,1) double {mustBeReal,mustBeFinite}
                a (2,1) double {mustBeReal,mustBeFinite}
                vm (2,1) double {mustBeReal,mustBeFinite}
                am (2,1) double {mustBeReal,mustBeFinite}
            end
            v=(v+vm)/2;a=(a+am)/2;J=[0,-1;1,0];
            N=zeros(5,5);N(1,1:2)=2*v.';
            N(2,1:2)=a.';N(2,3:4)=v.';
            N(3,1:2)=(-J*a).';N(3,3:4)=(J*v).';
            N(5,5)=1;
        end

        function c=regionalCertificate(design,cfg,joint)
        % Vdot <= -2*alpha*V + gamma*||d||^2 for additive normalized input d.
        % Uniform over the common L/G/M families, subject to their contracts.
        % No actual-map, sampled-system or upstream LPV-domain assertion.
        % With B=diag(theta.^degrees./[1,...,1,ell,ell]), eps=B\(x-xhat),
        % true 0<s_min<=|v|<=V, |a|<=A, |r+betaRate|<=Q, and v=s*d(psi+beta):
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
        % All five auxiliary outputs remain available in every family.
        % Reserving mu/2 for remainders and mu/4 for the additive input gives
        % alpha=mu/(8*pmax). This rate is computed after synthesis, not imposed.
        % A common P permits valid-family switching.
            arguments
                design (1,1) struct
                cfg (1,1) struct
                joint (1,1) struct
            end
            P=(design.P+design.P.')/2;z=cfg.invariantObserver;t=z.theta;ell=z.headingLengthM;
            V=cfg.iss.maximumSpeed;A=cfg.iss.maximumAcceleration;Q=cfg.maximumTrackAngleRate;
            outer=min(z.regionalRadiusCap,pi*ell/(2*t));
            validateattributes(outer,{'double'},{'real','finite','positive','scalar'});
            hp=t/ell;vt=V+t^2*outer;
            % Bounds on normalized Jacobian rows at the true state, and on
            % their Hessians throughout the declared outer error ball.
            h=[1;hypot(A/(V*t),1);hypot(A/(V*t),1);hypot(1,V/(t*ell));1];
            L=[t^2/V;t^2/V;t^2/V;hp+vt/ell^2;0];
            % The scheduler is bounded for every nonnegative measured speed.
            % Its nominal extra Grams only dissipate; bound the estimated-
            % Jacobian/secant remainder using its largest possible weights.
            maximum=motionResponseSupport.maximumWeights(design);
            k=[maximum([1,2,2,3]);0];
            active=[z.velocityAvailable;z.velocityAvailable&&z.accelerationAvailable; ...
                z.velocityAvailable&&z.accelerationAvailable;z.velocityAvailable;false];
            k=k.*active;E=eye(8);pa=norm(P*E(:,[3,6]),2);
            c1=2*pa*(t*V/ell^2+2*Q*t/ell+2*t^2/ell) ...
                +design.kappaG*norm(cfg.gnss.outputPoint.bodyOffset)*hp^2 ...
                +3*t*sum(k.*h.*L);
            c2=2*pa*t^3/ell^2+t*sum(k.*L.^2);
            worst=max([joint.worstLidarEigenvalue,joint.worstGnssEigenvalue,joint.worstMixedEigenvalue]);
            mu=t*(design.lambda-worst);ev=eig(P);pmin=min(ev);pmax=max(ev);
            valid=joint.certified && z.velocityAvailable && z.accelerationAvailable ...
                && pmin>0 && mu>0 && all(isfinite([c1,c2,mu,pmin,pmax]));
            radius=0;alpha=0;gamma=Inf;level=0;inputLimit=0;
            if valid
                radius=min(outer,mu/(c1+sqrt(c1^2+2*c2*mu)));
                alpha=mu/(8*pmax);
                gamma=4*pmax^2/mu;level=pmin*radius^2;
                inputLimit=sqrt(2*alpha*level/gamma);
            end
            c=struct('certified',valid && radius>0,'normalizedRadius',radius, ...
                'speedInterval',[z.redundancy.minimumCourseSpeed,V],'zeroSpeedCertified',false, ...
                'allAuxiliaryOutputsRequired',true, ...
                'outerRadius',outer,'lyapunovLevel',level,'alpha',alpha,'gamma',gamma, ...
                'normalizedAdditiveInputLimit',inputLimit,'nominalDissipation',mu, ...
                'cubicRemainderBound',c1,'quarticRemainderBound',c2, ...
                'minimumMetricEigenvalue',pmin,'maximumMetricEigenvalue',pmax, ...
                'scope',"Conditional continuous eight-state ISS: V(0)<level and ||d||_infinity<inputLimit; true speed stays in the declared positive interval and ALL five auxiliary outputs remain available. Other motion bounds and a common L/G/M finite-overlap contract must hold throughout. Output withdrawal and zero speed are executable but outside this certificate. Input d includes upstream interface error, sensor error and model residual. This does not certify actual maps, the full lateral cascade or sampling.");
        end
    end
end
