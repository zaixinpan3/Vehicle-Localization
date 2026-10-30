classdef motionPositionCouplingSupport
% Bound the position displacement induced by matched motion corrections.
% Motion outputs have zero position columns. With Pc=P(1:3,1:3), both the
% continuous matched correction and its frozen implicit step therefore obey
%   dp = -b_v*dv - b_a*da,
%   [b_v,b_a] = Pc(1,2:3)/Pc(1,1)./[theta,theta^2].
% This identity holds separately in both Cartesian axes. Constrain
%   norm([sigma_v*b_v,sigma_a*b_a]) <= rho*sigma_Lp.
% Multiplication by Pc(1,1)>0 gives the affine 3-by-3 Schur LMI below.
% The scales are declared design priors, not certified measurement errors.
% This bounds coupling per unit noise-normalized motion correction; it is
% neither a position-error bound, an empirical covariance nor runtime clipping.
    methods (Static)
        function block=lmi(Pc,cfg)
            [budget,scales]=motionPositionCouplingSupport.parameters(cfg);
            a=Pc(1,2:3).'.*scales;
            radius=budget*Pc(1,1);
            block=[radius,a.';a,radius*eye(2)];
        end

        function v=verify(P,cfg)
        % Independent scalar check, without YALMIP or a stored audit result.
            z=cfg.invariantObserver;
            required=isfield(z,'maximumMotionPositionNoiseRatio');
            v=struct('required',required,'verified',~required);
            if ~required,return;end
            [budget,scales]=motionPositionCouplingSupport.parameters(cfg);
            p=P(1,1);coupling=P(1,2:3)/p;
            strength=norm(coupling.*scales.');
            v=struct('required',true, ...
                'verified',p>0 && isfinite(strength) && strength<=budget*(1+1e-7), ...
                'physicalCoupling',coupling./[z.theta,z.theta^2], ...
                'scaledCouplingM',strength,'maximumScaledCouplingM',budget, ...
                'maximumNoiseRatio',z.maximumMotionPositionNoiseRatio, ...
                'scope',"Metric coupling bound for matched motion corrections. Declared sensor scales are design priors; actual position error, covariance and sampled trajectory maxima are not bounded by this constraint.");
        end

        function [budget,scales]=parameters(cfg)
            z=cfg.invariantObserver;n=z.noise;
            validateattributes(z.maximumMotionPositionNoiseRatio,{'double'},{'real','finite','positive','scalar'});
            validateattributes([n.lidarPositionM,n.velocityMps,n.accelerationMps2,z.theta], ...
                {'double'},{'real','finite','positive'});
            budget=z.maximumMotionPositionNoiseRatio*n.lidarPositionM;
            scales=[n.velocityMps/z.theta;n.accelerationMps2/z.theta^2];
        end
    end
end
