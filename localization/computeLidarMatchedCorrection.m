function [correction,audit] = computeLidarMatchedCorrection(measurement,pose,G,design,cfg,options)
% computeLidarMatchedCorrection Continuous LiDAR injection and its discretization.
% C=D\(G*T), xi=-F*D'*J'*W*r or S*(D\(qL-pose)); the continuous
% correction is kappa*T*(P\(C'*xi)). P is a Lyapunov matrix, not covariance.
% Supply either pose/information or residual/jacobian/weights/linearizationPose.
% Residuals and physical pose Jacobians MUST be evaluated at POSE, with frozen
% associations; a final optimizer gradient is not the predicted-pose residual.
% WEIGHTS is an N-vector of diagonal weights or an N-by-N PSD precision matrix.
% Optional frameReliability and directionReliability lie in [0,1].
% Continuous mode returns a state derivative. Discrete modes return a state
% increment for the locally affine channel, with geometry frozen at POSE.
% Implicit: (I+h*P\Q)^-1 is P-nonexpansive for every h>=0.
% Explicit: h is capped to h*lambdaMax(P^-1/2*Q*P^-1/2)<=1.8<2.
% Neither jump certificate certifies nonlinear associations or baseline flow.
    arguments
        measurement (1,1) struct
        pose (3,1) double {mustBeReal,mustBeFinite}
        G double {mustBeReal,mustBeFinite}
        design (1,1) struct
        cfg (1,1) struct
        options.StepSize (1,1) double {mustBeReal,mustBeFinite,mustBeNonnegative} = 0
        options.Discretization (1,1) string {mustBeMember(options.Discretization,["continuous","implicit","explicit"])} = "continuous"
    end
    P=design.P;T=design.T;kappa=design.kappa;n=size(P,1);
    assert(isreal(P) && isequal(size(P),[n,n]) && all(isfinite(P),'all') && ...
        norm(P-P.','fro')<=1e-12*max(1,norm(P,'fro')), ...
        'VehicleLocalization:InvalidLidarMetric','P must be finite symmetric positive definite.');
    [L,flag]=chol((P+P.')/2,'lower');
    assert(flag==0 && isequal(size(T),[n,n]) && isreal(T) && all(isfinite(T),'all') ...
        && rcond(T)>eps && isequal(size(G),[3,n]), ...
        'VehicleLocalization:InvalidLidarMetric','Require SPD P, nonsingular fixed T and a 3-by-n pose Jacobian.');
    validateattributes(kappa,{'numeric'},{'real','finite','nonnegative','scalar'});
    residualMode=isfield(measurement,'residual');
    if residualMode
        assert(~isfield(measurement,'pose') && ~isfield(measurement,'information'), ...
            'VehicleLocalization:AmbiguousLidarMeasurement','Choose residual or pose information input.');
        assert(isfield(measurement,'linearizationPose') && ...
            isequal(measurement.linearizationPose(:),pose), ...
            'VehicleLocalization:StaleLidarLinearization','Evaluate scan residuals at the supplied predicted pose.');
        r=measurement.residual(:);J=measurement.jacobian;W=measurement.weights;
        assert(isreal(r) && all(isfinite(r)) && isequal(size(J),[numel(r),3]) && ...
            isreal(J) && all(isfinite(J),'all') && isreal(W) && all(isfinite(W),'all'), ...
            'VehicleLocalization:InvalidLidarResidual','Invalid residual, Jacobian or weights.');
        if isvector(W) && numel(W)==numel(r)
            assert(all(W>=0),'VehicleLocalization:InvalidLidarResidual','Diagonal weights must be nonnegative.');
            WJ=W(:).*J;Wr=W(:).*r;
        else
            assert(isequal(size(W),[numel(r),numel(r)]) && ...
                norm(W-W.','fro')<=1e-10*max(1,norm(W,'fro')), ...
                'VehicleLocalization:InvalidLidarResidual','Precision must be symmetric and aligned.');
            W=(W+W.')/2;[V,E]=eig(W);ev=diag(E);
            assert(all(ev>=-1e-10*max(1,norm(W,2))), ...
                'VehicleLocalization:InvalidLidarResidual','Precision must be PSD.');
            W=V*diag(max(ev,0))*V.';WJ=W*J;Wr=W*r;
        end
        information=J.'*WJ;gradient=J.'*Wr;
    else
        assert(isfield(measurement,'pose') && isfield(measurement,'information'), ...
            'VehicleLocalization:InvalidLidarMeasurement','Supply pose and physical information.');
        information=measurement.information;
    end
    frame=1;direction=ones(3,1);
    if isfield(measurement,'frameReliability'),frame=measurement.frameReliability;end
    if isfield(measurement,'directionReliability'),direction=measurement.directionReliability(:);end
    variance=[];
    if isfield(measurement,'poseErrorVariance'),variance=measurement.poseErrorVariance;end
    filter=filterLidarPoseInformation(information,cfg,frame,direction,variance);
    C=filter.D\(G*T);
    if residualMode
        xi=-filter.F*filter.D.'*gradient;representation="predictedPoseResidual";
    else
        innovation=measurement.pose(:)-pose;
        assert(numel(innovation)==3 && isreal(innovation) && all(isfinite(innovation)), ...
            'VehicleLocalization:InvalidLidarMeasurement','Pose must contain three finite values.');
        innovation(3)=atan2(sin(innovation(3)),cos(innovation(3)));
        xi=filter.S*(filter.D\innovation);representation="localPoseInformation";
    end
    Q=kappa*(C.'*filter.S*C);Q=(Q+Q.')/2;
    Z=L\Q/L.';Z=(Z+Z.')/2;
    eigenvalues=max(eig(Z),0);largest=max(eigenvalues);
    step=options.StepSize;v=kappa*(P\(C.'*xi));
    if options.Discretization=="continuous"
        correction=T*v;transition=eye(n);gainBound=kappa*norm(filter.F,2);
        if ~filter.calibrated,gainBound=kappa/filter.regularizer;end
    else
        if options.Discretization=="explicit" && largest>0,step=min(step,1.8/largest);end
        if options.Discretization=="implicit"
            A=eye(n)+step*(P\Q);scaled=A\(step*v);transition=A\eye(n);
        else
            scaled=step*v;transition=eye(n)-step*(P\Q);
        end
        correction=T*scaled;gainBound=NaN;
    end
    contraction=L.'*transition/L.';
    energyRatio=norm(contraction,2)^2;
    audit=struct('filter',filter,'C',C,'dissipationMatrix',Q, ...
        'continuousCorrection',T*v,'effectiveStrengths',kappa*filter.strengths, ...
        'maximumDissipationEigenvalue',largest,'requestedStep',options.StepSize, ...
        'appliedStep',step,'explicitStepProduct',step*largest, ...
        'jumpMaximumEnergyRatio',energyRatio, ...
        'jumpNonexpansive',energyRatio<=1+1e-10, ...
        'jumpCertificateApplies',options.Discretization~="continuous", ...
        'residualNoiseCoefficientBound',gainBound,'representation',representation, ...
        'scope',"Fixed-metric locally affine LiDAR channel only; no full-system or outage ISS claim");
end
