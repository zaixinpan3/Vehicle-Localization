function certificate = verifyLateralObserverCertificate(design)
% verifyLateralObserverCertificate Independently recheck the interval certificate.
% Rebuild the domain from configuration and the model from vehicle parameters;
% do not trust saved vertex matrices, solver status or sampled gain estimates.
% The returned flag requires all twelve recovered affine Finsler blocks to
% be negative definite, all P_i >= I, a nonsingular slack, and the structural
% first-column identity. Numerical checks are in double precision, not an
% interval-arithmetic proof of floating-point roundoff.
    model = lateralObserverSupport.lateralBicycleModel(design.cfg.vehicle);
    polytope = lateralObserverSupport.buildSchedulingPolytope(design.cfg.scheduling.speedRange);
    domain = lateralObserverSupport.buildCertificateDomain(model, polytope, ...
        design.cfg.scheduling.longitudinalAccelerationRange);
    gains = design.vertexGains;
    basis = design.lyapunovBasis;
    X = design.slackMatrix;
    epsilon = design.slackScale;
    tau = design.tau;
    kappa = design.decayRate;
    gamma = design.issGain;
    lipschitz = design.lipschitzConstant;
    assert(isequal(size(gains), [2,2,3]) && isequal(size(basis), [2,2,3]) && ...
        isequal(size(X), [2,2]) && all(isfinite([gains(:); basis(:); X(:)])), ...
        "VehicleLocalization:InvalidLateralCertificate", "Invalid certificate matrices.");
    assert(all(isfinite([epsilon,tau,kappa,gamma,lipschitz])) && ...
        all([epsilon,tau,kappa,gamma] > 0) && lipschitz >= 0, ...
        "VehicleLocalization:InvalidLateralCertificate", "Invalid certificate scalars.");
    weights = domain.firstColumnWeights;
    residual = gains(:,1,3) - weights(1)*gains(:,1,1) - weights(2)*gains(:,1,2);
    g1 = (gains(:,1,2) - gains(:,1,1)) ./ domain.speedSpan;
    Ww = diag(design.axleForceMismatchScale);
    Wv = diag(design.measurementErrorScale);
    margins = zeros(3,4);
    blocks = zeros(10,10,3,4);
    minP = inf;
    maxP = 0;
    maxSymmetryResidual = 0;
    componentSquared = zeros(1,2);
    for i = 1:3
        P = basis(:,:,i);
        maxSymmetryResidual = max(maxSymmetryResidual, norm(P-P.', 'fro'));
        minP = min(minP, min(eig((P+P.')/2)));
        maxP = max(maxP, max(eig((P+P.')/2)));
        L = gains(:,:,i);
        Ae = domain.vertexA(:,:,i) - L*domain.vertexC(:,:,i) + ...
            domain.extensionFactors(i)*g1*model.C2(1,:);
        R = X*Ae;
        Bd = [(model.E-L*model.F)*Ww, -L*Wv];
        S = X*Bd;
        for j = 1:4
            d = domain.rateVertices(:,j);
            Pdot = d(1)*basis(:,:,1) + d(2)*basis(:,:,2) + d(3)*basis(:,:,3);
            Phi = R+R.'+Pdot+2*kappa*P+(lipschitz^2/tau)*eye(2);
            K = P-X+epsilon*R.';
            M = [Phi, K, S, sqrt(tau)*P; ...
                K.', -epsilon*(X+X.'), epsilon*S, zeros(2,2); ...
                S.', epsilon*S.', -gamma^2*eye(4), zeros(4,2); ...
                sqrt(tau)*P, zeros(2,2), zeros(2,4), -eye(2)];
            M = (M+M.')/2;
            blocks(:,:,i,j) = M;
            margins(i,j) = max(eig(M));
            % Fixed-certificate partial input bounds, using the same affine
            % block at every vertex. A physical-grid Schur maximum would
            % not be a certified interval bound.
            keep = [1:4,9:10];
            N = M(keep,keep);
            if max(eig(N)) < 0
                for channel = 1:2
                    columns = 4+(2*channel-1:2*channel);
                    T = M(keep,columns);
                    gainMatrix = T.'*((-N)\T);
                    componentSquared(channel) = max(componentSquared(channel), ...
                        max(eig((gainMatrix+gainMatrix.')/2)));
                end
            else
                componentSquared(:) = inf;
            end
        end
    end
    certificate = struct();
    certificate.method = "physical-curve-affine-extension-v1";
    certificate.vertexMargins = margins;
    certificate.vertexBlocks = blocks;
    certificate.structureResidual = norm(residual);
    certificate.minLyapunovEigenvalue = minP;
    certificate.maxLyapunovEigenvalue = maxP;
    certificate.minSlackEigenvalue = min(eig((X+X.')/2));
    certificate.mismatchIssGain = min(gamma, sqrt(componentSquared(1))*(1+1e-8));
    certificate.measurementIssGain = min(gamma, sqrt(componentSquared(2))*(1+1e-8));
    % Leave a roundoff guard relative to each block's norm. The solver's
    % requested strictness and the achieved numerical margin are distinct.
    blockNorm = max(arrayfun(@(k) norm(reshape(blocks(:,:,k),10,10),2), 1:12));
    certificate.roundoffGuard = 100*eps(max(1,blockNorm));
    certificate.certified = all(margins(:) < -certificate.roundoffGuard) && ...
        minP >= 1 && certificate.minSlackEigenvalue > 0 && ...
        maxSymmetryResidual <= 100*eps(max(1,maxP)) && ...
        certificate.structureResidual <= 100*eps(max(1,norm(gains(:)))) && ...
        isequal(design.polytope, polytope) && isequal(design.model, model);
end
