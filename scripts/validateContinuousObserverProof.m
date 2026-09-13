function report = validateContinuousObserverProof(outputFolder)
% validateContinuousObserverProof Check the continuous-mode ISS construction.
% The GNSS calculation covers the full declared q/H box by an analytic norm
% bound. The LiDAR calculation verifies a constant-matrix delay certificate
% and admits a conservative, explicitly computed neighborhood of q/H/W.
% This does not certify the existing event-based observer runtime or gains.

    arguments
        outputFolder (1, 1) string = "research/continuous_observer_iss_20260913"
    end
    assert(exist('sdpvar','file') == 2 && exist('sedumi','file') == 2, ...
        'YALMIP and SeDuMi must already be on the MATLAB path.');
    if ~isfolder(outputFolder), mkdir(outputFolder); end
    rng(20260913,'twister');
    chain = [0,1,0;0,0,1;0,0,0];
    A = blkdiag(chain,chain,0);
    C = eye(7); C = C([1,4,7],:);
    K = zeros(7,3); K(1:3,1) = [3;3;1];
    K(4:6,2) = [3;3;1]; K(7,3) = 1;
    pattern = zeros(7,4);
    pattern(2,1) = 1; pattern(5,1) = 1;
    pattern(3,2) = 1; pattern(6,2) = 1;
    pattern(3,3) = 1; pattern(6,3) = -1; pattern(7,4) = -1;
    maximumVelocity = 16; maximumAcceleration = 5;
    outputBound3 = sqrt(12*maximumVelocity^2+4*maximumAcceleration^2);
    outputBound4 = sqrt(16*maximumVelocity^2+4*maximumAcceleration^2+2);

    % Constructive GNSS certificate, not a sampled coefficient check.
    F6 = A(1:6,1:6)-K(1:6,1:2)*C(1:2,1:6);
    operator = kron(eye(6),F6.')+kron(F6.',eye(6));
    identity6 = eye(6);
    P6 = reshape(operator\(-identity6(:)),6,6);
    P6 = (P6+P6.')/2;
    NG = .002*pattern(1:6,1:3);
    thetaG = 20; qBoundG = .6;
    perturbationG = qBoundG*sqrt(qBoundG^2+4)+norm(NG,2)*outputBound3;
    marginG = thetaG-2*norm(P6,2)*perturbationG;
    assert(min(eig(P6)) > 0 && marginG > 0);
    report.gnss = struct('theta',thetaG,'maximumCourseRate',qBoundG, ...
        'maximumVelocityComponent',maximumVelocity, ...
        'maximumAccelerationComponent',maximumAcceleration, ...
        'P6',P6,'K',K(1:6,1:2),'N',NG, ...
        'lyapunovIdentityResidual',norm(F6.'*P6+P6*F6+eye(6),'fro'), ...
        'analyticUniformMargin',marginG, ...
        'scope',"Six-state robust block; seventh-state ISS also requires Theorem G's motion/chart/admission conditions.");

    % Illustrative fixed-delay LMI, followed by a rigorous norm enclosure.
    delay = .15; thetaL = 1; rate = .05;
    A0 = thetaL*A; Ad = -thetaL*K*C;
    yalmip('clear');
    P = sdpvar(7,7,'symmetric'); Q = sdpvar(7,7,'symmetric');
    R = sdpvar(7,7,'symmetric'); g = sdpvar(1); margin = sdpvar(1);
    block = delayBlock(A0,Ad,P,Q,R,g,delay,rate);
    constraints = [P >= 1e-4*eye(7), Q >= 1e-4*eye(7), ...
        R >= 1e-4*eye(7), trace(P) == 7, trace(Q)+trace(R) <= 1000, ...
        g >= 1e-4, g <= 1000, margin >= 1e-6, block <= -margin*eye(28)];
    diagnostics = optimize(constraints,-margin,sdpsettings('solver','sedumi','verbose',0));
    assert(diagnostics.problem == 0,'Constant-delay synthesis failed: %s',diagnostics.info);
    P = double(P); Q = double(Q); R = double(R); g = double(g);
    P = (P+P.')/2; Q = (Q+Q.')/2; R = (R+R.')/2;
    recovered = delayBlock(A0,Ad,P,Q,R,g,delay,rate);
    nominalMargin = -max(eig((recovered+recovered.')/2));
    assert(min([eig(P);eig(Q);eig(R)]) > 1e-6 && nominalMargin > 1e-6);

    % A perturbation of either drift changes the symmetric LMI by at most
    % 2*(||P||+d*||R||)*(||delta A0||+||delta Ad||).
    totalBudget = nominalMargin/(4*(norm(P,2)+delay*norm(R,2)));
    qBoundL = min(1,totalBudget/9);
    informationRadius = min(.5,totalBudget/(3*thetaL*norm(K,2)*norm(C,2)));
    NL = (totalBudget/(3*outputBound4))*pattern/norm(pattern,2);
    driftBound = qBoundL*sqrt(qBoundL^2+4)+norm(NL,2)*outputBound4;
    delayedBound = thetaL*norm(K,2)*informationRadius*norm(C,2);
    certifiedMargin = nominalMargin-2*(norm(P,2)+delay*norm(R,2))* ...
        (driftBound+delayedBound);
    assert(certifiedMargin > 0);
    report.lidar = struct('theta',thetaL,'delaySeconds',delay,'rate',rate, ...
        'P',P,'Q',Q,'R',R,'g',g,'K',K,'N',NL, ...
        'nominalBlockMargin',nominalMargin,'analyticUniformMargin',certifiedMargin, ...
        'maximumCourseRate',qBoundL,'minimumInformationWeight',1-informationRadius, ...
        'maximumVelocityComponent',maximumVelocity, ...
        'maximumAccelerationComponent',maximumAcceleration, ...
        'scope',"Continuous delayed observer with all four auxiliary channels and arbitrary symmetric W in the stated narrow sector; not the production gains or q=0.6 envelope.");

    % Independent algebra checks on the actual nonlinear model and outputs.
    algebraResidual = 0; schurResidual = 0; jensenSlack = Inf;
    operating = struct('maximumSpeed',maximumVelocity,'maximumAcceleration',maximumAcceleration);
    for trial = 1:100
        sample = struct('longitudinalSpeed',8,'lateralVelocity',.2, ...
            'longitudinalAcceleration',.3,'lateralAcceleration',.4, ...
            'yawRate',.2*randn,'sideSlipAngle',.03,'sideSlipAngleRate',.02);
        trueState = randn(7,1); estimated = randn(7,1);
        pastTrue = randn(7,1); pastEstimated = randn(7,1);
        plantDisturbance = .01*randn(7,1); poseNoise = .01*randn(3,1);
        poseWeight = .9*eye(3);
        truth = evaluateImprovedObserverChannels(trueState,sample,operating);
        estimate = evaluateImprovedObserverChannels(estimated,sample,operating);
        measuredAuxiliary = truth.invariantPrediction+.01*randn(4,1);
        derivativeTrue = truth.modelDerivative+plantDisturbance;
        derivativeEstimate = estimate.modelDerivative+K*poseWeight* ...
            (C*pastTrue+poseNoise-C*pastEstimated)+NL* ...
            (measuredAuxiliary-estimate.invariantPrediction);
        expectedError = truth.modelMatrix*(trueState-estimated)-K*poseWeight*C* ...
            (pastTrue-pastEstimated)-NL*(truth.invariantPrediction-estimate.invariantPrediction)+ ...
            plantDisturbance-K*poseWeight*poseNoise-NL*(measuredAuxiliary-truth.invariantPrediction);
        algebraResidual = max(algebraResidual,norm(derivativeTrue-derivativeEstimate-expectedError));
        current = randn(7,1); past = randn(7,1); input = randn(7,1);
        zeta = [current;past;input]; derivative = A0*current+Ad*past+input;
        rdelay = exp(-2*rate*delay);
        upper = 2*current.'*P*derivative+2*rate*current.'*P*current+ ...
            current.'*Q*current-rdelay*past.'*Q*past+delay^2*derivative.'*R*derivative- ...
            rdelay*(current-past).'*R*(current-past)-g*(input.'*input);
        schur = recovered(1:21,1:21)+delay^2*[A0,Ad,eye(7)].'*R*[A0,Ad,eye(7)];
        schurResidual = max(schurResidual,abs(upper-zeta.'*schur*zeta));
        % Affine history: weighted integral evaluated analytically.
        slope = randn(7,1);
        weightedIntegral = (1-rdelay)/(2*rate)*(slope.'*R*slope);
        jensenSlack = min(jensenSlack,delay*weightedIntegral-rdelay*delay^2*(slope.'*R*slope));
    end
    assert(algebraResidual < 1e-10 && schurResidual < 1e-8 && jensenSlack >= -1e-10);
    report.checks = struct('seed',20260913,'cases',100, ...
        'delayedErrorIdentityResidual',algebraResidual, ...
        'functionalSchurIdentityResidual',schurResidual,'minimumJensenSlack',jensenSlack, ...
        'allPassed',true,'matlabVersion',version);
    report.limitations = ["No runtime migration or vehicle simulation performed.", ...
        "LiDAR certificate is an illustrative conservative neighborhood, not the historical operating sector.", ...
        "GNSS heading requires motion and the explicit local admission inequality."];
    file = fopen(fullfile(outputFolder,'certificate_checks.json'),'w');
    assert(file >= 0); cleanup = onCleanup(@() fclose(file));
    fprintf(file,'%s\n',jsonencode(report,PrettyPrint=true));
    disp(report.gnss); disp(report.lidar); disp(report.checks);
end

function block = delayBlock(A0,Ad,P,Q,R,g,delay,rate)
% delayBlock Build the affine Schur form in equations (26)--(27).
    rdelay = exp(-2*rate*delay);
    Pi = [P*A0+A0.'*P+2*rate*P+Q-rdelay*R, P*Ad+rdelay*R, P; ...
        Ad.'*P+rdelay*R, -rdelay*(Q+R), zeros(7); ...
        P, zeros(7), -g*eye(7)];
    flow = [A0,Ad,eye(7)];
    block = [Pi,delay*flow.'*R;delay*R*flow,-R];
end
