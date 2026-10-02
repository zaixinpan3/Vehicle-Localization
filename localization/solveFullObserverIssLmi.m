function certificate=solveFullObserverIssLmi(cfg)
% solveFullObserverIssLmi Test candidate gains in the continuous ISS LMIs.
% Solve before discretization. Gain/weight products are not jointly linear:
% outer parameter selection fixes the gains; this inner SDP finds a common
% Lyapunov certificate. An infeasible result is not a proof of instability.
% Requires YALMIP and the configured SDP solver on the MATLAB path.
    arguments
        cfg (1,1) struct
    end
    problem=fullObserverIssProblem(cfg);
    persistent cache
    if isempty(cache),cache=containers.Map('KeyType','char','ValueType','any');end
    key=jsonencode(struct('problem',problem,'solver',cfg.iss.solver));
    if isKey(cache,key),certificate=cache(key);return;end
    assert(exist('sdpvar','file')==2,'VehicleLocalization:MissingSolver', ...
        'Add YALMIP and the SDP solver to the path before continuous gain design.');
    % Homogeneous weights with fixed trace avoid the poorly conditioned
    % large-yaw-weight solutions produced by fixing the velocity weight to 1.
    x=sdpvar(4,1);slack=sdpvar(1);cp=x(1);cy=x(2);cv=x(3);mu=x(4);g=problem.gains;
    W=diag(x);constraints=[x>=1e-9,sum(x)==1];
    for q2=[0,problem.maximumCourseRate^2]
        a=cp*g(1);b=cy*g(4);cross=-(a+b)*problem.maximumPositionHeadingCoupling;
        M=[2*a*problem.minimumPositionStrength,cross,-cp,0; ...
            cross,2*b*problem.minimumHeadingStrength-cp*problem.gnssLeverPenalty, ...
            -cv*g(2)*problem.maximumSpeed,-mu*g(3)*problem.maximumAcceleration; ...
            -cp,-cv*g(2)*problem.maximumSpeed,2*cv*g(2),-(cv+mu*q2); ...
            0,-mu*g(3)*problem.maximumAcceleration,-(cv+mu*q2),2*mu*g(3)];
        constraints=[constraints,M-problem.decayRate*W>=slack*eye(4)]; %#ok<AGROW>
    end
    result=optimize(constraints,-slack,sdpsettings('solver',char(cfg.iss.solver),'verbose',0));
    recovered=value(x);
    certificate=struct('problem',problem,'weights',recovered([1,2,4])/recovered(3),'solverStatus',result.problem, ...
        'solverInfo',string(result.info),'solverSlack',value(slack),'certified',false, ...
        'designRoute',"Continuous ISS LMI, then discretization with unchanged gains");
    % A solver warning is not a mathematical infeasibility result. Accept a
    % returned witness only through the independent, strictly positive LMI
    % margin, and retain the solver status even when that witness is valid.
    if all(isfinite(certificate.weights)) && all(certificate.weights>0)
        certificate.verification=verifyFullObserverIssCertificate(certificate,cfg);
        certificate.certified=certificate.verification.certified;
    end
    if certificate.certified,cache(key)=certificate;end
end
