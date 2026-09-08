function c = designAnisotropicPoseCertificate(fixture,options)
% designAnisotropicPoseCertificate Synthesize a timer metric for full W sectors.
% A shared nonnegative multiplier at each timer endpoint bounds every matrix
% orientation. Constraint generation adds violating nonlinear model vertices.
    arguments
        fixture (1,1) struct
        options.MinimumWeight (1,1) double = .5
        options.MaximumIterations (1,1) double = 25
    end
    assert(exist('sdpvar','file')~=0 && exist('sedumi','file')~=0);
    b=buildImprovedObserverCertificateData(fixture);
    c=struct('passed',false,'theta',fixture.observer.theta,'alpha',options.MinimumWeight, ...
        'tMin',fixture.measurement.minimumPoseInterval,'tMax',fixture.measurement.maximumPoseInterval, ...
        'onTime',fixture.measurement.lidarMaximumAge,'rate',.01,'poseScales',fixture.lidar.poseScales(:));
    assert(c.alpha>0 && c.alpha<1);
    c.knots=unique(round([0:.01:c.tMax,c.tMin,c.tMax,c.onTime],12)); n=numel(c.knots);
    D=diag(c.poseScales);E=D\b.Cb;B=-c.theta*(1-c.alpha)/2*fixture.K*D;
    center=(1+c.alpha)/2;
    vertices=[1,1;b.outputVertexCount,b.fVertexCount;4000,2];
    for iteration=1:options.MaximumIterations
        yalmip('clear');P=sdpvar(7,7,n,'symmetric');tau=sdpvar(2,n-1,'full');margin=sdpvar(1);
        F=[trace(P(:,:,1))==7,margin>=1e-7,margin<=1,tau(:)>=1e-7];
        for k=1:n
            F=[F,P(:,:,k)>=1e-5*eye(7)]; %#ok<AGROW>
            if c.knots(k)>=c.tMin-1e-12,F=[F,P(:,:,1)<=P(:,:,k)-1e-4*eye(7)];end %#ok<AGROW>
        end
        for j=1:size(vertices,1)
            Aoff=c.theta*(b.A+b.fVertices(:,:,vertices(j,2))+fixture.N*b.outputVertices(:,:,vertices(j,1)));
            for k=1:n-1
                active=c.knots(k)<c.onTime-1e-12;
                A=Aoff-active*c.theta*center*fixture.K*b.Cb;
                Pd=(P(:,:,k+1)-P(:,:,k))/(c.knots(k+1)-c.knots(k));
                for side=1:2
                    Pe=P(:,:,k+side-1);Q=A.'*Pe+Pe*A+Pd+2*c.rate*Pe;
                    if active
                        PB=Pe*B;Q=[Q+tau(side,k)*(E.'*E),PB;PB.',-tau(side,k)*eye(3)];
                    end
                    F=[F,Q<=-margin*eye(size(Q,1))]; %#ok<AGROW>
                end
            end
        end
        sol=optimize(F,-margin,sdpsettings('solver','sedumi','verbose',0));c.status=sol.problem;
        c.iteration=iteration;c.vertices=vertices;
        if ~ismember(sol.problem,[0,4]),return;end
        c.P=value(P);c.multipliers=value(tau);c.margin=value(margin);
        if any(~isfinite(c.P),'all') || any(~isfinite(c.multipliers),'all'),return;end
        check=verifyAnisotropicPoseCertificate(c,fixture);c.verification=check;
        fprintf('Full-matrix sector alpha %.3f, iteration %d, margin %.6g\n',c.alpha,iteration,check.maximumFlowEigenvalue);
        if check.passed,c.passed=true;return;end
        newVertex=check.worstCombination(1:2);
        if ismember(newVertex,vertices,'rows'),return;end
        vertices=unique([vertices;newVertex],'rows');
    end
end
