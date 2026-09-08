function result = verifyAnisotropicPoseCertificate(c,fixture)
% verifyAnisotropicPoseCertificate Verify every nonlinear vertex and full sector.
% W is any symmetric normalized pose weight in [alpha*I,I], including rotated
% eigenvectors and XY/yaw cross terms. A norm-bounded uncertainty LMI covers
% the whole sector, not a finite sampling of information-matrix directions.
    b=buildImprovedObserverCertificateData(fixture);
    knots=c.knots(:).'; n=numel(knots);
    assert(c.alpha>0 && c.alpha<1 && c.rate>0 && c.onTime<=c.tMin);
    assert(size(c.P,3)==n && all(diff(knots)>0) && knots(1)==0);
    assert(abs(knots(end)-c.tMax)<1e-12 && any(abs(knots-c.onTime)<1e-12));
    assert(any(abs(knots-c.tMin)<1e-12) && all(isfinite(c.P),'all'));
    assert(isequal(size(c.multipliers),[2,n-1]) && all(isfinite(c.multipliers),'all'));
    D=diag(c.poseScales); center=(1+c.alpha)/2; radius=(1-c.alpha)/2;
    B=-c.theta*radius*fixture.K*D; E=D\b.Cb;
    minP=Inf;maxP=-Inf;maxReset=-Inf;
    symmetry=max(abs(c.P-permute(c.P,[2,1,3])),[],'all');
    for k=1:n
        spectrum=eig((c.P(:,:,k)+c.P(:,:,k).')/2);
        minP=min(minP,min(spectrum));maxP=max(maxP,max(spectrum));
        if knots(k)>=c.tMin-1e-12
            R=c.P(:,:,1)-c.P(:,:,k);maxReset=max(maxReset,max(eig((R+R.')/2)));
        end
    end
    worst=-Inf; witness=[];count=0;maxOff=-Inf;
    for h=1:b.outputVertexCount
        for f=1:b.fVertexCount
            Aoff=c.theta*(b.A+b.fVertices(:,:,f)+fixture.N*b.outputVertices(:,:,h));
            maxOff=max(maxOff,max(eig((Aoff+Aoff.')/2)));
            for k=1:n-1
                active=knots(k)<c.onTime-1e-12;
                A=Aoff-active*c.theta*center*fixture.K*b.Cb;
                Pd=(c.P(:,:,k+1)-c.P(:,:,k))/(knots(k+1)-knots(k));
                for side=1:2
                    P=c.P(:,:,k+side-1);Q=A.'*P+P*A+Pd+2*c.rate*P;
                    if active
                        tau=c.multipliers(side,k);
                        assert(tau>0);
                        PB=P*B;Q=[Q+tau*(E.'*E),PB;PB.',-tau*eye(3)];
                    end
                    value=max(eig((Q+Q.')/2));count=count+1;
                    if value>worst,worst=value;witness=[h,f,k,side];end
                end
            end
        end
    end
    result=struct('passed',symmetry<1e-12 && minP>1e-5 && worst<-1e-6 && maxReset<-1e-6, ...
        'maximumFlowEigenvalue',worst,'minimumPEigenvalue',minP,'maximumPEigenvalue',maxP, ...
        'maximumResetEigenvalue',maxReset,'symmetryError',symmetry, ...
        'flowInequalityCount',count,'worstCombination',witness, ...
        'minimumNormalizedPoseWeight',c.alpha,'RDecayRatePerSecond',c.rate, ...
        'predictionEuclideanLogNormPerSecond',maxOff, ...
        'minimumQualifiedIntervalSeconds',c.tMin,'maximumQualifiedIntervalSeconds',c.tMax, ...
        'pulseDurationSeconds',c.onTime,'solverStatus',c.status,'solverOptimalityClaimed',false);
    % Every fused W is a contraction, even outside the positive lower sector.
    % This supplies a finite prediction-tail bound, not asymptotic stability.
    result.maximumContinuationLogNormPerSecond=maxOff+c.theta*norm(fixture.K*D,2)*norm(E,2);
    result.continuation150msHomogeneousBound=exp(.15*result.maximumContinuationLogNormPerSecond);
    result.continuation300msHomogeneousBound=exp(.30*result.maximumContinuationLogNormPerSecond);
end
