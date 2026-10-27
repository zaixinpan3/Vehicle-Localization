classdef (SharedTestFixtures={freshObserverGainsFixture}) invariantVectorIssTest < matlab.unittest.TestCase
% Finite nonlinear identities, correlated inputs and regional dissipation.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));setupVehicleLocalization;
        end
    end
    methods (Test)
        function finiteSecantsMatchIndependentNonlinearDifferences(testCase)
            error=finiteIdentityErrors(mncavFullObserverConfig());
            testCase.verifyLessThan(max(error),2e-12);
        end
        function equalStatesRecoverTheJacobianIncludingZeroSpeed(testCase)
            c=mncavFullObserverConfig();x=zeros(8,1);
            [F,H,G]=invariantIssSupport.secants(x,x,0,0,c);
            [~,J]=invariantObserverSupport.outputs(x,0,c);
            testCase.verifyEqual(H,J,AbsTol=0);
            testCase.verifyTrue(all(isfinite([F(:);H(:);G(:)])));
            testCase.verifyEqual(H(4,[2,5]),[0,1],AbsTol=0);
        end
        function finiteSecantsTransformWithTheRotatingFrame(testCase)
            errors=rotationErrors(mncavFullObserverConfig());
            testCase.verifyLessThan(max(errors),1e-12);
        end
        function sharedSensorVectorReconstructsEveryPacketIncrement(testCase)
            c=mncavFullObserverConfig();v=[12;.4];a=[.3;1.2];dv=[.2;-.1];da=[-.04;.08];dr=.003;
            N=invariantIssSupport.measurementSecant(v,a,v+dv,a+da,c);
            p=invariantObserverSupport.measurements(v(1),v(2),a(1),a(2),.1,0,true,true,true,c);
            m=invariantObserverSupport.measurements(v(1)+dv(1),v(2)+dv(2),a(1)+da(1),a(2)+da(2),.1+dr,0,true,true,true,c);
            testCase.verifyEqual(m.value-p.value,N*[dv;da;dr],AbsTol=2e-14);
            testCase.verifySize(N,[5,5]);
            testCase.verifyEqual(N(4,:),zeros(1,5),AbsTol=0);
            testCase.verifyGreaterThan(norm(N(2:3,3:4)),0);
        end
        function bernsteinControlsRepresentCoupledRatesThroughoutCells(testCase)
            controls=sensorRedundancySupport.courseControls(.4,2);u=.37;
            rate=-.4+.4*u;weights=[(1-u)^2,2*u*(1-u),u^2];
            testCase.verifyEqual(weights*controls(1:3,:),[rate,rate^2],AbsTol=2e-16);
            testCase.verifyLessThanOrEqual(max(controls(:,2)),.4^2);
            testCase.verifyGreaterThanOrEqual(min(controls(:,2)),0);
            testCase.verifyError(@()sensorRedundancySupport.courseControls(.4,3), ...
                'VehicleLocalization:InvalidRedundancyDomain');
        end
        function regionalBoundHasAStrictInvariantLevelAndInputBudget(testCase)
            c=freshObserverTestConfig();r=c.invariantDesign.verification.regionalIss;
            testCase.verifyTrue(r.certified);
            testCase.verifyGreaterThan(r.normalizedRadius,0);
            testCase.verifyLessThanOrEqual(r.normalizedRadius,r.outerRadius);
            testCase.verifyLessThanOrEqual(r.cubicRemainderBound*r.normalizedRadius ...
                +r.quarticRemainderBound*r.normalizedRadius^2,.500001*r.nominalDissipation);
            testCase.verifyEqual(r.lyapunovLevel,r.minimumMetricEigenvalue*r.normalizedRadius^2,RelTol=1e-12);
            testCase.verifyEqual(r.gamma*r.normalizedAdditiveInputLimit^2,2*r.alpha*r.lyapunovLevel,RelTol=1e-12);
        end
        function nonlinearDissipationHoldsAcrossComplementaryFamilies(testCase)
            ratios=regionalDissipation(freshObserverTestConfig());
            testCase.verifyLessThan(max(ratios),-.49);
        end
        function zeroSpeedIsOutsideEveryCertifiedSensorFamily(testCase)
            c=freshObserverTestConfig();v=c.invariantDesign.verification;
            testCase.verifyFalse(v.zeroSpeedCertified);
            testCase.verifyFalse(v.jointSensor.zeroSpeedCertified);
            testCase.verifyEqual(v.regionalIss.speedInterval,[10,15],AbsTol=0);
        end
        function removingEitherMixedAnchorRejectsItsCertificate(testCase)
            c=freshObserverTestConfig();d=c.invariantDesign;
            w=sensorRedundancySupport.witness(c,d.P,d.kappaL,0,d.auxiliaryWeights,d.gyroGainNormalized);
            v=sensorRedundancySupport.verify(w,true);
            testCase.verifyFalse(v.certified);
            testCase.verifyGreaterThan(v.worstMixedEigenvalue,0);
        end
        function regionalDomainChangesInvalidateTemporaryGains(testCase)
            c=freshObserverTestConfig();c.invariantObserver.regionalRadiusCap=.5;
            testCase.verifyError(@()invariantObserverSupport.runtime(c),'VehicleLocalization:StaleInvariantDesign');
        end
        function sampledAndActualGeometryClaimsRemainWithdrawn(testCase)
            r=invariantObserverSupport.runtime(freshObserverTestConfig());
            testCase.verifyTrue(r.continuousCertificate.verification.numericalInvariantRadiusCertified);
            testCase.verifyFalse(r.actualOverlapCertified);
            testCase.verifyFalse(r.sampledSystemCertified);
            testCase.verifyFalse(r.fullContinuousStateCertified);
        end
    end
end

function errors=finiteIdentityErrors(c)
    stream=RandStream('mt19937ar','Seed',2301);errors=zeros(250,3);
    for k=1:size(errors,1)
        x=randn(stream,8,1);y=randn(stream,8,1);beta=.2;rate=-.04;
        x([2,5])=10*x([2,5]);y([2,5])=10*y([2,5]);
        if k==1,y(7)=x(7);elseif k==2,y(7)=x(7)+1e-12;end
        [F,H,G]=invariantIssSupport.secants(x,y,beta,rate,c);e=x-y;
        dh=invariantObserverSupport.outputs(x,beta,c)-invariantObserverSupport.outputs(y,beta,c);
        dg=antenna(x,c)-antenna(y,c);df=drift(x,rate)-drift(y,rate);
        errors(k,:)=[norm(F*e-df)/max(1,norm(df)),norm(H*e-dh)/max(1,norm(dh)), ...
            norm(G*e-dg)/max(1,norm(dg))];
    end
end

function errors=rotationErrors(c)
    angle=.91;R=[direction(angle),[-sin(angle);cos(angle)]];T=eye(8);
    T([1,4],[1,4])=R;T([2,5],[2,5])=R;T([3,6],[3,6])=R;
    x=[2;12;.6;-1;.4;1.1;.3;.07];y=x+[.4;-.2;.1;-.3;.1;-.2;.1;-.02];
    xr=T*x;yr=T*y;xr(7)=xr(7)+angle;yr(7)=yr(7)+angle;
    [F,H,G]=invariantIssSupport.secants(x,y,.1,.03,c);
    [Fr,Hr,Gr]=invariantIssSupport.secants(xr,yr,.1,.03,c);
    errors=[norm(Fr-T*F*T.'),norm(Hr*T-H),norm(Gr*T-R*G)];
end

function ratios=regionalDissipation(c)
    d=c.invariantDesign;r=d.verification.regionalIss;t=c.invariantObserver.theta;ell=c.invariantObserver.headingLengthM;
    B=diag(t.^[1,2,3,1,2,3,1,2]./[ones(1,6),ell,ell]);D=diag(t.^-[2,3,3,2,2]);
    E=eye(8);C=E([1,4,7],:);Cp=C(1:2,:);K=diag([d.auxiliaryWeights([1,2,2,3]);0]);
    stream=RandStream('mt19937ar','Seed',2302);ratios=zeros(180,1);
    for k=1:numel(ratios)
        family=mod(k-1,3)+1;psi=6*rand(stream)-3;beta=.2;rate=.02;s=10+5*rand(stream);
        qLimit=min(c.maximumTrackAngleRate,c.iss.maximumAcceleration/s);
        q=qLimit*(2*rand(stream)-1);
        u=sqrt(max(0,c.iss.maximumAcceleration^2-(s*q)^2))*(2*rand(stream)-1);
        dc=direction(psi+beta);nc=[-dc(2);dc(1)];
        x=zeros(8,1);x(7:8)=[psi;q-rate];x([2,5])=s*dc;x([3,6])=u*dc+s*q*nc;
        eps=randn(stream,8,1);eps=.9*r.normalizedRadius*eps/norm(eps);hat=x-B*eps;
        WG=.5*eye(2);S=diag([.25,.25,.97]);valid=true(5,1);
        if family==1
            WG=zeros(2);
        elseif family==2
            S=zeros(3);
        else
            % Rank-one PSD LiDAR with cross coupling. It satisfies the scalar
            % bounds but does not dominate the former diagonal heading floor.
            coupling=c.iss.maximumPositionHeadingCoupling*[cos(k);sin(k)];h=c.iss.minimumHeadingStrength;
            S=[coupling*coupling.'/h,coupling;coupling.',h];valid(1:4)=false;
        end
        mismatch=c.invariantObserver.redundancy.informativeLidarMismatch;
        W=S-mismatch*eye(3)+.01*[0,1,0;-1,0,0;0,0,0];
        [hx,~]=invariantObserverSupport.outputs(x,beta,c);
        [hh,Hh]=invariantObserverSupport.outputs(hat,beta,c);
        Ka=K*diag(double(valid));Ka(5,5)=0;
        rhs=B\(drift(x,rate)-drift(hat,rate));
        rhs=rhs-t*d.gyroGainNormalized*E(:,8)*eps(8);
        rhs=rhs-t*d.kappaL*(d.P\(C.'*W*C*eps));
        rhs=rhs-d.kappaG*(d.P\(Cp.'*WG*(antenna(x,c)-antenna(hat,c))));
        rhs=rhs-t*(d.P\((D*Hh*B).'*Ka*D*(hx-hh)));
        ratios(k)=2*eps.'*d.P*rhs/(r.nominalDissipation*(eps.'*eps));
    end
end

function y=drift(x,b)
    q=x(8)+b;y=zeros(8,1);y([1,4])=x([2,5]);y([2,5])=x([3,6]);
    y([3,6])=q^2*x([2,5])+2*q*[0,-1;1,0]*x([3,6]);y(7)=x(8);
end

function d=direction(psi)
    d=[cos(psi);sin(psi)];
end

function y=antenna(x,c)
    d=direction(x(7));R=[d,[-d(2);d(1)]];y=x([1,4])+R*c.gnss.outputPoint.bodyOffset(:);
end
