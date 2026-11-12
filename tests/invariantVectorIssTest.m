classdef (SharedTestFixtures={freshObserverGainsFixture}) invariantVectorIssTest < matlab.unittest.TestCase
% Finite nonlinear identities, correlated inputs and rotational equivariance.
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
            [F,H,G]=localizationSupport.secants(x,x,0,c);
            [~,J]=localizationSupport.outputs(x,c);
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
            N=localizationSupport.measurementSecant(v,a,v+dv,a+da);
            p=localizationSupport.measurements(v(1),v(2),a(1),a(2),.1,true,true,true,c);
            m=localizationSupport.measurements(v(1)+dv(1),v(2)+dv(2),a(1)+da(1),a(2)+da(2),.1+dr,true,true,true,c);
            testCase.verifyEqual(m.value-p.value,N*[dv;da],AbsTol=3e-13);
            testCase.verifySize(N,[4,4]);
            testCase.verifyEqual(N(4,:),[0,1,0,0],AbsTol=0);
            testCase.verifyGreaterThan(norm(N(2:3,3:4)),0);
        end
    end
end

function errors=finiteIdentityErrors(c)
    stream=RandStream('mt19937ar','Seed',2301);errors=zeros(250,3);
    for k=1:size(errors,1)
        x=randn(stream,8,1);y=randn(stream,8,1);rate=-.04;qd=.3*randn(stream);
        x([2,5])=10*x([2,5]);y([2,5])=10*y([2,5]);
        if k==1,y(7)=x(7);elseif k==2,y(7)=x(7)+1e-12;end
        [F,H,G]=localizationSupport.secants(x,y,rate,c,qd);e=x-y;
        dh=localizationSupport.outputs(x,c)-localizationSupport.outputs(y,c);
        dg=antenna(x,c)-antenna(y,c);df=drift(x,rate,qd)-drift(y,rate,qd);
        errors(k,:)=[norm(F*e-df)/max(1,norm(df)),norm(H*e-dh)/max(1,norm(dh)), ...
            norm(G*e-dg)/max(1,norm(dg))];
    end
end

function errors=rotationErrors(c)
    angle=.91;R=[direction(angle),[-sin(angle);cos(angle)]];T=eye(8);
    T([1,4],[1,4])=R;T([2,5],[2,5])=R;T([3,6],[3,6])=R;
    x=[2;12;.6;-1;.4;1.1;.3;.07];y=x+[.4;-.2;.1;-.3;.1;-.2;.1;-.02];
    xr=T*x;yr=T*y;xr(7)=xr(7)+angle;yr(7)=yr(7)+angle;
    [F,H,G]=localizationSupport.secants(x,y,.03,c,.17);
    [Fr,Hr,Gr]=localizationSupport.secants(xr,yr,.03,c,.17);
    errors=[norm(Fr-T*F*T.'),norm(Hr*T-H),norm(Gr*T-R*G)];
end

function y=drift(x,b,qd)
    q=x(8)+b;y=zeros(8,1);y([1,4])=x([2,5]);y([2,5])=x([3,6]);
    y([3,6])=(q^2*eye(2)+qd*[0,-1;1,0])*x([2,5])+2*q*[0,-1;1,0]*x([3,6]);y(7)=x(8);
end

function d=direction(psi)
    d=[cos(psi);sin(psi)];
end

function y=antenna(x,c)
    d=direction(x(7));R=[d,[-d(2);d(1)]];y=x([1,4])+R*c.gnss.outputPoint.bodyOffset(:);
end
