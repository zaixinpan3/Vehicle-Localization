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
            [F,H,G]=localizationSupport.secants(x,x,c);
            [~,J]=localizationSupport.outputs(x,c);
            testCase.verifyEqual(H,J,AbsTol=0);
            testCase.verifyTrue(all(isfinite([F(:);H(:);G(:)])));
            testCase.verifyEqual(H(2,[2,5]),[0,1],AbsTol=0);
        end
        function finiteSecantsTransformWithTheRotatingFrame(testCase)
            errors=rotationErrors(mncavFullObserverConfig());
            testCase.verifyLessThan(max(errors),1e-12);
        end
        function sharedSensorVectorReconstructsEveryPacketIncrement(testCase)
            c=mncavFullObserverConfig();v=[12;.4];a=[.3;1.2];dv=[.2;-.1];da=[-.04;.08];
            N=localizationSupport.measurementSecant(v,a,v+dv,a+da);
            p=localizationSupport.measurements(v(1),v(2),a(1),a(2),true,true,c);
            m=localizationSupport.measurements(v(1)+dv(1),v(2)+dv(2),a(1)+da(1),a(2)+da(2),true,true,c);
            testCase.verifyEqual(m.value-p.value,N*[dv;da],AbsTol=3e-13);
            testCase.verifyEqual(N,eye(4));
        end
    end
end

function errors=finiteIdentityErrors(c)
    stream=RandStream('mt19937ar','Seed',2301);errors=zeros(250,3);
    for k=1:size(errors,1)
        x=randn(stream,8,1);y=randn(stream,8,1);
        x([2,5])=10*x([2,5]);y([2,5])=10*y([2,5]);
        if k==1,y(7)=x(7);elseif k==2,y(7)=x(7)+1e-12;end
        if k<=30,x([2,5])=x([2,5])/100;y([2,5])=y([2,5])/100;end
        [F,H,G]=localizationSupport.secants(x,y,c);e=x-y;
        dh=localizationSupport.outputs(x,c)-localizationSupport.outputs(y,c);
        dg=x([1,4,7])-y([1,4,7]);
        df=drift(x,c)-drift(y,c);
        errors(k,:)=[norm(F*e-df)/max(1,norm(df)),norm(H*e-dh)/max(1,norm(dh)), ...
            norm(G*e-dg)/max(1,norm(dg))];
    end
end

function errors=rotationErrors(c)
    angle=.91;R=[direction(angle),[-sin(angle);cos(angle)]];T=eye(8);
    T([1,4],[1,4])=R;T([2,5],[2,5])=R;T([3,6],[3,6])=R;
    x=[2;12;.6;-1;.4;1.1;.3;.07];y=x+[.4;-.2;.1;-.3;.1;-.2;.1;-.02];
    xr=T*x;yr=T*y;xr(7)=xr(7)+angle;yr(7)=yr(7)+angle;
    [F,H,G]=localizationSupport.secants(x,y,c);
    [Fr,Hr,Gr]=localizationSupport.secants(xr,yr,c);
    errors=[norm(Fr-T*F*T.'),norm(Hr*T-H),norm(Gr*T-blkdiag(R,1)*G)];
end

function y=drift(x,~)
    r=x(8);v=x([2,5]);a=x([3,6]);j=r^2*v+2*r*[-a(2);a(1)];
    y=[v(1);a(1);j(1);v(2);a(2);j(2);r;0];
end

function d=direction(psi)
    d=[cos(psi);sin(psi)];
end
