classdef autonomousPredictionTest < matlab.unittest.TestCase
% Body-frame transport identity and the Cartesian model used for gain synthesis.
    methods(TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));setupVehicleLocalization;
        end
    end
    methods(Test)
        function predictionUsesInternalBodyRateAtEverySpeed(testCase)
            c=mncavFullObserverConfig();h=.02;
            for speed=[0,.2,12]
                for r=[-.7,0,.13]
                    x=[1;speed;.6;2;.4;1.2;.3;r];y=localizationSupport.predict(x,h,c);
                    testCase.verifyEqual(y-x,h*drift(y),AbsTol=4e-15);
                end
            end
        end
        function physicalJerkIncludesBodyVelocityAndRateDerivatives(testCase)
            % Differentiate a trajectory with independently changing sideslip and yaw.
            t=.6;dt=1e-5;[v,a,R,r,uDD,rDot]=trajectory(t);
            [~,aPlus]=trajectory(t+dt);[~,aMinus]=trajectory(t-dt);
            J=[0,-1;1,0];residual=R*uDD+rDot*J*v;
            testCase.verifyEqual((aPlus-aMinus)/(2*dt),r^2*v+2*r*J*a+residual,AbsTol=1e-9);
            testCase.verifyGreaterThan(norm(residual),.1);
        end
        function zeroSpeedAndArbitrarySideslipJacobiansMatchFiniteDifferences(testCase)
            c=mncavFullObserverConfig();t=c.invariantObserver.theta;ell=c.invariantObserver.headingLengthM;
            T=diag([t,t^2,t^3,t,t^2,t^3,t/ell,t^2/ell]);
            for speed=[0,.2,2,12]
                for r=[-.31,0,.23]
                    v=speed*[.8;-.6];a=[.3;.7];x=[1;v(1);a(1);2;v(2);a(2);.2;r];
                    F=localizationSupport.secants(x,x,c);numeric=zeros(8);
                    for j=1:8
                        h=1e-5*max(1,abs(x(j)));p=x;m=x;p(j)=p(j)+h;m(j)=m(j)-h;
                        numeric(:,j)=(drift(p)-drift(m))/(2*h);
                    end
                    testCase.verifyEqual(F,numeric,AbsTol=2e-10);
                    A=localizationSupport.fusionMatrices(c,localizationSupport.fusionFeatures(v,a,r));
                    testCase.verifyEqual(A,(T\numeric*T)/t,AbsTol=2e-10);
                end
            end
        end
    end
end

function y=drift(x)
    r=x(8);v=x([2,5]);a=x([3,6]);j=r^2*v+2*r*[-a(2);a(1)];
    y=[v(1);a(1);j(1);v(2);a(2);j(2);r;0];
end

function [v,a,R,r,uDD,rDot]=trajectory(t)
    psi=.4*t+.15*t^2;r=.4+.3*t;rDot=.3;
    u=[3+t^2;.2+sin(t)];uD=[2*t;cos(t)];uDD=[2;-sin(t)];
    R=[cos(psi),-sin(psi);sin(psi),cos(psi)];v=R*u;
    a=R*uD+r*[-v(2);v(1)];
end
