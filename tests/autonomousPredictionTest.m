classdef autonomousPredictionTest < matlab.unittest.TestCase
% State-only prediction, physical identities and regularized finite increments.
    methods(TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));setupVehicleLocalization;
        end
    end
    methods(Test)
        function changingYawRateCannotChangeTranslationalPrediction(testCase)
            c=mncavFullObserverConfig();x=[1;12;.6;2;.4;1.2;.3;.1];y=x;y(8)=-.7;
            a=localizationSupport.predict(x,.02,c);b=localizationSupport.predict(y,.02,c);
            testCase.verifyEqual(a(1:6),b(1:6),AbsTol=0);
            testCase.verifyEqual(b(7)-a(7),.02*(y(8)-x(8)),AbsTol=1e-14);
        end
        function courseRateRecoversRotatingTrajectory(testCase)
            c=mncavFullObserverConfig();phi=.7;s=12;b=.6;q=.13;
            R=[cos(phi),-sin(phi);sin(phi),cos(phi)];v=R*[s;0];a=R*[b;s*q];
            x=[0;v(1);a(1);0;v(2);a(2);.3;-.2];
            testCase.verifyEqual(localizationSupport.courseRate(x,c),q,AbsTol=1e-14);
        end
        function jacobianMatchesPerturbationsAcrossLowSpeedBoundary(testCase)
            c=mncavFullObserverConfig();cutoff=c.invariantObserver.predictionMinimumSpeedMps;
            for speed=[0,.2,cutoff,2,12]
                x=[1;speed;.3;2;0;.7;.2;.12];[F,~,~]=localizationSupport.secants(x,x,c);
                numeric=zeros(8);
                for j=1:8
                    h=1e-6*max(1,abs(x(j)));plus=x;minus=x;plus(j)=plus(j)+h;minus(j)=minus(j)-h;
                    numeric(:,j)=(drift(plus,c)-drift(minus,c))/(2*h);
                end
                testCase.verifyLessThan(norm(F-numeric,'fro')/max(1,norm(F,'fro')),1e-7);
                testCase.verifyEqual(F([3,6],8),zeros(2,1),AbsTol=0);
            end
        end
        function coefficientModelMatchesIndependentPhysicalJacobian(testCase)
            c=mncavFullObserverConfig();[~,bounds]=localizationSupport.fusionVertices(c);
            t=c.invariantObserver.theta;ell=c.invariantObserver.headingLengthM;
            T=diag([t,t^2,t^3,t,t^2,t^3,t/ell,t^2/ell]);
            for speed=linspace(c.invariantObserver.fusion.minimumCourseSpeed,c.iss.maximumSpeed,7)
                for q=linspace(-min(c.maximumTrackAngleRate,c.iss.maximumAcceleration/speed), ...
                        min(c.maximumTrackAngleRate,c.iss.maximumAcceleration/speed),9)
                    limit=sqrt(max(0,c.iss.maximumAcceleration^2-(speed*q)^2));
                    for b=linspace(-limit,limit,5)
                        f=localizationSupport.fusionFeatures(speed,q,b,c);
                        testCase.verifyGreaterThanOrEqual(f.',bounds(:,1)-1e-12);
                        testCase.verifyLessThanOrEqual(f.',bounds(:,2)+1e-12);
                        A=localizationSupport.fusionMatrices(c,f);x=[0;speed;b;0;0;speed*q;.2;-.3];
                        F=zeros(8);
                        for j=1:8
                            h=1e-5*max(1,abs(x(j)));p=x;m=x;p(j)=p(j)+h;m(j)=m(j)-h;
                            F(:,j)=(drift(p,c)-drift(m,c))/(2*h);
                        end
                        testCase.verifyLessThan(norm(A-(T\F*T)/t,'fro'),1e-8);
                    end
                end
            end
        end
        function stoppedPredictionIsFiniteAndContinuous(testCase)
            c=mncavFullObserverConfig();x=[0;0;.3;0;0;.7;.2;.1];
            a=localizationSupport.predict(x,.01,c);
            testCase.verifyTrue(all(isfinite(a)));
            testCase.verifyEqual(localizationSupport.courseRate(x,c),0,AbsTol=0);
            x(2)=1e-10;b=localizationSupport.predict(x,.01,c);
            testCase.verifyLessThan(norm(a-b),1e-8);
        end
    end
end
function y=drift(x,c)
    z=x(2)^2+x(5)^2;threshold=c.invariantObserver.predictionMinimumSpeedMps^2;
    denominator=z+threshold/3*max(1-z/threshold,0)^3;
    q=(x(2)*x(6)-x(5)*x(3))/denominator;
    y=[x(2);x(3);q^2*x(2)-2*q*x(6);x(5);x(6);q^2*x(5)+2*q*x(3);x(8);0];
end
