classdef fourInvariantOutputsTest < matlab.unittest.TestCase
% Signed body projections, output rank and independent packet availability.
    properties (TestParameter)
        localAngle={0,.3,1.4}
    end
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));setupVehicleLocalization;
        end
    end
    methods (Test)
        function exactlyTheFourRequestedOutputsAreReturned(testCase)
            c=mncavFullObserverConfig();x=[1;3;1;2;4;-2;.2;.3];
            [h,H]=localizationSupport.outputs(x,c);
            rotation=exp(-1i*.2);
            velocity=rotation*(3+4i);acceleration=rotation*(1-2i);
            expected=[real(velocity);imag(velocity);real(acceleration);imag(acceleration)];
            testCase.verifyEqual(h,expected,AbsTol=1e-13);
            testCase.verifySize(H,[4,8]);
            testCase.verifyEqual(H(:,[1,4]),zeros(4,2),AbsTol=0);
        end
        function changingRateDoesNotChangeAdditionalOutputs(testCase)
            c=mncavFullObserverConfig();x=[0;12;.6;0;0;1.2;-.1;.03];
            y=x;y(8)=y(8)+.2;
            a=localizationSupport.outputs(x,c);b=localizationSupport.outputs(y,c);
            testCase.verifyEqual(b,a,AbsTol=0);
        end
        function localQuotientCoordinatesAreRecovered(testCase,localAngle)
            c=mncavFullObserverConfig();speed=12;parallel=.6;normal=1.2;rate=.03;
            x=[0;speed;parallel;0;0;normal;-localAngle;rate];
            h=localizationSupport.outputs(x,c);
            bodyVelocity=complex(h(1),h(2));bodyAcceleration=complex(h(3),h(4));
            testCase.verifyEqual(bodyVelocity,speed*exp(1i*localAngle),AbsTol=2e-14);
            testCase.verifyEqual(bodyAcceleration,(parallel+1i*normal)*exp(1i*localAngle),AbsTol=2e-14);
        end
        function jacobianHasFourIndependentRowsAndTheRotationNullDirection(testCase)
            c=mncavFullObserverConfig();x=[1;12;.6;2;1;1.2;-.1;.03];
            [~,H]=localizationSupport.outputs(x,c);
            orbit=[0;-x(5);-x(6);0;x(2);x(3);1;0];
            testCase.verifyGreaterThan(min(svd(H)),.1);
            testCase.verifyEqual(H*orbit,zeros(4,1),AbsTol=2e-14);
        end
        function orthogonalHeadingRetainsFullRank(testCase)
            c=mncavFullObserverConfig();x=[0;12;.6;0;0;1.2;-pi/2;.03];
            [~,H]=localizationSupport.outputs(x,c);
            testCase.verifyEqual(rank(H),4);
        end
        function restRetainsFourCoordinates(testCase)
            c=mncavFullObserverConfig();x=zeros(8,1);
            [h,H]=localizationSupport.outputs(x,c);
            testCase.verifyTrue(all(isfinite([h;H(:)])));
            testCase.verifyEqual(rank(H),4);
        end
        function accelerationIsAvailableWithoutVelocity(testCase)
            c=mncavFullObserverConfig();
            p=localizationSupport.measurements(NaN,NaN,20,-30,false,true,c);
            testCase.verifyEqual(p.valid,logical([0;0;1;1]));
            testCase.verifyEqual(p.value,[0;0;20;-30],AbsTol=0);
            testCase.verifyFalse(isfield(p,'yawRate'));
        end
        function missingAccelerationKeepsSignedVelocity(testCase)
            c=mncavFullObserverConfig();
            p=localizationSupport.measurements(12,1,NaN,NaN,true,false,c);
            testCase.verifyEqual(p.valid,logical([1;1;0;0]));
            testCase.verifyTrue(all(isfinite(p.value)));
        end
        function accelerationAloneReducesTheBodyResidual(testCase)
            c=mncavFullObserverConfig();d=correctionDesign();x=zeros(8,1);
            p=localizationSupport.measurements(NaN,NaN,20,-30,false,true,c);
            y=localizationSupport.correct(x,p,.1,d,c);
            before=localizationSupport.outputs(x,c); after=localizationSupport.outputs(y,c);
            testCase.verifyLessThan(norm(after(3:4)-p.value(3:4)),norm(before(3:4)-p.value(3:4)));
        end
        function motionInnovationCorrectsRateThroughTheHeadingMetric(testCase)
            c=mncavFullObserverConfig();d=correctionDesign();
            x=[0;12;0;0;0;0;.1;.03];
            p=localizationSupport.measurements(12,0,0,0,true,true,c);
            p.valid=logical([0;1;0;0]);
            uncoupled=localizationSupport.correct(x,p,.1,d,c);
            d.P(7:8,7:8)=[2,-1;-1,2];
            coupled=localizationSupport.correct(x,p,.1,d,c);
            testCase.verifyEqual(uncoupled(8),x(8),AbsTol=0);
            testCase.verifyLessThan(coupled(7),x(7));
            testCase.verifyLessThan(coupled(8),x(8));
            p.valid(:)=false;
            withdrawn=localizationSupport.correct(x,p,.1,d,c);
            testCase.verifyEqual(withdrawn,x,AbsTol=0);
        end
    end
end

function d=correctionDesign()
    d=struct('theta',1,'P',eye(8),'toPhysical',eye(8));
end
