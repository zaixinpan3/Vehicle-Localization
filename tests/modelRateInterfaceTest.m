classdef modelRateInterfaceTest < matlab.unittest.TestCase
% Test analytic transport and bounded-bandwidth state derivatives.
    methods(TestClassSetup)
        function paths(tc)
            root=fileparts(fileparts(mfilename('fullpath')));
            tc.applyFixture(matlab.unittest.fixtures.PathFixture(root));setupVehicleLocalization;
        end
    end
    methods(Test)
        function analyticLevelingRateMatchesSmoothRigidMotion(tc)
            roll=.2;pitch=-.1;w=[.3;-.2;.15];
            rd=w(1)+tan(pitch)*(sin(roll)*w(2)+cos(roll)*w(3));
            pd=cos(roll)*w(2)-sin(roll)*w(3);
            [R,Rd]=localizationSupport.levelingRotation(roll,pitch,w);
            h=1e-5;
            Rp=localizationSupport.levelingRotation(roll+h*rd,pitch+h*pd,w);
            Rm=localizationSupport.levelingRotation(roll-h*rd,pitch-h*pd,w);
            tc.verifyEqual(Rd,(Rp-Rm)/(2*h),AbsTol=2e-11);
            tc.verifyEqual(Rd*R.'+R*Rd.',zeros(3),AbsTol=1e-14);
        end
        function rigidTransportIncludesTangentialAndCentripetalAcceleration(tc)
            offset=[-2;.5];r=[.2;-.1];rd=[.3;-.2];a=[1,2;3,4];
            actual=localizationSupport.transportPlanarAcceleration(a,r,rd,offset);
            for k=1:2
                w=[0;0;r(k)];alpha=[0;0;rd(k)];d=[offset;0];
                expected=[a(k,:).';0]+cross(alpha,d)+cross(w,cross(w,d));
                tc.verifyEqual(actual(k,:),expected(1:2).',AbsTol=1e-12);
            end
        end
        function signalNoiseRateIsBoundedIndependentlyOfSampling(tc)
            tau=.1;u=[0;.01];
            for dt=[1e-2,1e-5,1e-8]
                [f,rate]=localizationSupport.filterSignalWithRate([0;dt],u,tau);
                tc.verifyEqual(rate,(u-f)/tau,AbsTol=1e-14);
                tc.verifyLessThanOrEqual(abs(rate(2)),.01/tau);
            end
        end
        function filterIsCausalAndConstantSignalsHaveZeroRate(tc)
            t=[0;.03;.2;.4];u=[1,2;1,2;1,2;8,-9];
            [a,ar]=localizationSupport.filterSignalWithRate(t,u,.1);
            [b,br]=localizationSupport.filterSignalWithRate(t(1:3),u(1:3,:),.1);
            tc.verifyEqual(a(1:3,:),b);tc.verifyEqual(ar(1:3,:),br);
            tc.verifyEqual(br,zeros(3,2));
        end
    end
end
