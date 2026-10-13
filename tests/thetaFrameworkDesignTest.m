classdef thetaFrameworkDesignTest < matlab.unittest.TestCase
% thetaFrameworkDesignTest The MnCAV observer follows the source paper's theta framework.
% K and N come from LMI (55); L = T(theta) K and M = T(theta) N / theta^3.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));
            external=fullfile(fileparts(root),'RobustVehicleLocalization','external');
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(external,'YALMIP'),IncludingSubfolders=true));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(external,'sedumi'),IncludingSubfolders=true));
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function storedDesignReproducesTheSynthesis(testCase)
            cfg=measuredCourseObserverConfig();fresh=fullObserverSupport.synthesizeThetaFrameworkDesign(measuredCourseObserverConfig(LoadDesign=false));
            a=fullObserverSupport.designFullObserverGains(cfg);c=cfg;c.hgo=struct('theta',cfg.hgo.theta,'sigma',fresh.sigma, ...
                'ell',cfg.hgo.ell,'P',fresh.P,'kappaL',fresh.kappaL,'kappaG',fresh.kappaG,'Z',fresh.Z,'lambda',fresh.lambda);
            b=fullObserverSupport.designFullObserverGains(c);
            testCase.verifyEqual(a.hgo.L,b.hgo.L,RelTol=1e-3,AbsTol=1e-6);
            testCase.verifyEqual(a.hgo.LG,b.hgo.LG,RelTol=1e-3,AbsTol=1e-6);
            testCase.verifyEqual(a.hgo.M,b.hgo.M,AbsTol=1e-6);
            testCase.verifyEqual(string(cfg.gainSource),"config/mncavThetaDesign.json");
        end
        function storedDesignIsCertifiedAtEveryVertex(testCase)
            cfg=measuredCourseObserverConfig();v=fullObserverSupport.verifyThetaFrameworkDesign(cfg.hgo,cfg);
            testCase.verifyTrue(v.certified);
            testCase.verifyLessThan(v.worstVertexEigenvalue,0);
            testCase.verifyEqual(v.vertices,768);
            testCase.verifyEqual(v.lipschitzConstant,0);
        end
        function gainsFollowTheThetaScalingOfThePaper(testCase)
            cfg=measuredCourseObserverConfig();d=fullObserverSupport.designFullObserverGains(cfg);h=cfg.hgo;
            Cp=zeros(2,7);Cp(1,1)=1;Cp(2,4)=1;C=[Cp;[zeros(1,6),1]];
            T=diag(h.theta.^[1 2 3 1 2 3 1]);E=diag([ones(1,6),1/h.ell]);Ey=diag([1 1 1/h.ell]);
            testCase.verifyEqual(d.hgo.L,E*T*h.kappaL*(h.P\C.')/Ey,RelTol=1e-12);
            testCase.verifyEqual(d.hgo.M,E*T*(h.P\h.Z.')/h.theta^3,RelTol=1e-12,AbsTol=1e-15);
            testCase.verifyEqual(d.lidarMatched.T,T,AbsTol=0);
            testCase.verifyEqual(d.lidarMatched.kappa,h.kappaL/h.theta,RelTol=1e-15);
            testCase.verifyEqual(d.hgo.LG(7,:),[0,0]);testCase.verifyEqual(d.hgo.M(7,:),zeros(1,4));
            doubled=cfg;doubled.hgo.theta=2*h.theta;e=fullObserverSupport.designFullObserverGains(doubled);
            testCase.verifyEqual(e.hgo.L(1:3,1)./d.hgo.L(1:3,1),2.^[1;2;3],RelTol=1e-12);
            testCase.verifyEqual(e.hgo.L(7,3)/d.hgo.L(7,3),2,RelTol=1e-12);
        end
        function mainOutputAloneStabilizesTheChains(testCase)
            cfg=measuredCourseObserverConfig();h=cfg.hgo;
            A=zeros(7);A(1,2)=1;A(2,3)=1;A(4,5)=1;A(5,6)=1;Cp=zeros(2,7);Cp(1,1)=1;Cp(2,4)=1;C=[Cp;[zeros(1,6),1]];
            K=h.kappaL*(h.P\C.');poles=eig(h.theta*(A-K*C));
            testCase.verifyLessThan(max(real(poles)),0);
            testCase.verifyGreaterThan(abs(K(2,1)),0);testCase.verifyGreaterThan(abs(K(3,1)),0);
        end
        function extraOutputsCarryVelocityThroughOutages(testCase)
            % Without LiDAR and GNSS the velocity and acceleration errors must
            % decay on the rotated body velocity and acceleration alone (the
            % dead-reckoning operating point of the design).
            cfg=measuredCourseObserverConfig();d=fullObserverSupport.designFullObserverGains(cfg);
            A=zeros(6);A(1,2)=1;A(2,3)=1;A(4,5)=1;A(5,6)=1;H=zeros(4,6);H(1,2)=1;H(2,5)=1;H(3,3)=1;H(4,6)=1;
            closed=A-d.hgo.M(1:6,:)*H;va=[2,3,5,6];
            testCase.verifyGreaterThan(d.hgo.M(2,1),0);
            testCase.verifyLessThan(max(real(eig(closed(va,va)))),0);
        end
        function thetaBelowSigmaIsRejected(testCase)
            cfg=measuredCourseObserverConfig(Theta=.5);
            testCase.verifyError(@()fullObserverSupport.designFullObserverGains(cfg),'VehicleLocalization:InvalidThetaDesign');
        end
        function staleDesignIsRejected(testCase)
            cfg=measuredCourseObserverConfig(LoadDesign=false);
            testCase.verifyFalse(isfield(cfg,'hgo'));
            testCase.verifyError(@()measuredCourseObserverConfig(DesignOverride=struct('headingLengthM',20)), ...
                'VehicleLocalization:StaleThetaDesign');
        end
        function corruptedDesignFailsTheVertexCheck(testCase)
            cfg=measuredCourseObserverConfig();h=cfg.hgo;h.kappaL=1e-3*h.kappaL;
            v=fullObserverSupport.verifyThetaFrameworkDesign(h,cfg);
            testCase.verifyFalse(v.certified);
        end
    end
end
