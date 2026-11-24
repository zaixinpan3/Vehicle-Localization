classdef mixedLidarCouplingTest < matlab.unittest.TestCase
% Matrix-order regression for complementary GNSS/LiDAR information.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));setupVehicleLocalization;
        end
    end
    methods (Test)
        function psdHeadingAndCouplingBoundsDoNotImplyAHeadingMatrixFloor(testCase)
            cfg=mncavFullObserverConfig();h=cfg.iss.minimumHeadingStrength;
            c=[.06;0];S=[c*c.'/h,c;c.',h];
            testCase.verifyGreaterThanOrEqual(min(eig(S)),-1e-14);
            testCase.verifyLessThan(max(eig(S)),1);
            testCase.verifyEqual(S(3,3),h,AbsTol=0);
            testCase.verifyLessThanOrEqual(norm(S(1:2,3)),.06);
            testCase.verifyLessThan(min(eig(S-diag([0,0,h]))),-.05);
        end
        function positiveTranslationInformationDoesNotRepairThatImplication(testCase)
            cfg=mncavFullObserverConfig();h=cfg.iss.minimumHeadingStrength;
            c=[.06;0];
            S=[cfg.iss.minimumPositionStrength*eye(2),c;c.',h];
            testCase.verifyGreaterThan(min(eig(S)),0);
            testCase.verifyLessThan(max(eig(S)),1);
            testCase.verifyLessThan(min(eig(S-diag([0,0,h]))),-.01);
        end
        function coupledGramCanLeaveHeadingUnobservable(testCase)
            cfg=mncavFullObserverConfig();h=(1+cfg.iss.minimumHeadingStrength)/2;
            c=[.06;0];S=[c*c.'/h,c;c.',h];
            gram=localizationSupport.decomposeLidarGram(S);
            support=localizationSupport.lidarPoseSupport(gram,cfg);
            testCase.verifyEqual(gram.matrix,S,AbsTol=5e-13);
            testCase.verifyEqual(gram.rank,1);
            testCase.verifyGreaterThan(gram.matrix(3,3),cfg.iss.minimumHeadingStrength);
            testCase.verifyLessThan(norm(gram.matrix(1:2,3)),.06+1e-13);
            testCase.verifyLessThan(min(eig(gram.matrix-diag([0,0,cfg.iss.minimumHeadingStrength]))),-.04);
            testCase.verifyFalse(support.headingSupported);
        end
    end
end
