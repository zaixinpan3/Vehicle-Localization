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
            c=[cfg.iss.maximumPositionHeadingCoupling;0];S=[c*c.'/h,c;c.',h];
            testCase.verifyGreaterThanOrEqual(min(eig(S)),-1e-14);
            testCase.verifyLessThan(max(eig(S)),1);
            testCase.verifyEqual(S(3,3),h,AbsTol=0);
            testCase.verifyLessThanOrEqual(norm(S(1:2,3)),cfg.iss.maximumPositionHeadingCoupling);
            testCase.verifyLessThan(min(eig(S-diag([0,0,h]))),-.05);
        end
        function positiveTranslationInformationDoesNotRepairThatImplication(testCase)
            cfg=mncavFullObserverConfig();h=cfg.iss.minimumHeadingStrength;
            c=[cfg.iss.maximumPositionHeadingCoupling;0];
            S=[cfg.iss.minimumPositionStrength*eye(2),c;c.',h];
            testCase.verifyGreaterThan(min(eig(S)),0);
            testCase.verifyLessThan(max(eig(S)),1);
            testCase.verifyLessThan(min(eig(S-diag([0,0,h]))),-.01);
        end
        function productionFilterCanReturnAnAdmissibleCoupledCounterexample(testCase)
            cfg=mncavFullObserverConfig();h=(1+cfg.iss.minimumHeadingStrength)/2;
            c=[cfg.iss.maximumPositionHeadingCoupling;0];S=[c*c.'/h,c;c.',h];
            D=diag(cfg.lidar.poseScales);H=cfg.lidar.gainInformationScale*S/(eye(3)-S);
            information=D\((H+H.')/2)/D;
            filter=localizationSupport.filterLidarPoseInformation(information,cfg);
            support=localizationSupport.lidarPoseSupport(filter,cfg);
            testCase.verifyEqual(filter.S,S,AbsTol=5e-13);
            testCase.verifyEqual(filter.rank,1);
            testCase.verifyGreaterThan(filter.S(3,3),cfg.iss.minimumHeadingStrength);
            testCase.verifyLessThan(norm(filter.S(1:2,3)),cfg.iss.maximumPositionHeadingCoupling+1e-13);
            testCase.verifyLessThan(min(eig(filter.S-diag([0,0,cfg.iss.minimumHeadingStrength]))),-.04);
            testCase.verifyFalse(support.headingSupported);
        end
    end
end
