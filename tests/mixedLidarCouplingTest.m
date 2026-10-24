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
            filter=lidarInjectionSupport.filterLidarPoseInformation(information,cfg);
            support=fullObserverSupport.lidarPoseSupport(filter,cfg);
            testCase.verifyEqual(filter.S,S,AbsTol=5e-13);
            testCase.verifyEqual(filter.rank,1);
            testCase.verifyGreaterThan(filter.S(3,3),cfg.iss.minimumHeadingStrength);
            testCase.verifyLessThan(norm(filter.S(1:2,3)),cfg.iss.maximumPositionHeadingCoupling+1e-13);
            testCase.verifyLessThan(min(eig(filter.S-diag([0,0,cfg.iss.minimumHeadingStrength]))),-.04);
            testCase.verifyFalse(support.headingSupported);
        end
        function lowerControlsRetainCouplingWithoutPretendingToBePsd(testCase)
            p=domain();controls=sensorRedundancySupport.mixedLidarLowerControls(p);
            testCase.verifyNumElements(controls,4);
            testCase.verifyEqual(controls{1}(1:2,3),p.maximumLidarCoupling*[1;1],AbsTol=0);
            testCase.verifyEqual(controls{1}(1:2,1:2),zeros(2),AbsTol=0);
            testCase.verifyLessThan(min(eig(controls{1})),0);
        end
        function zeroCouplingRecoversTheValidHeadingOnlyLimit(testCase)
            p=domain();p.maximumLidarCoupling=0;
            controls=sensorRedundancySupport.mixedLidarLowerControls(p);
            testCase.verifyNumElements(controls,1);
            testCase.verifyEqual(controls{1},diag([0,0,p.minimumLidarHeadingStrength]),AbsTol=0);
        end
        function everyAdmittedMatrixDominatesAConvexCombinationOfControls(testCase)
            [minimumDifference,maximumReconstruction,minimumWeight]=enclosureErrors(domain());
            testCase.verifyGreaterThanOrEqual(minimumDifference,-1e-13);
            testCase.verifyLessThan(maximumReconstruction,1e-13);
            testCase.verifyGreaterThanOrEqual(minimumWeight,0);
        end
        function couplingAndHeadingBoundsMustBeFiniteAndNonnegative(testCase)
            p=domain();p.maximumLidarCoupling=-.01;
            testCase.verifyError(@()sensorRedundancySupport.mixedLidarLowerControls(p),'MATLAB:expectedNonnegative');
        end
    end
end

function p=domain()
    c=mncavFullObserverConfig();
    p=struct('maximumLidarCoupling',c.iss.maximumPositionHeadingCoupling, ...
        'minimumLidarHeadingStrength',c.iss.minimumHeadingStrength);
end

function [minimumDifference,maximumReconstruction,minimumWeight]=enclosureErrors(p)
    controls=sensorRedundancySupport.mixedLidarLowerControls(p);
    coefficients=reshape(cat(3,controls{:}),9,[]);
    stream=RandStream('mt19937ar','Seed',2401);
    minimumDifference=Inf;maximumReconstruction=0;minimumWeight=1;
    for k=1:250
        angle=2*pi*rand(stream);c=p.maximumLidarCoupling*sqrt(rand(stream))*[cos(angle);sin(angle)];
        h=p.minimumLidarHeadingStrength+.01*rand(stream);
        T=c*c.'/h+diag(.002*rand(stream,2,1));S=[T,c;c.',h];
        u=(c/p.maximumLidarCoupling+1)/2;
        weights=[u(1)*u(2);u(1)*(1-u(2));(1-u(1))*u(2);(1-u(1))*(1-u(2))];
        lower=reshape(coefficients*weights,3,3);
        difference=S-lower;expected=blkdiag(T,h-p.minimumLidarHeadingStrength);
        minimumDifference=min(minimumDifference,min(eig((difference+difference.')/2)));
        maximumReconstruction=max(maximumReconstruction,norm(difference-expected,2));
        minimumWeight=min(minimumWeight,min(weights));
    end
end
