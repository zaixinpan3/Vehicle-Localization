classdef pillarShaftModesTest < matlab.unittest.TestCase
% pillarShaftModesTest: Minority shafts, bounded support and independent owners.
    properties (TestParameter)
        clutterSeed={3,14,17,41};
    end
    methods (TestClassSetup)
        function paths(~)
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function minorityShaftSurvivesEndClutter(testCase)
            shaft=pillarShaftModesTest.shaft([.12 .16]);rng(14);
            clutter=[.04+.52*rand(800,2),3.6+.5*rand(800,1)];
            [e,assigned]=pillarShaftModesTest.measure([shaft;clutter]);
            testCase.verifyTrue(e.found);testCase.verifyTrue(assigned.found);
            testCase.verifyLessThan(assigned.ownCount,100);
            testCase.verifyLessThan(norm(assigned.axisXY-[.12 .16]),.06);
        end
        function denseShortModeDoesNotHideSecondShaft(testCase)
            shaft=pillarShaftModesTest.shaft([.12 .16]);rng(17);
            blob=[.43+.015*randn(500,1),.42+.015*randn(500,1),.2+.2*rand(500,1)];
            e=pillarShaftModesTest.measure([shaft;blob]);
            testCase.verifyTrue(e.found);testCase.verifyLessThan(norm(e.axisXY-[.12 .16]),.06);
        end
        function neighboringDistinctShaftsDoNotCompete(testCase)
            p=[pillarShaftModesTest.shaft([.45 .3]);pillarShaftModesTest.shaft([.85 .3])];
            [~,a,g]=pillarShaftModesTest.measure(p,[1 2]);
            testCase.verifyTrue(all(a.found));testCase.verifyEqual(numel(g.representativeRows),2);
            testCase.verifyGreaterThan(norm(diff(a.axisXY)),.3);
        end
        function shaftAcrossBoundaryKeepsBothSupportedOwners(testCase)
            p=pillarShaftModesTest.shaft([.6 .3]);
            [~,a,g]=pillarShaftModesTest.measure(p,[1 2]);
            testCase.verifyTrue(all(a.found));testCase.verifyEqual(numel(unique(g.ownerGroup)),1);
            testCase.verifyGreaterThanOrEqual(a.ownCount,3*ones(2,1));
        end
        function shortNeighborCannotBorrowAShaft(testCase)
            shaft=pillarShaftModesTest.shaft([.72 .16]);rng(5);
            blob=[.56+.005*randn(40,1),.16+.005*randn(40,1),.15*rand(40,1)];
            [~,a]=pillarShaftModesTest.measure([shaft;blob],[1 2]);
            testCase.verifyFalse(a.found(1));testCase.verifyTrue(a.found(2));
        end
        function broadObliqueSheetIsRejected(testCase)
            [x,z]=meshgrid(.02:.012:.58,-1:.08:3);p=[x(:),.15+.45*x(:),z(:)];
            e=pillarShaftModesTest.measure(p);
            testCase.verifyFalse(any(e.found));
        end
        function separatedBlobsAreNotContinuous(testCase)
            rng(3);p=[.2+.01*randn(80,2),[.1*rand(40,1);3+.1*rand(40,1)]];
            e=pillarShaftModesTest.measure(p);
            testCase.verifyFalse(any(e.found));
        end
        function boundedShaftSurvivesContiguousWideBase(testCase)
            shaft=pillarShaftModesTest.shaft([.3 .3]);
            [x,z]=meshgrid(.02:.02:.58,-2:.08:-1);base=[x(:),.3*ones(numel(x),1),z(:)];
            e=pillarShaftModesTest.measure([shaft;base]);
            testCase.verifyTrue(e.found);testCase.verifyGreaterThan(e.minimumZ,-1.3);
            testCase.verifyGreaterThan(e.robustHeight,1.5);
        end
        function loweringPeakGateCannotRemoveExistingBoundedSupport(testCase)
            p=pillarShaftModesTest.shaft([.3 .3]);cfg=pillarShaftConfig();cfg.minimumPeakContrast=1.4;
            geometry=struct('origin',[0 0],'cellSize',[.6 .6],'mapSize',[1 1]);
            a=findPillarShaftModes(p,ones(size(p,1),1),geometry,cfg);
            cfg.minimumPeakContrast=1.2;b=findPillarShaftModes(p,ones(size(p,1),1),geometry,cfg);
            testCase.verifyTrue(a.found);testCase.verifyTrue(b.found);
            testCase.verifyGreaterThanOrEqual(b.score,a.score-1e-10);
        end
        function nativeMatchesMatlab(testCase)
            testCase.assumeTrue(perceptionNativeAvailable);
            p=pillarShaftModesTest.shaft([.3 .3]);cfg=pillarShaftConfig();
            geometry=struct('origin',[0 0],'cellSize',[.6 .6],'mapSize',[1 1]);ids=ones(size(p,1),1);
            a=findPillarShaftModes(p,ids,geometry,cfg);cfg.useNativeKernels=true;
            b=findPillarShaftModes(p,ids,geometry,cfg);
            testCase.verifyEqual(b,a,'AbsTol',1e-9);
        end
        function emptyOwnersReturnEmptyEvidence(testCase)
            geometry=struct('origin',[0 0],'cellSize',[.6 .6],'mapSize',[1 1]);
            e=findPillarShaftModes(zeros(0,3),zeros(0,1),geometry,pillarShaftConfig());
            testCase.verifyEmpty(e.found);testCase.verifySize(e.axisXY,[0 2]);
        end
        function uniformVolumeDoesNotBecomeAPole(testCase,clutterSeed)
            rng(clutterSeed);p=[.6*rand(800,2),-1+4*rand(800,1)];
            e=pillarShaftModesTest.measure(p);
            testCase.verifyFalse(e.found);
        end
        function leaningShaftRetainsItsIndependentFit(testCase)
            p=pillarShaftModesTest.shaft([.15 .3]);p(:,1)=p(:,1)+tand(12)*(p(:,3)+1);
            [~,a]=pillarShaftModesTest.measure(p,[1 2]);
            testCase.verifyTrue(all(a.found));
            testCase.verifyGreaterThan(atand(norm(a.slopeXY(1,:))),8);
        end
    end
    methods (Static,Access=private)
        function p=shaft(center)
            z=linspace(-1,3,60).';angle=(1:60).'*2.399;
            p=[center(1)+.025*cos(angle),center(2)+.025*sin(angle),z];
        end
        function [e,a,g]=measure(p,mapSize)
            if nargin<2,mapSize=[1 1];end
            geometry=struct('origin',[0 0],'cellSize',[.6 .6],'mapSize',mapSize);
            bins=floor(p(:,1:2)./.6)+1;ids=sub2ind(mapSize,bins(:,2),bins(:,1));cfg=pillarShaftConfig();
            cfg.useNativeKernels=perceptionNativeAvailable;
            e=findPillarShaftModes(p,ids,geometry,cfg);
            [a,g]=assignPillarShaftSupport(p,ids,geometry,e,cfg);
        end
    end
end
