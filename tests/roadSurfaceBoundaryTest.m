classdef roadSurfaceBoundaryTest < matlab.unittest.TestCase
% roadSurfaceBoundaryTest: Preserve road growth when curb sides are missing.
    methods (TestClassSetup)
        function paths(~)
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function lowerSideAloneRetainsUnboundedRoadGrowth(testCase)
            verifyIncompleteBoundary(testCase,1);
        end
        function upperSideAloneRetainsUnboundedRoadGrowth(testCase)
            verifyIncompleteBoundary(testCase,7);
        end
        function centerlineCurbsDoNotInventSideBoundaries(testCase)
            verifyIncompleteBoundary(testCase,4);
        end
        function twoSidesConstrainRoadGrowth(testCase)
            [stats,energy,xy,cfg]=flatRoad([2 6]);
            actual=extractRoadSurface(stats,energy,xy,cfg);
            expected=false(7,5);expected(3:5,:)=true;
            testCase.verifyEqual(actual.curbInteriorMask,expected);
            testCase.verifyEqual(actual.roadCellMask,expected);
        end
    end
end

function verifyIncompleteBoundary(testCase,row)
    [stats,energy,xy,cfg]=flatRoad(row);
    actual=extractRoadSurface(stats,energy,xy,cfg);
    cfg.curbInteriorConstraintEnabled=false;
    unconstrained=extractRoadSurface(stats,energy,xy,cfg);
    testCase.verifyTrue(all(actual.curbInteriorMask(:)));
    testCase.verifyEqual(actual.roadCellMask,unconstrained.roadCellMask);
    testCase.verifyEqual(actual.candidateRoadMask,~energy.extractedMask);
    testCase.verifyGreaterThan(actual.numRoadCells,0);
end

function [stats,energy,xy,cfg]=flatRoad(rows)
    [x,y]=meshgrid(3:7,-3:3);
    stats=struct('countMap',ones(7,5),'heightMap',zeros(7,5), ...
        'roughnessMap',zeros(7,5),'supportMask',true(7,5));
    curb=false(7,5);curb(rows,:)=true;
    energy=struct('total',zeros(7,5),'extractedMask',curb);
    xy=struct('occupiedMask',true(7,5),'xMap',x,'yMap',y, ...
        'yCenters',(-3:3).','cellSize',[1 1]);
    full=groundFeatureConfig();cfg=full.road;
    cfg.curbBarrierRadiusCells=0;cfg.curbInteriorMinBoundaryCells=2;
    cfg.curbInteriorClearanceCells=1;cfg.seedAbsYMaxMeters=1;
    cfg.minRoadCells=1;
end
