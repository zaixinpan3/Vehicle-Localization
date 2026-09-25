function tests=pillarRadialDistributionTest
% pillarRadialDistributionTest: Whole-pillar radial mass and neighborhood edges.
    tests=functiontests(localfunctions);
end
function setupOnce(~)
    setupVehicleLocalization();
end
function testCircularSupportIncludesNeighborAndHasMonotoneMass(testCase)
    g=struct('origin',[0 0],'cellSize',[.6 .6],'mapSize',[3 3]);
    p=[.59 .3 0;.61 .3 1;.85 .3 2;1.2 .3 3]; ids=[1;4;4;7];
    peaks=[.6 .3;.8 .3;1.2 .3]; d=computePillarRadialDistribution(p,ids,g,peaks,[true;false;false]);
    verifyEqual(testCase,d.count(1,d.radii==.1),2);
    verifyEqual(testCase,d.ownCount(1,d.radii==.1),1);
    verifyEqual(testCase,d.height(1,d.radii==.1),1);
    verifyTrue(testCase,all(diff(d.count(1,:))>=0));
    verifyEqual(testCase,d.count(2:3,:),zeros(2,9));
end
function testNoLinearIndexWrapAtMapEdges(testCase)
    g=struct('origin',[0 0],'cellSize',[.6 .6],'mapSize',[4 4]);
    p=[.3 2.1 0;.3 2.1 2;.9 .3 5]; ids=[4;4;5];
    d=computePillarRadialDistribution(p,ids,g,[.3 2.1;.9 .3]);
    verifyEqual(testCase,d.count(1,:),2*ones(1,9));
    verifyEqual(testCase,d.height(1,:),2*ones(1,9));
end
function testVerticalShaftAndVerticalSheetHaveDifferentMassGrowth(testCase)
    z=linspace(0,3,31).'; pole=[repmat([1.5 1.5],31,1),z];
    [x,zz]=meshgrid(.7:.025:2.3,z); sheet=[x(:),repmat(1.5,numel(x),1),zz(:)];
    g=struct('origin',[0 0],'cellSize',[.6 .6],'mapSize',[5 5]);
    p=measure(pole,g); s=measure(sheet,g);
    verifyEqual(testCase,p.count(1,:),31*ones(1,9));
    verifyGreaterThan(testCase,s.count(1,end)/s.count(1,2),4);
    verifyLessThan(testCase,p.radialStd(1,end),1e-10);
    verifyGreaterThan(testCase,s.xyLinearity(1,end),.99);
end
function d=measure(points,g)
    bins=floor(points(:,1:2)./g.cellSize)+1; ids=sub2ind(g.mapSize,bins(:,2),bins(:,1));
    occupied=unique(ids); peaks=repmat([1.5 1.5],numel(occupied),1);
    d=computePillarRadialDistribution(points,ids,g,peaks);
end
function testTranslationAndPointOrderInvariance(testCase)
    g=struct('origin',[0 0],'cellSize',[.6 .6],'mapSize',[3 3]);
    p=[.39 .4 0;.48 .4 1;.61 .4 2;.74 .4 3]; ids=[1;1;4;4]; peaks=[.51 .4;.63 .4];
    a=computePillarRadialDistribution(p,ids,g,peaks);
    shifted=g; shifted.origin=[12 -2];
    b=computePillarRadialDistribution(p(end:-1:1,:)+[12 -2 7],ids(end:-1:1),shifted,peaks+[12 -2]);
    for name={'heightStd','tilt','radialStd','xyLinearity','zSkewness','zKurtosis'}
        verifyEqual(testCase,a.(name{1}),b.(name{1}),'AbsTol',5e-9);
    end
end
function testEmptyInput(testCase)
    g=struct('origin',[0 0],'cellSize',[.6 .6],'mapSize',[3 3]);
    d=computePillarRadialDistribution(zeros(0,3),zeros(0,1),g,zeros(0,2));
    verifyEqual(testCase,size(d.count),[0 9]);
end
