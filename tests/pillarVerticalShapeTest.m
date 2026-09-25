function tests=pillarVerticalShapeTest
% pillarVerticalShapeTest: Height distribution invariants and gap rejection.
    tests=functiontests(localfunctions);
end
function setupOnce(testCase)
    testCase.TestData.root=setupVehicleLocalization();
end
function testContinuousAndDisconnectedSupports(testCase)
    continuous=linspace(0,3,61).'; disconnected=[linspace(0,.2,30),linspace(2.8,3,31)].';
    xyz=[zeros(122,2),[continuous;disconnected]]; ids=[ones(61,1);2*ones(61,1)];
    s=computePillarVerticalShape(xyz,ids);
    verifyLessThan(testCase,s.gapRatio(1),0.02);
    verifyGreaterThan(testCase,s.gapRatio(2),0.8);
    verifyEqual(testCase,s.pillarIndices,[1;2]);
end
function testSingleOutlierDoesNotSupplyInterquartileHeight(testCase)
    z=[linspace(0,.1,40),4].'; s=computePillarVerticalShape([zeros(41,2),z],ones(41,1));
    verifyGreaterThan(testCase,s.gapRatio,0.9);
    verifyLessThan(testCase,s.interquartileRatio,0.02);
    verifyGreaterThan(testCase,s.kurtosis,20);
end
function testPermutationAndVerticalAffineInvariance(testCase)
    z=[0 .03 .1 .2 .8 1.2 2 3].'; p=[zeros(8,2),z]; ids=ones(8,1);
    a=computePillarVerticalShape(p,ids);
    b=computePillarVerticalShape(p(end:-1:1,:).*[1 1 2]+[4 -3 7],ids);
    for name={'gapRatio','interquartileRatio','skewness','kurtosis'}
        verifyEqual(testCase,a.(name{1}),b.(name{1}),'AbsTol',1e-12);
    end
    verifyEqual(testCase,2*a.lowerHalfHeight,b.lowerHalfHeight,'AbsTol',1e-12);
    verifyEqual(testCase,2*a.upperHalfHeight,b.upperHalfHeight,'AbsTol',1e-12);
end
function testEmptyAndConstantHeight(testCase)
    empty=computePillarVerticalShape(zeros(0,3),zeros(0,1)); verifyEmpty(testCase,empty.pillarIndices);
    s=computePillarVerticalShape([0 0 2;1 1 2],[7;7]);
    for name=setdiff(fieldnames(s),{'pillarIndices'}).', verifyEqual(testCase,s.(name{1}),0); end
end
function testEvaluationMaskAndValidation(testCase)
    p=[zeros(6,2),(0:5).']; ids=[1;1;1;9;9;9];
    s=computePillarVerticalShape(p,ids,[false;true]); verifyEqual(testCase,s.gapRatio,[0;.5]);
    verifyError(testCase,@()computePillarVerticalShape(p,ids,true),'perception:InvalidPillarShape');
    verifyError(testCase,@()computePillarVerticalShape([NaN 0 0],1),'perception:InvalidPillarShape');
end
