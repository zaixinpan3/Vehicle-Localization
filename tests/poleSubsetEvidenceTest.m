function tests=poleSubsetEvidenceTest
% poleSubsetEvidenceTest: Existence of a shaft amid majority non-pole returns.
    tests=functiontests(localfunctions);
end
function setupOnce(~)
    setupVehicleLocalization();
end
function testMinorityShaftSurvivesDenseEndClutter(testCase)
    shaft=makeShaft(); rng(14);
    clutter=[.04+.52*rand(800,2),3.6+.5*rand(800,1)];
    pure=measure(shaft); mixed=measure([shaft;clutter]);
    verifyTrue(testCase,pure.found); verifyTrue(testCase,mixed.found);
    verifyLessThan(testCase,size(shaft,1)/(size(shaft,1)+size(clutter,1)),.08);
    verifyGreaterThan(testCase,mixed.robustHeight,1.5);
    verifyGreaterThan(testCase,mixed.score,.5*pure.score);
end
function testSecondModeIsShaftWhenDensestModeIsShort(testCase)
    shaft=makeShaft(); rng(17);
    blob=[.43+.015*randn(500,1),.42+.015*randn(500,1),.2+.2*rand(500,1)];
    mixed=measure([shaft;blob]); only=measure(blob);
    verifyTrue(testCase,mixed.found);verifyFalse(testCase,only.found);
    verifyLessThan(testCase,norm(mixed.axisXY-[.12 .16]),.1);
end
function testHorizontalSlabDoesNotCreateShaft(testCase)
    [x,y]=meshgrid(.02:.025:.58); points=[x(:),y(:),zeros(numel(x),1)];
    e=measure(points);verifyFalse(testCase,e.found);
end
function testBroadVerticalSheetHasNoCompactDensityMode(testCase)
    [x,z]=meshgrid(.01:.01:.59,-1:.04:3); points=[x(:),.3*ones(numel(x),1),z(:)];
    e=measure(points);verifyFalse(testCase,e.found);
end
function testDisconnectedBlobsAndOutliersAreInsufficient(testCase)
    rng(3); xy=.12+.015*randn(80,2); z=[.1*rand(40,1);3+.1*rand(40,1)];
    e=measure([xy,z]);verifyFalse(testCase,e.found);
end
function testTiltedBoundaryShaftHasOwnSupportInBothPillars(testCase)
    z=linspace(-1,3,100).'; p=[.45+.12*(z+1),.3+.02*sin((1:100).'),z];
    g=struct('origin',[0 0],'cellSize',[.6 .6],'mapSize',[2 2]); e=measure(p,g);
    verifyTrue(testCase,all(e.found));verifyGreaterThanOrEqual(testCase,e.ownCount,6*ones(size(e.ownCount)));
end
function testNeighborShaftDoesNotLabelEmptyOwnerSupport(testCase)
    shaft=makeShaft()+[.6 0 0]; rng(5); blob=[.48+.01*randn(40,2),.2*rand(40,1)];
    g=struct('origin',[0 0],'cellSize',[.6 .6],'mapSize',[2 2]); e=measure([shaft;blob],g);
    verifyFalse(testCase,e.found(1));verifyTrue(testCase,e.found(2));
end
function testPermutationAndTranslationInvariance(testCase)
    p=makeShaft();a=measure(p);g=struct('origin',[10 -4],'cellSize',[.6 .6],'mapSize',[1 1]);
    b=measure(p(end:-1:1,:)+[10 -4 2],g);
    verifyEqual(testCase,a.found,b.found);verifyEqual(testCase,a.score,b.score,'AbsTol',1e-9);
end
function testNativeMatchesMinoritySupportAndSheetRejection(testCase)
    assumeTrue(testCase,perceptionNativeAvailable);
    shaft=makeShaft(); rng(14); clutter=[.04+.52*rand(800,2),3.6+.5*rand(800,1)];
    [x,z]=meshgrid(.01:.01:.59,-1:.04:3); sheet=[x(:),.3*ones(numel(x),1),z(:)];
    g=struct('origin',[0 0],'cellSize',[.6 .6],'mapSize',[1 1]);
    for points={shaft,[shaft;clutter],sheet,zeros(0,3)}
        p=points{1}; cfg=structuralPillarConfig(.6); ids=ones(size(p,1),1);
        a=findPillarPoleSubsets(p,ids,g,cfg.pole.subset); cfg.pole.subset.useNativeKernels=true;
        b=findPillarPoleSubsets(p,ids,g,cfg.pole.subset);
        verifyEqual(testCase,b,a,'AbsTol',1e-9);
    end
end
function testSubsetDetectionKeepsAllReturnsInOutputMoments(testCase)
    shaft=makeShaft(); rng(14); clutter=[.04+.52*rand(800,2),3.6+.5*rand(800,1)];p=[shaft;clutter];
    g=struct('origin',[0 0],'cellSize',[.6 .6],'mapSize',[1 1]);
    grid=struct('points',p,'pointPillarLinIdx',ones(size(p,1),1),'pointAttributes',struct(),'pillarGeometry',g);
    c=structuralPillarConfig(.6);c.pole.detector="subset";c.pole.probabilityEvidence="subset";
    cloud=coarseSemanticProbabilityCloudConfig();cloud.semanticNames="pole";
    r=analyzeStructuralPillars(grid,c,cloud);
    verifyTrue(testCase,r.poleCellMask);verifyEqual(testCase,r.columnMaps.statistics.count,size(p,1));
    verifyEqual(testCase,r.columnMaps.statistics.meanXYZ,mean(p,1),'AbsTol',1e-12);
    verifyLessThan(testCase,r.columnMaps.poleSubset.ownCount,size(p,1));
end
function testRecordedSubsetBackendParity(testCase)
    assumeTrue(testCase,perceptionNativeAvailable);
    file=fullfile(fileparts(fileparts(mfilename('fullpath'))),'data','raw','MissisipiPointClouds.mat');
    assumeTrue(testCase,isfile(file));
    for index=[1 121 891]
        frame=loadPointCloudFrame(file,index);c=perceptionConfig();
        c.offGroundFeatures.pole.detector="subset";c.offGroundFeatures.pole.probabilityEvidence="subset";
        c.executionBackend="matlab";a=perceiveFrame(frame,c);
        c.executionBackend="native";b=perceiveFrame(frame,c);
        verifyEqual(testCase,b.candidates,a.candidates);
        verifyEqual(testCase,b.probabilityCloud.components.semanticProbability,a.probabilityCloud.components.semanticProbability,'AbsTol',1e-8);
        verifyEqual(testCase,b.probabilityCloud.components.mean,a.probabilityCloud.components.mean,'AbsTol',1e-10);
    end
end
function p=makeShaft()
    z=linspace(-1,3,60).';a=(1:60).'*2.399;
    p=[.12+.025*cos(a),.16+.025*sin(a),z];
end
function e=measure(p,g)
    if nargin<2,g=struct('origin',[0 0],'cellSize',[.6 .6],'mapSize',[1 1]);end
    bins=floor((p(:,1:2)-g.origin)./g.cellSize)+1; ids=sub2ind(g.mapSize,bins(:,2),bins(:,1));
    cfg=structuralPillarConfig(.6); e=findPillarPoleSubsets(p,ids,g,cfg.pole.subset);
end
