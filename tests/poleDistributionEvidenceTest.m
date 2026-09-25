function tests=poleDistributionEvidenceTest
% poleDistributionEvidenceTest: Shaft evidence and whole-pillar integration.
    tests=functiontests(localfunctions);
end
function setupOnce(~)
    setupVehicleLocalization();
end
function testContinuousShaftOutranksDisconnectedReturns(testCase)
    g=geometry(); n=61; z=linspace(0,3,n).';
    shaft=[repmat([1.19 .9],n,1),z];
    fragments=shaft; fragments(:,3)=[linspace(0,.1,30),linspace(2.9,3,31)].';
    a=measure(shaft,g); b=measure(fragments,g);
    verifyGreaterThan(testCase,a.score,.95);
    verifyLessThan(testCase,b.score,.08);
    verifyGreaterThan(testCase,a.score,b.score);
end
function testContextReducesEvidenceAndBoundaryPartnerContributes(testCase)
    g=geometry(); z=linspace(0,3,31).';
    pole=[repmat([1.19 .9],31,1),z;repmat([1.21 .9],31,1),z+.01];
    clean=measure(pole,g);
    clutter=[repmat([1.6 .9],62,1),linspace(0,3,62).'];
    mixed=measure([pole;clutter],g);
    verifyEqual(testCase,clean.coreCount,62*ones(2,1));
    verifyGreaterThan(testCase,min(clean.score),.9);
    verifyLessThan(testCase,max(mixed.score),.51);
end
function testSingleHighOutlierIsWeakEvidence(testCase)
    z=[linspace(0,.1,40),5].'; e=measure([repmat([1.19 .9],41,1),z],geometry());
    verifyLessThan(testCase,e.score,.01);
end
function testPermutationAndTranslationInvariance(testCase)
    g=geometry(); p=[repmat([1.19 .9],31,1),linspace(0,3,31).'];
    a=measure(p,g); g.origin=g.origin+[10 -7];
    b=measure(p(end:-1:1,:)+[10 -7 2],g);
    verifyEqual(testCase,a.score,b.score,'AbsTol',1e-12);
end
function testEmptyAndDisabledSelection(testCase)
    g=geometry(); e=scorePolePillarDistributions(zeros(0,3),zeros(0,1),g,zeros(0,2));verifyEmpty(testCase,e.score);
    e=scorePolePillarDistributions([.1 .1 0;.1 .1 3],[1;1],g,[.1 .1],false);
    verifyEqual(testCase,e.score,0);
end
function testScoringKeepsPillarMasksAndAllPointMoments(testCase)
    cfg=perceptionConfig(); z=linspace(-1,3,81).';
    theta=(1:81).'*2.399; points=[3.4+.05*cos(theta),.4+.05*sin(theta),z];
    frame=struct('x',points(:,1),'y',points(:,2),'z',points(:,3),'intensity',ones(81,1));
    grid=pillarizePointCloud(frame,cfg.voxel); cloud=cfg.coarseProbabilityCloud;
    cloud.semanticNames="pole"; structural=cfg.offGroundFeatures;
    structural.pole.probabilityEvidence="shape"; old=analyzeStructuralPillars(grid,structural,cloud);
    structural.pole.probabilityEvidence="distribution"; updated=analyzeStructuralPillars(grid,structural,cloud);
    verifyTrue(testCase,any(old.poleCellMask(:)));
    verifyEqual(testCase,updated.poleCellMask,old.poleCellMask);
    verifyEqual(testCase,updated.columnMaps.moments,old.columnMaps.moments);
    verifyTrue(testCase,isfield(updated.columnMaps,'poleDistribution'));
    verifyGreaterThan(testCase,updated.poleProbability(updated.poleCellMask),single(.9));
end
function e=measure(p,g)
    bins=floor((p(:,1:2)-g.origin)./g.cellSize)+1;
    ids=sub2ind(g.mapSize,bins(:,2),bins(:,1));occupied=unique(ids);
    peak=repmat(g.origin+[1.2 .9],numel(occupied),1);
    e=scorePolePillarDistributions(p,ids,g,peak);
end
function g=geometry()
    g=struct('origin',[0 0],'cellSize',[.6 .6],'mapSize',[5 5]);
end
