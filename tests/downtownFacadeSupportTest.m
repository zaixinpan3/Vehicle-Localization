classdef downtownFacadeSupportTest < matlab.unittest.TestCase
% downtownFacadeSupportTest: Whole-population wall and distributed-clutter controls.
    methods (TestClassSetup)
        function paths(t)
            t.applyFixture(matlab.unittest.fixtures.PathFixture(fileparts(fileparts(mfilename('fullpath')))));
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function coherentWallRetainsSparseOwner(t)
            [stats,query]=makePopulation(false,true);[x,n]=descriptors(stats,query);
            t.verifyTrue(evaluatePillarDistributionRules(x,n,facadeSupportRule()));
            t.verifyEqual(stats.count(stats.pillarIndices==query),1);
        end
        function uniformWallRetainsOwnVerticalSupport(t)
            [stats,query]=makePopulation(false,false);[x,n]=descriptors(stats,query);
            t.verifyTrue(evaluatePillarDistributionRules(x,n,facadeSupportRule()));
        end
        function denseDistributedClutterCannotBorrowFacade(t)
            [stats,query]=makePopulation(true,false);[x,n]=descriptors(stats,query);
            t.verifyFalse(evaluatePillarDistributionRules(x,n,facadeSupportRule()));
        end
        function translatedPopulationKeepsDecisionAndStatistics(t)
            for clutter=[false true]
                [stats,query]=makePopulation(clutter,false);[x,n]=descriptors(stats,query);
                shifted=stats;shifted.meanXYZ=shifted.meanXYZ+[12 -9 5];
                shifted.minimumXYZ=shifted.minimumXYZ+[12 -9 5];shifted.maximumXYZ=shifted.maximumXYZ+[12 -9 5];
                [y,m]=descriptors(shifted,query);
                t.verifyEqual(m,n);t.verifyEqual(y,x,'AbsTol',1e-7);
                t.verifyEqual(evaluatePillarDistributionRules(y,m,facadeSupportRule()), ...
                    evaluatePillarDistributionRules(x,n,facadeSupportRule()));
                t.verifyEqual(shifted.covarianceXYZ,stats.covarianceXYZ);
            end
        end
    end
end
function [stats,query]=makePopulation(clutter,sparse)
ids=[];points=[];query=sub2ind([15 15],8,8);
for r=1:15
    cols=8;if clutter,cols=1:15;end
    for c=cols
        id=sub2ind([15 15],r,c);n=12;if sparse && id==query,n=1;end
        z=linspace(-1,3,n).';if clutter,z=linspace(-.2,.2,n).';end
        xyz=[c*.3+.01*cos((1:n).'),r*.3+.01*sin((1:n).'),z];
        points=[points;xyz];ids=[ids;repmat(id,n,1)]; %#ok<AGROW>
    end
end
stats=aggregatePillarStatistics(points,ids,struct(),false);
end
function [x,names]=descriptors(stats,query)
[x,names]=measurePillarMomentContext(stats,[15 15],query);names="population_"+names;
cov=stats.covarianceXYZ(stats.pillarIndices==query,:);
matrix=[cov(1) cov(2) cov(4);cov(2) cov(3) cov(5);cov(4) cov(5) cov(6)];
x=[x,max(0,min(eig(matrix)))];names=[names,"whole_r0_smallestVariance"];
end
function rule=facadeSupportRule
cfg=downtownCandidateConfig(.3);groups=cfg.precisionFilter.rules.facade.allOf;
if iscell(groups),rule=groups{end};else,rule=groups(end);end
end
