classdef downtownPoleRecoveryTest < matlab.unittest.TestCase
% downtownPoleRecoveryTest: Conservative continuous-owner morphology controls.
    methods (TestClassSetup)
        function paths(t)
            t.applyFixture(matlab.unittest.fixtures.PathFixture(fileparts(fileparts(mfilename('fullpath')))));
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function continuousDenseOwnerPasses(t)
            [x,names]=shaftFeatures(false,false);
            t.verifyTrue(evaluatePillarDistributionRules(x,names,recoveryRule()));
        end
        function disconnectedDenseReturnsAreRejected(t)
            [x,names]=shaftFeatures(true,false);
            t.verifyFalse(evaluatePillarDistributionRules(x,names,recoveryRule()));
        end
        function wideOwnerCannotUseRecovery(t)
            [x,names]=shaftFeatures(false,true);
            t.verifyFalse(evaluatePillarDistributionRules(x,names,recoveryRule()));
        end
        function sparseOrCrowdedOwnersCannotBorrowRecovery(t)
            [x,names]=shaftFeatures(false,false);rule=recoveryRule();
            x(names=="count")=20;
            t.verifyFalse(evaluatePillarDistributionRules(x,names,rule));
            x(names=="count")=60;x(names=="population_r1_centerCountFraction")=.5;
            t.verifyFalse(evaluatePillarDistributionRules(x,names,rule));
        end
    end
end
function [x,names]=shaftFeatures(disconnected,wide)
z=linspace(0,1.2,60).';
if disconnected,z=[linspace(0,.2,30),linspace(1,1.2,30)].';end
radius=.04;if wide,radius=.14;end
phi=(0:59)'*2*pi/7;xyz=[radius*cos(phi),radius*sin(phi),z];
stats=aggregatePillarStatistics(xyz,ones(60,1),struct(),false);
shape=aggregatePillarDistributionShape(xyz,ones(60,1));
covariance=stats.covarianceXYZ;vertical=covariance(6)/sum(covariance([1 3 6]));
radial=covariance(1)+covariance(3)-sum(covariance(4:5).^2)/covariance(6);
names=["count","height","nonreflective_verticalVarianceFraction","radialVariance", ...
       "population_r1_centerCountFraction","distribution_"+shape.names];
x=[60,max(z)-min(z),vertical,radial,1,shape.values];
end
function rule=recoveryRule
cfg=downtownCandidateConfig(.3);groups=cfg.precisionFilter.rules.pole.allOf;
if iscell(groups),group=groups{1};else,group=groups(1);end
rule=group.alternatives(end).predicates;
end
