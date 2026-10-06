classdef downtownSparseSupportTest < matlab.unittest.TestCase
% downtownSparseSupportTest: Reject smooth road and disconnected pole heights.
    methods (TestClassSetup)
        function paths(t)
            t.applyFixture(matlab.unittest.fixtures.PathFixture(fileparts(fileparts(mfilename('fullpath')))));
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function continuousSlopeIsNotCurbDiscontinuity(t)
            [x,y]=meshgrid((0:8)*.3);base=.02*x(:)+.01*y(:);
            stats=struct('pillarIndices',int32((1:81).'),'count',ones(81,1), ...
                'meanXYZ',[x(:),y(:),base],'covarianceXYZ',zeros(81,6));
            rule=lastRule('curb');[v,n]=measurePillarMomentContext(stats,[9 9],41);
            t.verifyFalse(evaluatePillarDistributionRules(v,"population_"+n,rule));
            stats.meanXYZ(:,3)=base+.14*(x(:)>=1.2);
            [v,n]=measurePillarMomentContext(stats,[9 9],41);
            t.verifyTrue(evaluatePillarDistributionRules(v,"population_"+n,rule));
        end
        function disconnectedHeightsCannotKeepIsolatedPoleOwner(t)
            rule=lastRule('pole');
            for disconnected=[false true]
                z=linspace(0,5.2,10).';
                if disconnected,z=[(0:.1:.4)';(4.8:.1:5.2)'];end
                points=[.1+.01*cos(z),.1+.01*sin(z),z];
                s=aggregatePillarDistributionShape(points,ones(size(z)));
                accepted=evaluatePillarDistributionRules([numel(z),s.values], ...
                    ["count","distribution_"+s.names],rule);
                t.verifyEqual(accepted,~disconnected);
                t.verifyEqual(size(points,1),10);
            end
        end
        function sparseOwnersDoNotInventDenseContinuityEvidence(t)
            points=[.1 .1 0;.1 .1 2];s=aggregatePillarDistributionShape(points,[1;1]);
            t.verifyTrue(evaluatePillarDistributionRules([2,s.values], ...
                ["count","distribution_"+s.names],lastRule('pole')));
        end
    end
end
function rule=lastRule(name)
cfg=downtownCandidateConfig(.3);groups=cfg.precisionFilter.rules.(name).allOf;
if iscell(groups),rule=groups{end};else,rule=groups(end);end
end
