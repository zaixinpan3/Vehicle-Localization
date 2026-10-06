classdef downtownHaloSupportTest < matlab.unittest.TestCase
% downtownHaloSupportTest: Distribution support survives candidate expansion.
    methods (TestClassSetup)
        function paths(t)
            root=fileparts(fileparts(mfilename('fullpath')));
            t.applyFixture(matlab.unittest.fixtures.PathFixture(root));
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function unsupportedPoleHaloCannotBorrowAValidSeed(t)
            z=linspace(0,3,15).';
            [x,y]=ndgrid(linspace(.61,.89,20),linspace(.01,.29,20));
            points=[.1+zeros(size(z)),.1+zeros(size(z)),z; ...
                .4+zeros(size(z)),.1+zeros(size(z)),z; x(:),y(:),zeros(numel(x),1)];
            pillars=struct('points',points,'pointPillarLinIdx',int32([ones(15,1);2*ones(15,1);3*ones(numel(x),1)]), ...
                'pointAttributes',struct(),'pillarGeometry',struct('origin',[0 0], ...
                'cellSize',[.3 .3],'mapSize',[1 3]));
            p=perceptionConfig('Downtown');cloud=coarseSemanticProbabilityCloudConfig(p.voxel);cloud.semanticNames="pole";
            branch=analyzeDowntownPillarStatistics(pillars,p.offGroundFeatures,cloud);
            branch.poleCellMask=[true false false];
            cfg=p.downtownCandidates;cfg.precisionFilter.enabled=false;
            gate=filterDowntownPoleDistribution(branch.columnMaps,cfg.poleDistributionGate);
            t.verifyEqual(gate,[true false false]);
            [~,current]=classifyDowntownPillarStatistics(struct(),branch,"pole",cfg);
            cfg.poleDistributionGate.requireHaloSupport=false;
            [~,historical]=classifyDowntownPillarStatistics(struct(),branch,"pole",cfg);
            t.verifyEqual(current.poleCellMask,[true false false]);
            t.verifyEqual(historical.poleCellMask,[true true false]);
            t.verifyEqual(current.columnMaps.statistics,historical.columnMaps.statistics);
        end
        function facadeRequiresHorizontalExtentOrWithinPillarHeight(t)
            stats=struct('pillarIndices',int32([1;2;3]),'count',[20;20;20], ...
                'meanXYZ',[.1 .1 0;.4 .1 2;.7 .1 4], ...
                'covarianceXYZ',repmat([.001 0 .001 0 0 .01],3,1));
            p=perceptionConfig('Downtown');groups=p.downtownCandidates.precisionFilter.rules.facade.allOf;
            if iscell(groups),veto=groups{end};else,veto=groups(end);end
            [x,n]=measurePillarMomentContext(stats,[1 3],[1;2;3]);
            t.verifyFalse(any(evaluatePillarDistributionRules(x,"population_"+n,veto)));
            % A narrow but genuinely tall wall keeps vertical support.
            tall=stats;tall.covarianceXYZ(:,6)=1.2;
            [x,n]=measurePillarMomentContext(tall,[1 3],[1;2;3]);
            t.verifyTrue(all(evaluatePillarDistributionRules(x,"population_"+n,veto)));
            % A low wall can instead supply extended horizontal support.
            wide=stats;wide.pillarIndices=int32([1;7;13]);wide.meanXYZ(:,1)=[.1;1.9;3.7];
            [x,n]=measurePillarMomentContext(wide,[1 13],7);
            t.verifyTrue(evaluatePillarDistributionRules(x,"population_"+n,veto));
            wide.meanXYZ=wide.meanXYZ+[100 -200 30];
            [shifted,other]=measurePillarMomentContext(wide,[1 13],7);
            t.verifyEqual(other,n);t.verifyEqual(shifted,x,'AbsTol',1e-8);
        end
        function historicalOperatingPointsPreserveHaloExpansion(t)
            for policy=["highRecall","strictPrecision"]
                cfg=downtownCandidateConfig(.3,policy);
                t.verifyFalse(cfg.poleDistributionGate.requireHaloSupport);
            end
            t.verifyTrue(downtownCandidateConfig(.3).poleDistributionGate.requireHaloSupport);
        end
    end
end
