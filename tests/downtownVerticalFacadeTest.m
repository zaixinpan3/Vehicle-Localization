classdef downtownVerticalFacadeTest < matlab.unittest.TestCase
% downtownVerticalFacadeTest: Reject broken line support without erasing walls.
    methods (TestClassSetup)
        function paths(t)
            t.applyFixture(matlab.unittest.fixtures.PathFixture(fileparts(fileparts(mfilename('fullpath')))));
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function disconnectedUprightLineCannotSupplyWallSupport(t)
            [x,n]=populationFeatures("broken",[0 0 0]);
            t.verifyFalse(evaluatePillarDistributionRules(x,n,verticalRule()));
        end
        function laterallyExtendedWallRetainsDisconnectedSparseOwner(t)
            [x,n]=populationFeatures("wall",[0 0 0]);
            t.verifyTrue(evaluatePillarDistributionRules(x,n,verticalRule()));
        end
        function continuousOwnerRetainsLocalSupport(t)
            [x,n]=populationFeatures("continuous",[0 0 0]);
            t.verifyTrue(evaluatePillarDistributionRules(x,n,verticalRule()));
        end
        function multiscaleSupportRetainsCoherentWall(t)
            [x,n]=populationFeatures("wall",[0 0 0]);
            t.verifyTrue(evaluatePillarDistributionRules(x,n,multiscaleRule()));
        end
        function multiscaleSupportRejectsStaggeredBrokenLine(t)
            [x,n]=populationFeatures("broken",[0 0 0]);
            t.verifyFalse(evaluatePillarDistributionRules(x,n,multiscaleRule()));
        end
        function translatedMultiscaleSupportPreservesDecision(t)
            [x,n]=populationFeatures("wall",[0 0 0]);
            [y,m]=populationFeatures("wall",[20 -15 8]);
            t.verifyEqual(m,n);
            t.verifyEqual(evaluatePillarDistributionRules(y,m,multiscaleRule()), ...
                evaluatePillarDistributionRules(x,n,multiscaleRule()));
        end
        function lowOwnerRetainsCoherentWallContext(t)
            [x,n]=populationFeatures("shortWall",[0 0 0]);
            t.verifyTrue(evaluatePillarDistributionRules(x,n,shortOwnerRule()));
        end
        function lowOwnerCannotBorrowScatteredTallNeighbors(t)
            [x,n]=populationFeatures("shortClutter",[0 0 0]);
            t.verifyFalse(evaluatePillarDistributionRules(x,n,shortOwnerRule()));
        end
        function translatedLowWallPreservesSupport(t)
            [x,n]=populationFeatures("shortWall",[0 0 0]);
            [y,m]=populationFeatures("shortWall",[20 -15 8]);
            t.verifyEqual(m,n);
            t.verifyEqual(evaluatePillarDistributionRules(y,m,shortOwnerRule()), ...
                evaluatePillarDistributionRules(x,n,shortOwnerRule()));
        end
        function translatedPopulationPreservesDecision(t)
            for scenario=["broken","wall","continuous"]
                [x,n]=populationFeatures(scenario,[0 0 0]);
                [y,m]=populationFeatures(scenario,[20 -15 8]);
                t.verifyEqual(m,n);
                invariant=ismember(n,["whole_r2_linearity","whole_r2_principalVerticalComponent", ...
                    "population_r7_withinVerticalFraction","distribution_heightMaximumGapFraction"]);
                t.verifyEqual(y(invariant),x(invariant),'AbsTol',1e-7);
                t.verifyEqual(evaluatePillarDistributionRules(y,m,verticalRule()), ...
                    evaluatePillarDistributionRules(x,n,verticalRule()));
            end
        end
    end
end
function [x,n]=populationFeatures(scenario,offset)
points=[];ids=[];query=sub2ind([5 5],3,3);
for r=1:5
    if any(scenario==["wall","shortClutter"]),cols=1:5;else,cols=3;end
    for c=cols
        z=[0;.1;.2;1.8;2];
        if scenario=="continuous",z=linspace(0,2,40).';end
        if startsWith(scenario,"short")
            z=linspace(0,4,20).';
            if sub2ind([5 5],r,c)==query,z=linspace(0,.15,5).';end
        end
        xyz=[.3*c+zeros(size(z)),.3*r+zeros(size(z)),z];
        if any(scenario==["broken","continuous"]),xyz(:,3)=xyz(:,3)+10*r;end
        points=[points;xyz+offset];ids=[ids;repmat(sub2ind([5 5],r,c),numel(z),1)]; %#ok<AGROW>
    end
end
pillars=struct('points',points,'pointPillarLinIdx',int32(ids), ...
    'pointAttributes',struct(),'pillarGeometry',struct('origin',[0 0], ...
    'cellSize',[.3 .3],'mapSize',[5 5]));
p=perceptionConfig('Downtown');cloud=coarseSemanticProbabilityCloudConfig(p.voxel);
branch=analyzeDowntownPillarStatistics(pillars,p.offGroundFeatures,cloud);
[x,n]=measureDowntownDistributionContext(branch.columnMaps,query);
[v,columns]=measurePillarMomentContext(branch.columnMaps.statistics,[5 5],query);
x=[x,v];n=[n,"population_"+columns];
shape=aggregatePillarDistributionShape(points,ids);
at=find(shape.pillarIndices==query);x=[x,shape.values(at,:),max(points(ids==query,3))-min(points(ids==query,3))];n=[n,"distribution_"+shape.names,"height"];
end
function rule=verticalRule
cfg=downtownCandidateConfig(.3);groups=cfg.precisionFilter.rules.facade.allOf;rule=[];
for k=1:numel(groups)
    if iscell(groups),group=groups{k};else,group=groups(k);end
    descriptors=strings(1,0);
    for alternative=reshape(group.alternatives,1,[])
        descriptors=[descriptors,string({alternative.predicates.feature})]; %#ok<AGROW>
    end
    if all(ismember(["whole_r2_linearity","distribution_heightMaximumGapFraction"],descriptors))
        rule=group;break;
    end
end
assert(~isempty(rule),'The disconnected-line facade rule must exist.');
end

function rule=multiscaleRule
cfg=downtownCandidateConfig(.3);groups=cfg.precisionFilter.rules.facade.allOf;rule=[];
for k=1:numel(groups)
    if iscell(groups),group=groups{k};else,group=groups(k);end
    descriptors=strings(1,0);
    for alternative=reshape(group.alternatives,1,[])
        descriptors=[descriptors,string({alternative.predicates.feature})]; %#ok<AGROW>
    end
    if all(ismember(["population_r2_transverseAnisotropy","whole_r1_middleVariance"],descriptors))
        rule=group;break;
    end
end
assert(~isempty(rule),'The multiscale facade support rule must exist.');
end

function rule=shortOwnerRule
cfg=downtownCandidateConfig(.3);groups=cfg.precisionFilter.rules.facade.allOf;rule=[];
for k=1:numel(groups)
    if iscell(groups),group=groups{k};else,group=groups(k);end
    descriptors=strings(1,0);
    for alternative=reshape(group.alternatives,1,[])
        descriptors=[descriptors,string({alternative.predicates.feature})]; %#ok<AGROW>
    end
    if all(ismember(["height","whole_r2_normalVerticalComponent","whole_r2_planarity"],descriptors))
        rule=group;break;
    end
end
assert(~isempty(rule),'The low-owner facade support rule must exist.');
end
