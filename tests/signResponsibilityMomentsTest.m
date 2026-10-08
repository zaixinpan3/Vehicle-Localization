classdef signResponsibilityMomentsTest < matlab.unittest.TestCase
% signResponsibilityMomentsTest Intensity-responsibility sign moments and the sign mass gate.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));setupVehicleLocalization;
        end
    end
    methods (Test)
        function dimReturnsBarelyMoveTheSignMoments(testCase)
            [post,postI]=postReturns(60);[panel,panelI]=panelReturns(10);
            stats=measureSignResponsibilityMoments([post;panel],7*ones(70,1),[postI;panelI],7,1800,100);
            % The dim post adds below 1e-5 to the expected number of sign returns.
            testCase.verifyEqual(stats.count,sum(1./(1+exp(-(panelI-1800)/100))),AbsTol=1e-4);
            testCase.verifyEqual(stats.meanXYZ,mean(panel,1),AbsTol=2e-3);
            uniform=mean([post;panel],1);
            testCase.verifyGreaterThan(norm(uniform(1:2)-mean(panel(:,1:2),1)),0.2);
            covariance=cov(panel,1);
            testCase.verifyEqual(stats.covarianceXYZ,covariance([1 4 5 7 8 9]),AbsTol=2e-3);
        end
        function singleBrightReturnHasAboutUnitMass(testCase)
            [post,postI]=postReturns(300);[panel,panelI]=panelReturns(1);
            stats=measureSignResponsibilityMoments([post;panel],ones(301,1),[postI;panelI],1,1800,100);
            testCase.verifyEqual(stats.count,1,AbsTol=0.01);
            testCase.verifyLessThan(stats.count,coarseSemanticProbabilityCloudConfig().trafficSignMinimumMass);
        end
        function onlySelectedPillarsAreMeasured(testCase)
            [panel,panelI]=panelReturns(8);
            ids=[3*ones(4,1);5*ones(4,1)];
            stats=measureSignResponsibilityMoments(panel,ids,panelI,5,1800,100);
            testCase.verifyEqual(double(stats.pillarIndices),5);
            testCase.verifyEqual(stats.meanXYZ,mean(panel(5:8,:),1),AbsTol=1e-3);
        end
        function signComponentsNeedTheMinimumSoftMass(testCase)
            cfg=coarseSemanticProbabilityCloudConfig();cfg.semanticNames="trafficSign";
            cloud=buildCoarseSemanticProbabilityCloud(struct(),signColumns([2.4 5.0]),cfg);
            testCase.verifyEqual(cloud.components.numComponents,1);
            testCase.verifyEqual(cloud.components.mean(1,:),[12.3 -4.1],AbsTol=1e-9);
            cfg.trafficSignMinimumMass=2;
            cloud=buildCoarseSemanticProbabilityCloud(struct(),signColumns([2.4 5.0]),cfg);
            testCase.verifyEqual(cloud.components.numComponents,2);
        end
    end
end

function [p,intensity]=postReturns(n)
% A dim vertical post at the origin.
    z=linspace(0,3,n).';p=[0.02*sin((1:n).'),0.02*cos((1:n).'),z];intensity=80+40*mod((1:n).',3);
end

function [p,intensity]=panelReturns(n)
% Bright sign-panel returns 0.3 m from the post, higher up.
    s=linspace(-0.2,0.2,n).';p=[0.3+0*s,s,2.3+0.1*s];intensity=2400+300*mod((1:n).',2);
end

function off=signColumns(masses)
% Two accepted sign pillars with soft masses MASSES at (5.2,3.1) and (12.3,-4.1).
    mapSize=[100 100];cells=[sub2ind(mapSize,30,40),sub2ind(mapSize,70,20)];
    xy=[5.2 3.1;12.3 -4.1];n=prod(mapSize);
    moments=struct('count',zeros(n,1),'mean',zeros(n,2),'covariance',zeros(n,3), ...
        'meanZ',zeros(n,1),'heightCovariance',zeros(n,3));
    moments.count(cells)=masses;moments.mean(cells,:)=xy;moments.covariance(cells,:)=repmat([0.01 0 0.01],2,1);
    moments.meanZ(cells)=2;moments.heightCovariance(cells,:)=repmat([0 0 0.01],2,1);
    xMap=nan(mapSize);yMap=xMap;xMap(cells)=xy(:,1);yMap(cells)=xy(:,2);
    maps=struct('mapSize',mapSize,'dx',0.6,'dy',0.6,'xMap',xMap,'yMap',yMap,'moments',moments, ...
        'trafficSignMoments',moments);
    mask=false(mapSize);mask(cells)=true;probability=zeros(mapSize);probability(cells)=0.8;
    off=struct('columnMaps',maps,'trafficSignCellMask',mask,'trafficSignProbability',probability);
end
