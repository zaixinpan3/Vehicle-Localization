classdef softPointAssociationTest < matlab.unittest.TestCase
% softPointAssociationTest Ambiguity scatter, priors and independent anchors.
    methods (TestClassSetup)
        function paths(~),setupVehicleLocalization;end
    end
    methods (Test)
        function symmetricModesRetainBetweenModeScatter(t)
            cfg=softPointAssociationTest.settings();means=[-.3 0;.3 0];covs=repmat(.01*eye(2),1,1,2);
            [mu,cov,n]=softPointAssociationTarget(means,covs,[1;1],[0;0],1,cfg);
            t.verifyEqual(mu,[0 0],AbsTol=1e-12);t.verifyEqual(cov,diag([.10,.01]),AbsTol=1e-12);t.verifyEqual(n,2);
        end
        function dominantPriorKeepsSharpTarget(t)
            cfg=softPointAssociationTest.settings();means=[0 0;.3 0];covs=repmat(.01*eye(2),1,1,2);prior=[0;2*log(19)];
            [mu,cov,n]=softPointAssociationTarget(means,covs,prior,prior,1,cfg);
            t.verifyEqual(mu,[0 0]);t.verifyEqual(cov,.01*eye(2));t.verifyEqual(n,1);
        end
        function remoteTemperedModeCannotEraseLocalSupport(t)
            cfg=softPointAssociationTest.settings();means=[0 0;10 0];covs=repmat(.01*eye(2),1,1,2);
            % Geometry tempering makes the remote mode cheaper, but it lies
            % outside the local ambiguity neighborhood of the MAP target.
            [mu,cov,n]=softPointAssociationTarget(means,covs,[100;101],[100;0],1,cfg);
            t.verifyEqual(mu,[0 0]);t.verifyEqual(cov,.01*eye(2));t.verifyEqual(n,1);
        end
        function invariantToPermutationTranslationAndCommonPriorCost(t)
            cfg=softPointAssociationTest.settings();means=[-.3 .1;.3 -.1;4 0];covs=repmat(.01*eye(2),1,1,3);cost=[1;2;Inf];prior=[0;.2;1];
            [mu,cov,n]=softPointAssociationTarget(means,covs,cost,prior,1,cfg);
            order=[3 2 1];shift=[1e3 -50];
            [other,otherCov,otherN]=softPointAssociationTarget(means(order,:)+shift,covs(:,:,order),cost(order)+50,prior(order)+50,3,cfg);
            t.verifyEqual(other,mu+shift,AbsTol=1e-10);t.verifyEqual(otherCov,cov,AbsTol=1e-10);t.verifyEqual(otherN,n);
        end
        function independentPoleAnchorsRetainAccurateFineSolve(t)
            [fixed,moving,cfg]=canonicalPyramidTest.aliasRegistrationScene();cfg.softPointAssociation=softPointAssociationTest.settings();
            result=registerSemanticProbabilityCloud(fixed,moving,[.7 0 0],cfg);
            t.verifyTrue(result.accepted);t.verifyEqual(result.pyramid.unmergedAnchors,2);
            t.verifyFalse(result.pyramid.softFineAssociation);t.verifyEqual(result.poseXYTheta,[0 0 0],AbsTol=.02);
        end
        function unavailableAidLeavesAmbiguousGeometryUnchanged(t)
            [fixed,moving,cfg]=canonicalPyramidTest.fullyAliasedScene();cfg.softPointAssociation=softPointAssociationTest.settings();
            baseline=registerSemanticProbabilityCloud(fixed,moving,[-.4 0 0],cfg);
            aids={struct('valid',false),struct('valid',true,'position',[0 0],'covariance',100*eye(2))};
            for k=1:numel(aids)
                result=registerSemanticProbabilityCloud(fixed,moving,[-.4 0 0],cfg,aids{k});
                t.verifyEqual(result.poseXYTheta,baseline.poseXYTheta,AbsTol=1e-12);
                t.verifyEqual(result.information,baseline.information,AbsTol=1e-12);
                t.verifyFalse(result.positionAiding.used);
            end
        end
        function informativeAidSelectsASharpMode(t)
            [fixed,moving,cfg]=canonicalPyramidTest.fullyAliasedScene();cfg.softPointAssociation=softPointAssociationTest.settings();
            aid=struct('valid',true,'position',[0 0],'covariance',.01*eye(2));
            result=registerSemanticProbabilityCloud(fixed,moving,[-.4 0 0],cfg,aid);
            t.verifyTrue(result.accepted);t.verifyTrue(result.positionAiding.used);
            t.verifyFalse(result.pyramid.softFineAssociation);t.verifyEqual(result.poseXYTheta,[0 0 0],AbsTol=1e-6);
            t.verifyFalse(result.positionAiding.gnssInformationAdded);
        end
        function invalidConfigurationFailsBeforeSolving(t)
            cfg=softPointAssociationTest.settings();validateSoftPointAssociation(cfg);
            for field=["temperature","radius","minimumPosterior","hardWithUnmergedAnchors"]
                bad=cfg;bad.(field)=NaN;
                t.verifyError(@()validateSoftPointAssociation(bad),'VehicleLocalization:InvalidSoftPointAssociation');
            end
            cfg.hardWithUnmergedAnchors=1;t.verifyError(@()validateSoftPointAssociation(cfg),'VehicleLocalization:InvalidSoftPointAssociation');
        end
    end
    methods (Static)
        function cfg=settings()
            cfg=struct('temperature',2.5,'radius',1.5,'minimumPosterior',.9,'hardWithUnmergedAnchors',2);
        end
    end
end
