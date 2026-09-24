classdef canonicalPyramidTest < matlab.unittest.TestCase
% canonicalPyramidTest Cloud canonicalization and the coarse-to-fine registration contract.
    methods (TestClassSetup)
        function paths(~)
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function momentMatchingPreservesClassMassAndMoments(testCase)
            cloud=canonicalPyramidTest.splitScene();c=cloud.components;
            [merged,groups]=canonicalizeSemanticCloud(cloud,1.5);m=merged.components;
            testCase.verifyEqual(m.numComponents,c.numComponents-2);
            testCase.verifyEqual(numel(groups),m.numComponents);
            testCase.verifyEqual(sort(vertcat(groups{:})),(1:c.numComponents).');
            poles=c.semanticName=="pole";mergedPoles=m.semanticName=="pole";
            testCase.verifyEqual(sum(m.mixtureWeight(mergedPoles)),sum(c.mixtureWeight(poles)),AbsTol=1e-12);
            first=@(cc,keep)sum(cc.mean(keep,:).*cc.mixtureWeight(keep),1);
            testCase.verifyEqual(first(m,mergedPoles),first(c,poles),AbsTol=1e-12);
            second=@(cc,keep)canonicalPyramidTest.secondMoment(cc,keep);
            testCase.verifyEqual(second(m,mergedPoles),second(c,poles),AbsTol=1e-12);
            % Line classes and distant poles are untouched.
            curbs=c.semanticName=="curb";
            testCase.verifyEqual(m.mean(m.semanticName=="curb",:),c.mean(curbs,:));
            testCase.verifyEqual(m.covariance(:,:,m.semanticName=="curb"),c.covariance(:,:,curbs));
            for g=find(cellfun(@numel,groups)>1).'
                S=m.covariance(:,:,g);testCase.verifyEqual(S,S.',AbsTol=0);testCase.verifyGreaterThan(min(eig(S)),0);
            end
        end
        function zeroRadiusIsTheIdentity(testCase)
            cloud=canonicalPyramidTest.splitScene();
            [same,groups]=canonicalizeSemanticCloud(cloud,0);
            testCase.verifyEqual(same,cloud);
            testCase.verifyEqual(groups,num2cell((1:cloud.components.numComponents).'));
        end
        function commonWeightScaleDoesNotChangeTheGrouping(testCase)
            cloud=canonicalPyramidTest.splitScene();
            [a,ga]=canonicalizeSemanticCloud(cloud,1.5);
            cloud.components.mixtureWeight=1e-200*cloud.components.mixtureWeight;
            [b,gb]=canonicalizeSemanticCloud(cloud,1.5);
            testCase.verifyEqual(gb,ga);
            testCase.verifyEqual(b.components.mean,a.components.mean,AbsTol=1e-12);
            testCase.verifyEqual(b.components.covariance,a.components.covariance,AbsTol=1e-12);
        end
        function heightMomentsMarginalizeToThePlanarMoments(testCase)
            cloud=canonicalPyramidTest.splitScene();c=cloud.components;n=c.numComponents;
            c.meanXYZ=[c.mean,(1:n).'];c.covarianceXYZ=repmat(eye(3),1,1,n);
            for k=1:n,c.covarianceXYZ(1:2,1:2,k)=c.covariance(:,:,k);end
            c.heightAvailable=true(n,1);cloud.components=c;
            merged=canonicalizeSemanticCloud(cloud,1.5);m=merged.components;
            testCase.verifyTrue(all(m.heightAvailable));
            testCase.verifyEqual(m.meanXYZ(:,1:2),m.mean,AbsTol=0);
            testCase.verifyEqual(m.covarianceXYZ(1:2,1:2,:),m.covariance,AbsTol=0);
            for k=1:m.numComponents,testCase.verifyGreaterThan(min(eig(m.covarianceXYZ(:,:,k))),0);end
            % One member without height removes height from its merged group.
            c.heightAvailable(2)=false;cloud.components=c;
            [partial,groups]=canonicalizeSemanticCloud(cloud,1.5);
            g=find(cellfun(@(x)any(x==2),groups));
            testCase.verifyFalse(partial.components.heightAvailable(g));
            testCase.verifyTrue(all(partial.components.heightAvailable(setdiff(1:numel(groups),g))));
        end
        function heightEvidenceSidecarFollowsTheGroups(testCase)
            cloud=canonicalPyramidTest.splitScene();n=cloud.components.numComponents;
            cloud.heightEvidence=struct('mean',[cloud.components.mean,ones(n,1)],'covariance',repmat(eye(3),1,1,n), ...
                'available',true(n,1),'scope',"test");
            cloud.heightEvidence.available(3)=false;
            [merged,groups]=canonicalizeSemanticCloud(cloud,1.5);e=merged.heightEvidence;
            testCase.verifyEqual(numel(e.available),numel(groups));
            testCase.verifyEqual(e.scope,"test");
            g=find(cellfun(@(x)any(x==3),groups));
            testCase.verifyFalse(e.available(g));
            testCase.verifyTrue(all(e.available(setdiff(1:numel(groups),g))));
        end
        function pyramidRecoversTheTrueBasinOfASplitLandmark(testCase)
            [fixed,moving,cfg]=canonicalPyramidTest.aliasRegistrationScene();
            flat=cfg;flat.pyramid.mapMergeRadius=0;flat.pyramid.sourceMergeRadius=0;
            single=registerSemanticProbabilityCloud(fixed,moving,[.7 0 0],flat);
            pyramid=registerSemanticProbabilityCloud(fixed,moving,[.7 0 0],cfg);
            testCase.verifyGreaterThan(abs(single.poseXYTheta(1)),.3,"A single-level solve locks onto the split sub-components.");
            testCase.verifyTrue(pyramid.accepted,pyramid.reason);
            testCase.verifyEqual(pyramid.poseXYTheta,[0 0 0],AbsTol=2e-2);
            testCase.verifyTrue(pyramid.pyramid.coarseAccepted);
            testCase.verifyFalse(pyramid.pyramid.coarseRetained);
            testCase.verifyEqual(pyramid.initialPoseXYTheta,[.7 0 0]);
            testCase.verifyEqual(pyramid.pyramid.fineSeedXYTheta,pyramid.pyramid.coarsePoseXYTheta);
            testCase.verifyLessThan(pyramid.pyramid.coarseComponents,fixed.components.numComponents);
        end
        function trustRadiusRetainsTheCoarsePoseForPlanarOnlyRefinement(testCase)
            [fixed,moving,cfg]=canonicalPyramidTest.fullyAliasedScene();
            r=registerSemanticProbabilityCloud(fixed,moving,[-.4 0 0],cfg);
            testCase.verifyTrue(r.accepted);
            testCase.verifyTrue(r.pyramid.coarseRetained);
            testCase.verifyEqual(r.reason,"coarseRetainedByTrustRadius");
            testCase.verifyEqual(r.poseXYTheta,r.pyramid.coarsePoseXYTheta);
            testCase.verifyGreaterThan(r.pyramid.refinementShiftM,cfg.pyramid.trustRadius);
            testCase.verifyTrue(all(r.correspondences.target>=1 & r.correspondences.target<=fixed.components.numComponents));
            testCase.verifyFalse(r.positionAiding.used);
            % Position aid lifts the gate: the refinement is then informed by
            % more than planar geometry. (relativeHeightAssociationTest covers
            % the same lift by height association.)
            aid=struct('position',[0 0],'covariance',.01*eye(2),'valid',true);
            aided=registerSemanticProbabilityCloud(fixed,moving,[-.4 0 0],cfg,aid);
            testCase.verifyFalse(aided.pyramid.coarseRetained);
            testCase.verifyFalse(aided.pyramid.planarOnlyRefinement);
            testCase.verifyEqual(aided.poseXYTheta,[0 0 0],AbsTol=1e-6);
        end
        function obsoleteConfigurationIsRejected(testCase)
            [fixed,moving,cfg]=canonicalPyramidTest.aliasRegistrationScene();
            testCase.verifyError(@()registerSemanticProbabilityCloud(fixed,moving,[0 0 0],rmfield(cfg,'pyramid')), ...
                'VehicleLocalization:MissingPyramidConfiguration');
            bad=cfg;bad.pyramid.trustRadius=0;
            testCase.verifyError(@()registerSemanticProbabilityCloud(fixed,moving,[0 0 0],bad), ...
                'VehicleLocalization:InvalidPyramidConfiguration');
        end
    end
    methods (Static)
        function cloud=splitScene()
        % One pole split into three sub-components 0.6 m apart, one distant
        % pole, and two curb lines.
            means=[0 0;.6 .05;-.5 -.05;8 3;2 -4;-3 6];
            covariance=cat(3,.03*eye(2),.03*eye(2),.03*eye(2),.03*eye(2),[2 0;0 .01],[.01 0;0 2]);
            c=struct('mean',means,'covariance',covariance,'semanticName',["pole";"pole";"pole";"pole";"curb";"curb"], ...
                'mixtureWeight',[.3;.2;.1;.2;.1;.1],'numComponents',6);
            c.repeatability=ones(6,1);
            cloud=struct('components',c,'frameCalibration',lidarFrameCalibrationConfig());
        end
        function value=secondMoment(c,keep)
            value=zeros(2);
            for k=find(keep(:)).'
                value=value+c.mixtureWeight(k)*(c.covariance(:,:,k)+c.mean(k,:).'*c.mean(k,:));
            end
        end
        function [fixed,moving,cfg]=aliasRegistrationScene()
        % Source: three poles that the map splits into two sub-components
        % 0.6 m apart along x, two poles the map keeps single, and two curb
        % lines along x. A planar solve seeded 0.7 m ahead locks onto the
        % split sub-components; the canonical level does not.
            means=[0 0;4 2;-4 -2;0 -6;0 6;6 5;-6 -5];
            names=[repmat("pole",5,1);"curb";"curb"];n=numel(names);
            covariance=repmat(.03*eye(2),1,1,n);covariance(:,:,n-1)=[3 0;0 .01];covariance(:,:,n)=[3 0;0 .01];
            c=struct('mean',means,'covariance',covariance,'semanticName',names,'mixtureWeight',ones(n,1)/n,'numComponents',n);
            c.repeatability=ones(n,1);
            moving=struct('components',c,'frameCalibration',lidarFrameCalibrationConfig());
            fixed=moving;f=fixed.components;
            f.mean=[f.mean;means(1:3,:)+[.6 0]];f.covariance=cat(3,f.covariance,repmat(.03*eye(2),1,1,3));
            f.semanticName=[f.semanticName;repmat("pole",3,1)];f.mixtureWeight=[f.mixtureWeight;ones(3,1)/n];
            f.mixtureWeight=f.mixtureWeight/sum(f.mixtureWeight);f.repeatability=ones(n+3,1);f.numComponents=n+3;
            fixed.components=f;cfg=distributionRegistrationConfig();
        end
        function [fixed,moving,cfg]=fullyAliasedScene()
        % Every pole of the map is duplicated 0.4 m behind, so the canonical
        % pose is the midpoint and any planar refinement leaves it by 0.2 m.
            [~,moving,cfg]=canonicalPyramidTest.aliasRegistrationScene();
            fixed=moving;f=fixed.components;poles=find(f.semanticName=="pole");
            f.mean=[f.mean;f.mean(poles,:)-[.4 0]];f.covariance=cat(3,f.covariance,f.covariance(:,:,poles));
            f.semanticName=[f.semanticName;f.semanticName(poles)];f.mixtureWeight=[f.mixtureWeight;f.mixtureWeight(poles)];
            f.mixtureWeight=f.mixtureWeight/sum(f.mixtureWeight);f.repeatability=[f.repeatability;f.repeatability(poles)];
            f.numComponents=numel(f.semanticName);fixed.components=f;
        end
    end
end
