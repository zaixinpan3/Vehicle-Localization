classdef canonicalPyramidTest < matlab.unittest.TestCase
% canonicalPyramidTest Map canonicalization preserves mass and spatial moments.
    methods (TestClassSetup)
        function paths(~)
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function momentMatchingPreservesClassMassAndMoments(testCase)
            cloud=canonicalPyramidTest.splitScene();c=cloud.components;
            [merged,groups]=registrationSupport.canonicalizeSemanticCloud(cloud,1.5);m=merged.components;
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
            [same,groups]=registrationSupport.canonicalizeSemanticCloud(cloud,0);
            testCase.verifyEqual(same,cloud);
            testCase.verifyEqual(groups,num2cell((1:cloud.components.numComponents).'));
        end
        function commonWeightScaleDoesNotChangeTheGrouping(testCase)
            cloud=canonicalPyramidTest.splitScene();
            [a,ga]=registrationSupport.canonicalizeSemanticCloud(cloud,1.5);
            cloud.components.mixtureWeight=1e-200*cloud.components.mixtureWeight;
            [b,gb]=registrationSupport.canonicalizeSemanticCloud(cloud,1.5);
            testCase.verifyEqual(gb,ga);
            testCase.verifyEqual(b.components.mean,a.components.mean,AbsTol=1e-12);
            testCase.verifyEqual(b.components.covariance,a.components.covariance,AbsTol=1e-12);
        end
        function heightMomentsMarginalizeToThePlanarMoments(testCase)
            cloud=canonicalPyramidTest.splitScene();c=cloud.components;n=c.numComponents;
            c.meanXYZ=[c.mean,(1:n).'];c.covarianceXYZ=repmat(eye(3),1,1,n);
            for k=1:n,c.covarianceXYZ(1:2,1:2,k)=c.covariance(:,:,k);end
            c.heightAvailable=true(n,1);cloud.components=c;
            merged=registrationSupport.canonicalizeSemanticCloud(cloud,1.5);m=merged.components;
            testCase.verifyTrue(all(m.heightAvailable));
            testCase.verifyEqual(m.meanXYZ(:,1:2),m.mean,AbsTol=0);
            testCase.verifyEqual(m.covarianceXYZ(1:2,1:2,:),m.covariance,AbsTol=0);
            for k=1:m.numComponents,testCase.verifyGreaterThan(min(eig(m.covarianceXYZ(:,:,k))),0);end
            % One member without height removes height from its merged group.
            c.heightAvailable(2)=false;cloud.components=c;
            [partial,groups]=registrationSupport.canonicalizeSemanticCloud(cloud,1.5);
            g=find(cellfun(@(x)any(x==2),groups));
            testCase.verifyFalse(partial.components.heightAvailable(g));
            testCase.verifyTrue(all(partial.components.heightAvailable(setdiff(1:numel(groups),g))));
        end
        function heightEvidenceSidecarFollowsTheGroups(testCase)
            cloud=canonicalPyramidTest.splitScene();n=cloud.components.numComponents;
            cloud.heightEvidence=struct('mean',[cloud.components.mean,ones(n,1)],'covariance',repmat(eye(3),1,1,n), ...
                'available',true(n,1),'scope',"test");
            cloud.heightEvidence.available(3)=false;
            [merged,groups]=registrationSupport.canonicalizeSemanticCloud(cloud,1.5);e=merged.heightEvidence;
            testCase.verifyEqual(numel(e.available),numel(groups));
            testCase.verifyEqual(e.scope,"test");
            g=find(cellfun(@(x)any(x==3),groups));
            testCase.verifyFalse(e.available(g));
            testCase.verifyTrue(all(e.available(setdiff(1:numel(groups),g))));
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
    end
end
