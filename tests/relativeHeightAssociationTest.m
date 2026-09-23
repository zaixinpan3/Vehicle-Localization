classdef relativeHeightAssociationTest < matlab.unittest.TestCase
% relativeHeightAssociationTest Height resolves aliases without creating XY force.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function distinguishesPlanarAliases(testCase)
            [fixed,moving,cfg]=scene();
            baseline=cfg;baseline.relativeHeight.enabled=false;
            before=registerSemanticProbabilityCloud(fixed,moving,[-.4 0 0],baseline);
            after=registerSemanticProbabilityCloud(fixed,moving,[-.4 0 0],cfg);
            testCase.verifyEqual(before.poseXYTheta,[-.4 0 0],AbsTol=1e-8);
            testCase.verifyTrue(after.accepted,after.reason);
            testCase.verifyEqual(after.poseXYTheta,[0 0 0],AbsTol=1e-5);
            testCase.verifyTrue(after.height.relativeAssociation.enabled);
            testCase.verifyEqual(after.correspondences.target(after.correspondences.semanticName=="pole"),[7;8]);
        end
        function arbitraryVerticalDatumsCancel(testCase)
            [fixed,moving,cfg]=scene();
            before=registerSemanticProbabilityCloud(fixed,moving,[-.4 0 0],cfg);
            fixed.components.meanXYZ(:,3)=fixed.components.meanXYZ(:,3)+1500;
            moving.components.meanXYZ(:,3)=moving.components.meanXYZ(:,3)-20;
            after=registerSemanticProbabilityCloud(fixed,moving,[-.4 0 0],cfg);
            testCase.verifyEqual(after.poseXYTheta,before.poseXYTheta,AbsTol=1e-10);
            testCase.verifyEqual(after.height.relativeAssociation.offset,1620,AbsTol=1e-9);
        end
        function correctPairsKeepPlanarInformation(testCase)
            [fixed,moving,cfg]=scene();
            baseline=cfg;baseline.relativeHeight.enabled=false;
            before=registerSemanticProbabilityCloud(fixed,moving,[0 0 0],baseline);
            after=registerSemanticProbabilityCloud(fixed,moving,[0 0 0],cfg);
            testCase.verifyEqual(after.poseXYTheta,before.poseXYTheta,AbsTol=0);
            testCase.verifyEqual(after.information,before.information,AbsTol=0);
            testCase.verifyFalse(after.height.relativeAssociation.planarForce);
        end
        function unavailableAnchorHeightUsesExactXyResult(testCase)
            [fixed,moving,cfg]=scene();
            moving.components.heightAvailable(1:4)=false;
            moving.components.meanXYZ(1:4,:)=NaN;
            baseline=cfg;baseline.relativeHeight.enabled=false;
            before=registerSemanticProbabilityCloud(fixed,moving,[-.4 0 0],baseline);
            after=registerSemanticProbabilityCloud(fixed,moving,[-.4 0 0],cfg);
            testCase.verifyFalse(after.height.relativeAssociation.enabled);
            testCase.verifyEqual(after.poseXYTheta,before.poseXYTheta,AbsTol=0);
            testCase.verifyEqual(after.information,before.information,AbsTol=0);
        end
        function anUnavailableCandidateDoesNotRemovePlanarSupport(testCase)
            [fixed,moving,cfg]=scene();
            fixed.components.heightAvailable(7)=false;
            fixed.components.meanXYZ(7,:)=NaN;
            result=registerSemanticProbabilityCloud(fixed,moving,[0 0 0],cfg);
            testCase.verifyTrue(result.accepted,result.reason);
            testCase.verifyEqual(result.poseXYTheta,[0 0 0],AbsTol=1e-8);
            testCase.verifyEqual(result.correspondences.heightAssociationCost(result.correspondences.target==7),0,AbsTol=0);
        end
        function currentHeightDoesNotMixDifferentVerticalOrigins(testCase)
            [~,old,~]=scene();
            current=old;current.components.meanXYZ(:,3)=current.components.meanXYZ(:,3)+2;
            [~,history]=updateLocalizationSourceWindow(old,0,[0 0 0],[]);
            [pooled,~,~,confirmed]=updateLocalizationSourceWindow(current,.1,[0 0 0],history);
            testCase.verifyEqual(pooled.components.mean,current.components.mean,AbsTol=1e-12);
            testCase.verifyEqual(pooled.heightEvidence.mean,current.components.meanXYZ,AbsTol=0);
            testCase.verifyEqual(confirmed.heightEvidence.mean,current.components.meanXYZ,AbsTol=0);
            testCase.verifyEqual(pooled.heightEvidence.scope,"currentAcquisitionOnly");
        end
        function missingCurrentScanDoesNotReuseStaleHeight(testCase)
            [~,cloud,~]=scene();
            [~,history]=updateLocalizationSourceWindow(cloud,0,[0 0 0],[]);
            [~,history]=updateLocalizationSourceWindow(cloud,.1,[0 0 0],history);
            missing=localizationSourceWindowTest.empty(cloud);
            [pooled,~]=updateLocalizationSourceWindow(missing,.2,[0 0 0],history);
            testCase.verifyEqual(pooled.components.numComponents,6);
            testCase.verifyFalse(any(pooled.heightEvidence.available));
        end
        function reorderedCurrentObservationsKeepTrackHeightAligned(testCase)
            [~,cloud,~]=scene();current=subsetScene(cloud,6:-1:1);
            [~,history]=updateLocalizationSourceWindow(cloud,0,[0 0 0],[]);
            [pooled,~]=updateLocalizationSourceWindow(current,.1,[0 0 0],history);
            testCase.verifyEqual(pooled.heightEvidence.mean,cloud.components.meanXYZ,AbsTol=0);
        end
        function partialVerticalVisibilityDoesNotDiscardCorrectSupport(testCase)
            [fixed,moving,cfg]=scene();fixed=subsetScene(fixed,[1:4 7:8]);
            moving.components.meanXYZ(5:6,3)=moving.components.meanXYZ(5:6,3)+2;
            moving.components.covarianceXYZ(3,3,5:6)=4;
            result=registerSemanticProbabilityCloud(fixed,moving,[0 0 0],cfg);
            testCase.verifyTrue(result.accepted,result.reason);
            testCase.verifyEqual(result.poseXYTheta,[0 0 0],AbsTol=1e-8);
            testCase.verifyEqual(height(result.correspondences),6);
        end
        function croppingKeepsHeightIndicesAligned(testCase)
            [fixed,~,~]=scene();
            [local,ids]=selectLocalProbabilityCloud(fixed,[2 3 0],.1);
            testCase.verifyEqual(ids,8);
            testCase.verifyEqual(local.components.meanXYZ,fixed.components.meanXYZ(8,:),AbsTol=0);
            testCase.verifyEqual(local.components.covarianceXYZ,fixed.components.covarianceXYZ(:,:,8),AbsTol=0);
        end
        function unselectedClassKeepsPlanarAssociation(testCase)
            [fixed,moving,cfg]=scene();cfg.relativeHeight.semanticNames="trafficSign";
            result=registerSemanticProbabilityCloud(fixed,moving,[-.4 0 0],cfg);
            testCase.verifyEqual(result.poseXYTheta,[-.4 0 0],AbsTol=1e-8);
            testCase.verifyTrue(result.height.relativeAssociation.enabled);
            testCase.verifyEqual(result.correspondences.heightAssociationCost,zeros(6,1),AbsTol=0);
        end
        function cropDoesNotClaimWholeMapNormalization(testCase)
            [fixed,moving,cfg]=scene();
            fixed.queryRelationship="exactIntensityNormalization";
            fixed.totalMass=80;fixed.components.mass=10*ones(8,1);
            [local,ids]=selectLocalProbabilityCloud(fixed,[0 0 0],6);
            result=registerSemanticProbabilityCloud(local,moving,[0 0 0],cfg);
            testCase.verifyTrue(result.accepted,result.reason);
            testCase.verifyFalse(isfield(local,'queryRelationship'));
            testCase.verifyEqual(local.components.mixtureWeight,fixed.components.mixtureWeight(ids),AbsTol=0);
        end
        function marginalHeightDoesNotExtrapolateVerticalStructure(testCase)
            [fixed,moving,cfg]=scene();
            fixed.components.covarianceXYZ(:,:,7)=[.02 0 .2;0 .02 0;.2 0 4];
            pairs=struct('source',(1:4).','target',(1:4).','semanticName',repmat("curb",4,1));
            model=prepareRelativeHeightAssociation(fixed,moving,[0 0 0],pairs,cfg.relativeHeight);
            [nearCost,nearResidual]=model.cost(5,7,[0 0 0]);
            [shiftedCost,shiftedResidual]=model.cost(5,7,[1 0 0]);
            testCase.verifyEqual(nearResidual,0,AbsTol=1e-10);
            testCase.verifyEqual(shiftedResidual,nearResidual,AbsTol=1e-10);
            testCase.verifyEqual(shiftedCost,nearCost,AbsTol=1e-10);
        end
        function conflictingHeightModesAreRejected(testCase)
            [fixed,moving,cfg]=scene();cfg.heightMode="xyz";cfg.heightTranslation=100;
            testCase.verifyError(@()registerSemanticProbabilityCloud(fixed,moving,[0 0 0],cfg), ...
                'VehicleLocalization:ConflictingHeightModes');
        end
    end
end

function [fixed,moving,cfg]=scene()
    means=[-6 0;-2 0;2 0;6 0;-2 3;2 3];
    covariance=cat(3,repmat(diag([.5 .005]),1,1,4),repmat(.02*eye(2),1,1,2));
    spatial=zeros(3,3,6);spatial(1:2,1:2,:)=covariance;spatial(3,3,:)=.02;
    c=struct('mean',means,'covariance',covariance, ...
        'semanticName',[repmat("curb",4,1);"pole";"pole"], ...
        'mixtureWeight',ones(6,1)/6,'numComponents',6, ...
        'meanXYZ',[means,[zeros(4,1);1;4]],'covarianceXYZ',spatial,'heightAvailable',true(6,1));
    moving=struct('components',c,'frameCalibration',lidarFrameCalibrationConfig());
    ids=[1:6 5 6];f=c;
    f.mean=c.mean(ids,:);f.covariance=c.covariance(:,:,ids);
    f.semanticName=c.semanticName(ids);f.mixtureWeight=ones(8,1)/8;f.numComponents=8;
    f.meanXYZ=c.meanXYZ(ids,:);f.meanXYZ(:,3)=f.meanXYZ(:,3)+100;
    f.covarianceXYZ=c.covarianceXYZ(:,:,ids);f.heightAvailable=true(8,1);
    f.mean(5:6,1)=f.mean(5:6,1)-.4;f.meanXYZ(5:6,1)=f.mean(5:6,1);
    f.meanXYZ(5:6,3)=f.meanXYZ(5:6,3)+3;
    fixed=struct('components',f,'frameCalibration',moving.frameCalibration);
    cfg=distributionRegistrationConfig();cfg.relativeHeight.enabled=true;
end

function cloud=subsetScene(cloud,ids)
    c=cloud.components;
    for field=["mean","meanXYZ","semanticName","mixtureWeight","heightAvailable"]
        c.(field)=c.(field)(ids,:);
    end
    c.covariance=c.covariance(:,:,ids);c.covarianceXYZ=c.covarianceXYZ(:,:,ids);
    c.numComponents=numel(ids);cloud.components=c;
end
