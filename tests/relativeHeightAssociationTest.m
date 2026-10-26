classdef relativeHeightAssociationTest < matlab.unittest.TestCase
% relativeHeightAssociationTest Source-window height stays tied to its acquisition.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));
            setupVehicleLocalization;
        end
    end
    methods (Test)
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
        function croppingKeepsHeightIndicesAligned(testCase)
            [fixed,~,~]=scene();
            [local,ids]=registrationSupport.selectLocalProbabilityCloud(fixed,[2 3 0],.1);
            testCase.verifyEqual(ids,8);
            testCase.verifyEqual(local.components.meanXYZ,fixed.components.meanXYZ(8,:),AbsTol=0);
            testCase.verifyEqual(local.components.covarianceXYZ,fixed.components.covarianceXYZ(:,:,8),AbsTol=0);
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
    cfg=semanticGeometryConfig();
end

function cloud=subsetScene(cloud,ids)
    c=cloud.components;
    for field=["mean","meanXYZ","semanticName","mixtureWeight","heightAvailable"]
        c.(field)=c.(field)(ids,:);
    end
    c.covariance=c.covariance(:,:,ids);c.covarianceXYZ=c.covarianceXYZ(:,:,ids);
    c.numComponents=numel(ids);cloud.components=c;
end
