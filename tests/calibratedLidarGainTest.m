classdef calibratedLidarGainTest < matlab.unittest.TestCase
% calibratedLidarGainTest Physical weighting, nullspaces and continuous proof.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function averageAccuracyGetsFullSupportedAuthority(testCase)
            cfg=config();a=filterLidarPoseInformation(diag([1,3,90]),cfg,1,ones(3,1),[.01;.001]);
            testCase.verifyEqual(a.S,eye(3),AbsTol=1e-12);
            testCase.verifyTrue(a.calibrated);
        end
        function fourTimesErrorVarianceGetsQuarterAuthority(testCase)
            cfg=config();a=filterLidarPoseInformation(eye(3),cfg,1,ones(3,1),[.04;.004]);
            testCase.verifyEqual(a.S,.25*eye(3),AbsTol=1e-12);
        end
        function weakGlobalLossScaleDoesNotChangeCorrection(testCase)
            cfg=config();H=[2,.1,.4;.1,3,.5;.4,.5,8];g=[1;2;3];
            a=filterLidarPoseInformation(H,cfg,1,ones(3,1),[.02;.001]);
            b=filterLidarPoseInformation(1e-12*H,cfg,1,ones(3,1),[.02;.001]);
            testCase.verifyEqual(a.S,b.S,AbsTol=1e-12);
            testCase.verifyEqual(a.F*g,b.F*(1e-12*g),AbsTol=1e-11);
        end
        function differentPositionAndYawWeightsPreserveAffineIdentity(testCase)
            cfg=config();H=[2,.1,.4;.1,3,.5;.4,.5,8];
            a=filterLidarPoseInformation(H,cfg,1,ones(3,1),[.04;.002]);
            testCase.verifyEqual(a.F*H,a.S,AbsTol=1e-12);
            testCase.verifyGreaterThan(norm(a.F-a.F.'),1e-4);
        end
        function obliqueNullspaceIsPreserved(testCase)
            cfg=config();n=[0;-3;1];J=[1,0,0;0,1,3];H=J.'*J;
            a=filterLidarPoseInformation(H,cfg,1,ones(3,1),[.03;.001]);
            testCase.verifyEqual(a.S*n,zeros(3,1),AbsTol=1e-12);
            testCase.verifyEqual(a.F*H,a.S,AbsTol=1e-12);
            testCase.verifyGreaterThanOrEqual(min(eig(a.S)),-1e-12);
            testCase.verifyLessThanOrEqual(max(eig(a.S)),1+1e-12);
        end
        function rejectedAndEmptyChannelsRemainZero(testCase)
            cfg=config();a=filterLidarPoseInformation(zeros(3),cfg);
            b=filterLidarPoseInformation(eye(3),cfg,0,ones(3,1),[.01;.001]);
            testCase.verifyEqual(a.S,zeros(3),AbsTol=0);
            testCase.verifyEqual(b.S,zeros(3),AbsTol=0);
        end
        function implicitJumpRemainsNonexpansive(testCase)
            cfg=config();d=designFullObserverGains(cfg);G=zeros(3,7);G(:,[1,4,7])=eye(3);
            m=struct('pose',zeros(3,1),'information',[2,.1,.4;.1,3,.5;.4,.5,8], ...
                'poseErrorVariance',[.04;.002]);
            [~,a]=computeLidarMatchedCorrection(m,[1;2;.1],G,d.lidarMatched,cfg, ...
                StepSize=10,Discretization="implicit");
            testCase.verifyTrue(a.jumpNonexpansive);
            testCase.verifyTrue(d.fullContinuousStateCertified);
            testCase.verifyFalse(a.filter.regularizer>0);
        end
        function sandwichPredictorIsInvariantToLossScale(testCase)
            m=model();a=lidarPoseErrorFeatures(m);m.rowInfluence=100*m.rowInfluence;
            b=lidarPoseErrorFeatures(m);
            testCase.verifyEqual(a,b,RelTol=1e-10,AbsTol=1e-14);
            testCase.verifyGreaterThan(a,zeros(2,1));
        end
        function calibratedPosePacketsReachThePublicRuntime(testCase)
            [data,lateral,cfg]=posePackets();
            r=runFullLocalizationObserver(data,struct(),cfg,LateralInputs=lateral);
            testCase.verifyEqual(r.pose(2,:),[1/2.6,1/2.6,.1/1.2],AbsTol=1e-12);
        end
        function missingVarianceCannotSilentlyUseGeometricWeights(testCase)
            [data,lateral,cfg]=posePackets();data.lidar=rmfield(data.lidar,'poseErrorVariance');
            testCase.verifyError(@()runFullLocalizationObserver(data,struct(),cfg,LateralInputs=lateral), ...
                'VehicleLocalization:MissingLidarErrorCalibration');
        end
        function residualOverrideCannotReuseAnUnrelatedCalibration(testCase)
            [data,lateral,cfg]=posePackets();data.lidar=rmfield(data.lidar,'poseErrorVariance');
            data.lidar.residualModels={model();model()};
            data.lidar.evaluateResidual=@(~,pose)struct('residual',pose,'jacobian',eye(3), ...
                'weights',ones(3,1),'linearizationPose',pose);
            testCase.verifyError(@()runFullLocalizationObserver(data,struct(),cfg,LateralInputs=lateral), ...
                'VehicleLocalization:MissingLidarErrorCalibration');
        end
        function namedProfileRecomputesItsContinuousCertificate(testCase)
            cfg=mncavFullObserverConfig(GainProfile="mississippi-20240607-calibrated");
            d=designFullObserverGains(cfg);
            testCase.verifyFalse(isfield(cfg.lidar,'gainInformationScale'));
            testCase.verifyTrue(d.continuousCertificate.certified);
            testCase.verifyGreaterThan(d.continuousCertificate.verification.verifiedDecayRate,cfg.iss.decayRate);
        end
        function empiricalFloorIncludesUnexplainedError(testCase)
            cfg=config();a=predictLidarPoseErrorVariance(zeros(2,1),cfg.lidar.errorCalibration);
            testCase.verifyEqual(a,[.01;.001],AbsTol=0);
        end
        function legacyScaleCannotSurviveInCalibratedConfiguration(testCase)
            cfg=config();cfg.lidar.gainInformationScale=16;
            testCase.verifyError(@()fullObserverIssProblem(cfg),'VehicleLocalization:AmbiguousLidarCalibration');
        end
        function changedCalibrationInvalidatesContinuousCertificate(testCase)
            cfg=config();s=solveFullObserverIssLmi(cfg);
            cfg.lidar.errorCalibration.referenceVariance=2*cfg.lidar.errorCalibration.referenceVariance;
            testCase.verifyError(@()verifyFullObserverIssCertificate(s,cfg), ...
                'VehicleLocalization:StaleFullIssCertificate');
        end
    end
end

function [data,lateral,cfg]=posePackets()
    cfg=config();cfg.initialState=[1;0;0;1;0;0;.1];t=[0;.1];z=zeros(2,1);
    data.highRate=struct('time',t,'longitudinalSpeed',z,'longitudinalAcceleration',z, ...
        'lateralAcceleration',z,'yawRate',z);
    data.lidar=struct('time',t,'delay',0,'pose',zeros(2,3),'valid',true(2,1), ...
        'information',repmat(eye(3),1,1,2),'poseErrorVariance',repmat([.01,.001],2,1));
    lateral=struct('time',t,'lateralVelocity',z,'sideSlipAngleRate',z);
end

function cfg=config()
    cfg=mncavFullObserverConfig();cfg.lidar=rmfield(cfg.lidar,'gainInformationScale');
    cfg.lidar.errorCalibration=struct('kind',"conditional-pose-second-moment-v1", ...
        'referenceVariance',[.01;.001],'coefficients',[.01,1;.001,1]);
    cfg.iss.minimumPositionStrength=.4;cfg.iss.minimumHeadingStrength=.1;
    cfg.iss.maximumPositionHeadingCoupling=0;cfg.gains([1,4])=[16,2];
end

function m=model()
    source=[0,0;4,0;0,3;4,3];target=source+[.1,.03;-.04,.1;.08,-.02;-.02,.06];
    m=struct('kind',"frozen-semantic-lidar-v1",'method',"geometricD2D", ...
        'origin',[0,0],'anchorPose',[0,0,0],'poseProjector',eye(3),'poseSpread',zeros(3), ...
        'sourceMean',source,'targetMean',target,'rowInfluence',ones(4,1)/4, ...
        'precisionRoot',repmat(eye(2),1,1,4),'sourceAxis',zeros(4,2), ...
        'targetNormal',zeros(4,2),'directionScale',zeros(4,1),'conditionedOnPositionAid',false);
end
