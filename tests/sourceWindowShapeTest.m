classdef sourceWindowShapeTest < matlab.unittest.TestCase
% sourceWindowShapeTest Joint center/shape consistency before temporal fusion.
    methods (TestClassSetup)
        function paths(~),setupVehicleLocalization();end
    end
    methods (Test)
        function colocatedOrthogonalAnisotropicCloudsAreNotConfirmed(t)
            a=sourceWindowShapeTest.cloud(diag([.04 .001]));b=sourceWindowShapeTest.cloud(diag([.001 .04]));
            [~,h]=updateLocalizationSourceWindow(a,0,[0 0 0],[]);
            [actual,~,details,current]=updateLocalizationSourceWindow(b,.1,[0 0 0],h);
            t.verifyEmpty(actual.components.mean);
            t.verifyEmpty(current.components.mean);
            t.verifyEqual(details.shapeRejectedPairs,1);
        end
        function largeScaleConflictCannotUseTheSameCenter(t)
            a=sourceWindowShapeTest.cloud(.002*eye(2));b=sourceWindowShapeTest.cloud(.2*eye(2));
            [~,h]=updateLocalizationSourceWindow(a,0,[0 0 0],[]);
            [actual,~,d]=updateLocalizationSourceWindow(b,.1,[0 0 0],h);
            t.verifyEmpty(actual.components.mean);
            t.verifyEqual(d.shapeRejectedPairs,1);
        end
        function moderateShapeVariationStillConfirms(t)
            a=sourceWindowShapeTest.cloud(diag([.04 .003]));b=sourceWindowShapeTest.cloud(diag([.05 .004]));
            [~,h]=updateLocalizationSourceWindow(a,0,[0 0 0],[]);
            [actual,~,d]=updateLocalizationSourceWindow(b,.1,[0 0 0],h);
            t.verifyEqual(actual.components.detectionFrameCount,2);
            t.verifyEqual(actual.components.covariance,(a.components.covariance+b.components.covariance)/2,AbsTol=1e-14);
            t.verifyEqual(d.shapeRejectedPairs,0);
        end
        function equivalentShapeAfterOdometryRotationConfirms(t)
            a=sourceWindowShapeTest.cloud(diag([.04 .001]));b=sourceWindowShapeTest.cloud(diag([.001 .04]));
            [~,h]=updateLocalizationSourceWindow(a,0,[0 0 0],[]);
            [actual,~]=updateLocalizationSourceWindow(b,.1,[0 0 pi/2],h);
            t.verifyEqual(actual.components.detectionFrameCount,2);
            t.verifyEqual(actual.components.covariance,b.components.covariance,AbsTol=1e-14);
        end
        function nearlyRoundShapesDoNotInventAnAxisConflict(t)
            a=sourceWindowShapeTest.cloud(diag([.01 .009]));b=sourceWindowShapeTest.cloud(diag([.009 .01]));
            [~,h]=updateLocalizationSourceWindow(a,0,[0 0 0],[]);
            [actual,~]=updateLocalizationSourceWindow(b,.1,[0 0 0],h);
            t.verifyEqual(actual.components.detectionFrameCount,2);
        end
        function earlierAcquisitionPreventsShapeBridging(t)
            a=sourceWindowShapeTest.cloud(diag([.04 .001]));
            b=sourceWindowShapeTest.rotated(a,pi/4);c=sourceWindowShapeTest.rotated(a,pi/2);
            cfg=localizationSourceWindowConfig();cfg.maximumShapeDistance=.75;
            [~,h]=updateLocalizationSourceWindow(a,0,[0 0 0],[],cfg);
            [before,h]=updateLocalizationSourceWindow(b,.1,[0 0 0],h,cfg);
            [after,~,d,current]=updateLocalizationSourceWindow(c,.2,[0 0 0],h,cfg);
            t.verifyEqual(before.components.detectionFrameCount,2);
            t.verifyEqual(after.components.detectionFrameCount,2);
            t.verifyEmpty(current.components.mean);
            t.verifyEqual(d.shapeRejectedPairs,1);
        end
        function shapeCompatibilityBreaksAPositionTie(t)
            a=sourceWindowShapeTest.cloud(diag([.04 .001]));a.components.mean=zeros(2,2);
            a.components.covariance(:,:,2)=diag([.001 .04]);a.components.semanticName=["trafficSign";"trafficSign"];
            a.components.mixtureWeight=[.5;.5];a.components.numComponents=2;
            b=sourceWindowShapeTest.cloud(diag([.001 .04]));
            [~,h]=updateLocalizationSourceWindow(a,0,[0 0 0],[]);
            [actual,~]=updateLocalizationSourceWindow(b,.1,[0 0 0],h);
            t.verifyEqual(actual.components.covariance,b.components.covariance,AbsTol=1e-14);
            t.verifyEqual(actual.components.detectionFrameCount,2);
        end
        function compatibleShapeCannotOverridePositionFailure(t)
            a=sourceWindowShapeTest.cloud(diag([.04 .001]));b=a;b.components.mean=[1 0];
            [~,h]=updateLocalizationSourceWindow(a,0,[0 0 0],[]);
            [actual,~]=updateLocalizationSourceWindow(b,.1,[0 0 0],h);
            t.verifyEmpty(actual.components.mean);
        end
        function invalidShapeConfigurationIsRejected(t)
            a=sourceWindowShapeTest.cloud(diag([.04 .001]));cfg=localizationSourceWindowConfig();cfg.maximumShapeDistance=NaN;
            t.verifyError(@()updateLocalizationSourceWindow(a,0,[0 0 0],[],cfg),'VehicleLocalization:InvalidWindowConfiguration');
        end
        function zeroShapeRegularizationIsRejected(t)
            a=sourceWindowShapeTest.cloud(diag([.04 .001]));cfg=localizationSourceWindowConfig();cfg.shapeVarianceFloor=0;
            t.verifyError(@()updateLocalizationSourceWindow(a,0,[0 0 0],[],cfg),'VehicleLocalization:InvalidWindowConfiguration');
        end
    end
    methods (Static)
        function a=cloud(covariance)
            a=struct('dimension',2,'frameCalibration',lidarFrameCalibrationConfig(), ...
                'components',struct('mean',[0 0],'covariance',covariance,'semanticName',"trafficSign", ...
                'mixtureWeight',1,'numComponents',1));
        end
        function a=rotated(a,angle)
            r=[cos(angle) -sin(angle);sin(angle) cos(angle)];
            a.components.covariance=r*a.components.covariance*r.';
        end
    end
end
