classdef mncavLidarCertificateTest < matlab.unittest.TestCase
    % Check continuum enclosure and reject unsupported certificate reuse.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function recordedRateFitsTheNewDesignRange(testCase)
            cfg=improvedObserverConfig("lidar","mncav");d=improvedObserverReferenceDesign(cfg);
            v=verifyImprovedObserverDesign(d,cfg);
            testCase.verifyGreaterThan(cfg.operating.maximumTrackAngleRate,.354975175657248);
            testCase.verifyEqual(cfg.measurement.fixedLidarDelay,.15);
            testCase.verifyGreaterThan(v.uniformMargin,.08);
            testCase.verifySize(v.details.vertexMargins,[4,1]);
        end
        function verticesEncloseTheActualScaledModel(testCase)
            cfg=improvedObserverConfig("lidar","mncav");d=improvedObserverReferenceDesign(cfg);
            [vertices,~]=continuousLidarCertificateVertices(d,cfg);
            data=buildImprovedObserverCertificateData(cfg);b=cfg.operating.maximumTrackAngleRate;
            sample=struct('longitudinalSpeed',8,'lateralVelocity',0, ...
                'longitudinalAcceleration',0,'lateralAcceleration',0, ...
                'yawRate',0,'sideSlipAngle',0,'sideSlipAngleRate',0);
            for q=linspace(-b,b,101)
                sample.yawRate=q;channels=evaluateImprovedObserverChannels(zeros(7,1),sample);
                a=(q+b)/(2*b);c=q^2/b^2;
                weights=[(1-a)*(1-c),(1-a)*c,a*(1-c),a*c];
                enclosure=sum(vertices.*reshape(weights,1,1,4),3);
                testCase.verifyEqual(enclosure,data.T\channels.modelMatrix*data.T,AbsTol=1e-12);
            end
        end
        function broaderBoundCannotReuseSuccessFlag(testCase)
            cfg=improvedObserverConfig("lidar","mncav");d=improvedObserverReferenceDesign(cfg);
            cfg.operating.maximumTrackAngleRate=.8;
            testCase.verifyFalse(verifyImprovedObserverDesign(d,cfg).certified);
        end
        function delayAndInformationChangesAreRechecked(testCase)
            cfg=improvedObserverConfig("lidar","mncav");d=improvedObserverReferenceDesign(cfg);
            cfg.measurement.fixedLidarDelay=.5;
            testCase.verifyFalse(verifyImprovedObserverDesign(d,cfg).certified);
            cfg.measurement.fixedLidarDelay=.15;cfg.lidar.minimumPoseWeight=.5;
            testCase.verifyFalse(verifyImprovedObserverDesign(d,cfg).certified);
        end
        function legacyNormProofDoesNotCoverTheNewBound(testCase)
            cfg=improvedObserverConfig("lidar","mncav");d=improvedObserverReferenceDesign(cfg);
            d=rmfield(d,'certificateMethod');
            testCase.verifyFalse(verifyImprovedObserverDesign(d,cfg).certified);
        end
        function unknownMethodIsRejected(testCase)
            cfg=improvedObserverConfig("lidar","mncav");d=improvedObserverReferenceDesign(cfg);
            d.certificateMethod="unchecked";
            testCase.verifyError(@() verifyImprovedObserverDesign(d,cfg), ...
                'VehicleLocalization:CertificateMismatch');
        end
        function gnssCannotSelectLidarCertificate(testCase)
            testCase.verifyError(@() improvedObserverConfig("gnss","mncav"), ...
                'VehicleLocalization:UnsupportedObserverProfile');
        end
    end
end
