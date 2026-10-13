classdef fullObserverContinuousIssTest < matlab.unittest.TestCase
% Continuous intrinsic gain checks; numerical and actual-overlap scope differ.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));setupVehicleLocalization;
        end
    end
    methods (Test)
        function currentVehicleHasAnIntrinsicSurrogateCertificate(testCase)
            d=fullObserverSupport.designFullObserverGains(mncavFullObserverConfig());
            testCase.verifyEqual(d.framework,"invariant");
            testCase.verifyTrue(d.surrogateContinuousLmiVerified);
            testCase.verifyFalse(d.fullContinuousStateCertified);
            testCase.verifyLessThan(d.continuousCertificate.verification.worstVertexEigenvalue,0);
            testCase.verifyFalse(d.baselineFullStateCertified);
        end
        function timingDoesNotSelectAnotherGainDesign(testCase)
            c=mncavFullObserverConfig();a=fullObserverSupport.designFullObserverGains(c);
            b=fullObserverSupport.designFullObserverGains(rmfield(c,'timing'));
            testCase.verifyEqual(a,b);
            c.timing="historical_transport";
            testCase.verifyError(@()fullObserverSupport.designFullObserverGains(c),'VehicleLocalization:ObsoleteFullConfig');
        end
        function retiredConfigurationsCannotSelectAnAlternative(testCase)
            c=struct('kind',"old-seven-state",'gains',[4,4,12,4]);
            testCase.verifyError(@()fullObserverSupport.designFullObserverGains(c),'VehicleLocalization:RemovedFullObserverConfig');
            c=mncavFullObserverConfig();c.hgo=struct();
            testCase.verifyError(@()fullObserverSupport.designFullObserverGains(c),'VehicleLocalization:RemovedFullObserverConfig');
        end
        function changedNoiseCannotLoadSavedGains(testCase)
            c=mncavFullObserverConfig();noise=c.invariantObserver.noise;noise.gyroRadps=2*noise.gyroRadps;
            testCase.verifyError(@()mncavFullObserverConfig(DesignOverride=struct('noise',noise)), ...
                'VehicleLocalization:StaleInvariantDesign');
        end
        function solverClaimCannotReplaceNumericVerification(testCase)
            c=mncavFullObserverConfig();h=c.invariantDesign;h.kappaL=0;h.verification.certified=true;
            v=invariantObserverSupport.verify(h,c);testCase.verifyFalse(v.certified);
            c.invariantDesign=h;
            testCase.verifyError(@()fullObserverSupport.designFullObserverGains(c),'VehicleLocalization:InfeasibleInvariantDesign');
        end
        function missingDomainCannotBypassVerification(testCase)
            c=rmfield(mncavFullObserverConfig(),'iss');
            testCase.verifyError(@()fullObserverSupport.designFullObserverGains(c),'VehicleLocalization:MissingFullIssDomain');
        end
        function insufficientGeometryRemainsZeroInformation(testCase)
            c=mncavFullObserverConfig();c.initialState=[.2;8;0;-.1;0;0;.02];t=[0;.1];z=zeros(2,1);
            data=struct('highRate',struct('time',t,'longitudinalSpeed',8+z, ...
                'longitudinalAcceleration',z,'lateralAcceleration',z,'yawRate',z));
            data.lidar=struct('time',t,'pose',zeros(2,3),'valid',true(2,1), ...
                'information',zeros(3,3,2),'delay',0);
            lat=struct('time',t,'lateralVelocity',z,'sideSlipAngleRate',z);
            r=runFullLocalizationObserver(data,struct(),c,LateralInputs=lat);
            testCase.verifyTrue(r.diagnostics.continuousLmiVerified);
            testCase.verifyFalse(any(r.diagnostics.continuousInformationQualified));
            testCase.verifyEqual(r.diagnostics.poseWeight,zeros(3,3,2));
            testCase.verifyFalse(r.diagnostics.allTheoremHypothesesVerified);
        end
    end
end
