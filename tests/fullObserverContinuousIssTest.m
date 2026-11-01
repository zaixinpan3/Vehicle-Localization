classdef (SharedTestFixtures={freshObserverGainsFixture}) fullObserverContinuousIssTest < matlab.unittest.TestCase
% Continuous intrinsic gain checks; numerical and actual-overlap scope differ.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));setupVehicleLocalization;
        end
    end
    methods (Test)
        function currentVehicleHasAnIntrinsicSurrogateCertificate(testCase)
            d=localizationSupport.designFullObserverGains(freshObserverTestConfig());
            testCase.verifyEqual(d.framework,"invariant");
            testCase.verifyTrue(d.surrogateContinuousLmiVerified);
            testCase.verifyTrue(d.nominalFusionLmiVerified);
            testCase.verifyFalse(d.fullContinuousStateCertified);
            testCase.verifyLessThan(d.continuousCertificate.verification.worstVertexEigenvalue,0);
            testCase.verifyFalse(d.baselineFullStateCertified);
        end
        function timingDoesNotSelectAnotherGainDesign(testCase)
            c=freshObserverTestConfig();a=localizationSupport.designFullObserverGains(c);
            b=localizationSupport.designFullObserverGains(rmfield(c,'timing'));
            testCase.verifyEqual(a,b);
            c.timing="historical_transport";
            testCase.verifyError(@()localizationSupport.designFullObserverGains(c),'VehicleLocalization:ObsoleteFullConfig');
        end
        function retiredConfigurationsCannotSelectAnAlternative(testCase)
            c=struct('kind',"old-seven-state",'gains',[4,4,12,4]);
            testCase.verifyError(@()localizationSupport.designFullObserverGains(c),'VehicleLocalization:RemovedFullObserverConfig');
            c=freshObserverTestConfig();c.hgo=struct();
            testCase.verifyError(@()localizationSupport.designFullObserverGains(c),'VehicleLocalization:RemovedFullObserverConfig');
        end
        function changedNoiseCannotReuseTemporaryGains(testCase)
            c=freshObserverTestConfig();c.invariantObserver.noise.gyroRadps=2*c.invariantObserver.noise.gyroRadps;
            testCase.verifyError(@()localizationSupport.runtime(c), ...
                'VehicleLocalization:StaleInvariantDesign');
        end
        function solverClaimCannotReplaceNumericVerification(testCase)
            c=freshObserverTestConfig();h=c.invariantDesign;h.kappaL=0;h.verification.certified=true;
            v=localizationSupport.verifyInvariantDesign(h,c);testCase.verifyFalse(v.certified);
            testCase.verifyFalse(v.numericalInvariantRadiusCertified);
            testCase.verifyFalse(v.operatingSet.invarianceVerified);
            c.invariantDesign=h;
            testCase.verifyError(@()localizationSupport.designFullObserverGains(c),'VehicleLocalization:InfeasibleInvariantDesign');
        end
        function missingDomainCannotBypassVerification(testCase)
            c=rmfield(freshObserverTestConfig(),'iss');
            testCase.verifyError(@()localizationSupport.designFullObserverGains(c),'VehicleLocalization:MissingFullIssDomain');
        end
        function insufficientGeometryRemainsZeroInformation(testCase)
            c=freshObserverTestConfig();c.initialState=[.2;8;0;-.1;0;0;.02];t=[0;.1];z=zeros(2,1);
            data=struct('highRate',struct('time',t,'longitudinalSpeed',8+z, ...
                'longitudinalAcceleration',z,'lateralAcceleration',z,'yawRate',z));
            data.lidar=struct('time',t,'pose',zeros(2,3),'valid',true(2,1), ...
                'information',zeros(3,3,2),'delay',0);
            lat=struct('time',t,'lateralVelocity',z,'sideSlipAngle',z,'sideSlipAngleRate',z);
            r=runFullLocalizationObserver(data,struct(),c,LateralInputs=lat);
            testCase.verifyTrue(r.diagnostics.continuousLmiVerified);
            testCase.verifyFalse(any(r.diagnostics.continuousInformationQualified));
            testCase.verifyEqual(r.diagnostics.poseWeight,zeros(3,3,2));
            testCase.verifyFalse(r.diagnostics.allTheoremHypothesesVerified);
        end
    end
end
