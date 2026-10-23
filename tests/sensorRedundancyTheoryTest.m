classdef (SharedTestFixtures={freshObserverGainsFixture}) sensorRedundancyTheoryTest < matlab.unittest.TestCase
% Common continuous-theory inequalities and the position-only observability limit.
    properties (Access=private)
        TheoryReport
    end
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));setupVehicleLocalization;
            testCase.TheoryReport=verifySensorRedundancyTheory(Config=freshObserverTestConfig());
        end
    end
    methods (Test)
        function oneWitnessCoversBothMissingSensorCases(testCase)
            r=testCase.TheoryReport;
            testCase.verifyTrue(r.commonLmiVerified);
            testCase.verifyTrue(r.coversGnssAbsent);
            testCase.verifyTrue(r.coversZeroLidarInformation);
            testCase.verifyEqual(r.lidarVertices,192);
            testCase.verifyEqual(r.gnssBernsteinVertices,8640);
            testCase.verifyTrue(r.coversComplementaryInformationAtRest);
            testCase.verifyEqual(r.mixedVertices,1152);
            testCase.verifyLessThan(r.worstMixedEigenvalue,0);
            testCase.verifyLessThan(max(r.worstLidarEigenvalue,r.worstGnssEigenvalue),-.009);
        end
        function zeroMotionCannotBeAdmittedAsFullStateGnssAdequacy(testCase)
            r=testCase.TheoryReport;w=r.witness;w.domain.minimumGnssSpeed=0;
            testCase.verifyError(@()verifySensorRedundancyTheory(Witness=w,Config=freshObserverTestConfig()), ...
                'VehicleLocalization:UnobservableGnssHeading');
        end
        function temporaryWitnessVerificationNeedsNoSdpSolver(testCase)
            c=freshObserverTestConfig();
            originalPath=path;cleanup=onCleanup(@()path(originalPath));
            root=fileparts(fileparts(mfilename('fullpath')));restoredefaultpath;addpath(root);setupVehicleLocalization;
            testCase.verifyEqual(exist('sdpvar','file'),0);
            r=verifySensorRedundancyTheory(Witness=testCase.TheoryReport.witness,Config=c);
            testCase.verifyTrue(r.commonLmiVerified);
        end
        function changedManeuverInputsRejectTheWitness(testCase)
            r=testCase.TheoryReport;w=r.witness;w.inputs.maximumSpeed=17;
            testCase.verifyError(@()verifySensorRedundancyTheory(Witness=w,Config=freshObserverTestConfig()), ...
                'VehicleLocalization:StaleRedundancyWitness');
        end
        function losingTheDirectionChannelInvalidatesTheGnssProof(testCase)
            r=testCase.TheoryReport;w=r.witness;w.directionGain=1e-10;
            testCase.verifyError(@()verifySensorRedundancyTheory(Witness=w,Config=freshObserverTestConfig()), ...
                'VehicleLocalization:UnverifiedRedundancyTheory');
        end
        function parkedHeadingsHaveIdenticalAvailableOutputs(testCase)
            c=freshObserverTestConfig();d=c.gnss.outputPoint.bodyOffset(:);
            psi=.2;R=[cos(psi),-sin(psi);sin(psi),cos(psi)];
            a=zeros(8,1);a([1,4])=-d;
            b=zeros(8,1);b([1,4])=-R*d;b(7)=psi;
            testCase.verifyEqual(a([1,4])+d,b([1,4])+R*d,AbsTol=1e-12);
            testCase.verifyEqual(invariantObserverSupport.outputs(a,0,0,c), ...
                invariantObserverSupport.outputs(b,0,0,c),AbsTol=1e-12);
            testCase.verifyEqual(invariantObserverSupport.predict(a,.1,0),a,AbsTol=1e-12);
            testCase.verifyEqual(invariantObserverSupport.predict(b,.1,0),b,AbsTol=1e-12);
            testCase.verifyGreaterThan(abs(a(7)-b(7)),.1);
        end
        function theoryWitnessDoesNotCertifyOrChangeTheRuntimeGains(testCase)
            before=freshObserverTestConfig();r=testCase.TheoryReport;after=freshObserverTestConfig();
            testCase.verifyEqual(after.invariantDesign,before.invariantDesign);
            testCase.verifyFalse(r.appliesToDefaultRuntimeGains);
            testCase.verifyGreaterThan(norm(r.witness.P-before.invariantDesign.P,'fro'),1);
            runtime=invariantObserverSupport.runtime(after);
            testCase.verifyFalse(runtime.fullContinuousStateCertified);
        end
        function freshlySolvedGainsSatisfyTheNewCommonConditions(testCase)
            c=freshObserverTestConfig();v=invariantObserverSupport.verify(c.invariantDesign,c);
            testCase.verifyTrue(v.jointSensorLmiVerified);
            testCase.verifyEqual(v.jointSensor.vertices,9984);
            testCase.verifyLessThan(v.jointSensor.worstMixedEigenvalue,0);
            testCase.verifyLessThan(v.jointSensor.worstLidarEigenvalue,0);
            testCase.verifyLessThan(v.jointSensor.worstGnssEigenvalue,-.009);
        end
        function inadequateDirectionGainCannotReuseASavedClaim(testCase)
            c=freshObserverTestConfig();c.invariantDesign.auxiliaryWeights(5)=1e-10;
            c.invariantDesign.verification.jointSensorLmiVerified=true;
            v=invariantObserverSupport.verify(c.invariantDesign,c);
            testCase.verifyFalse(v.certified);
            testCase.verifyFalse(v.jointSensorLmiVerified);
            testCase.verifyError(@()invariantObserverSupport.runtime(c), ...
                'VehicleLocalization:InfeasibleInvariantDesign');
        end
        function changedCommonDomainCannotReuseTemporaryGains(testCase)
            c=freshObserverTestConfig();c.invariantObserver.redundancy.minimumGnssSpeed=5;
            testCase.verifyError(@()invariantObserverSupport.runtime(c), ...
                'VehicleLocalization:StaleInvariantDesign');
        end
        function aDifferentHighGainScaleRequiresANewCertificate(testCase)
            c=freshObserverTestConfig();c.invariantObserver.theta=5;
            testCase.verifyError(@()invariantObserverSupport.runtime(c), ...
                'VehicleLocalization:StaleInvariantDesign');
        end
        function absentVelocityWithdrawsTheGnssAdequacyClaim(testCase)
            c=freshObserverTestConfig();c.invariantObserver.velocityAvailable=false;
            v=invariantObserverSupport.verify(c.invariantDesign,c);
            testCase.verifyTrue(v.jointSensorLmiVerified);
            testCase.verifyFalse(v.jointSensor.gnssAdequacyEnabled);
            testCase.verifyEqual(v.jointSensor.lidarVertices,96);
            testCase.verifyEqual(v.jointSensor.gnssBernsteinVertices,0);
        end
        function zeroLeverRetainsFeasibilityWithOneWeightCorner(testCase)
            c=freshObserverTestConfig();c.gnss.outputPoint.bodyOffset=[0;0];h=c.invariantDesign;
            w=sensorRedundancySupport.witness(c,h.P,h.kappaL,h.kappaG,h.auxiliaryWeights, ...
                h.gyroGainNormalized,h.speedGainNormalized);
            v=sensorRedundancySupport.verify(w,true,true);
            testCase.verifyTrue(v.certified);
            testCase.verifyEqual(v.gnssBernsteinVertices,360);
        end
    end
end
