classdef sensorRedundancyTheoryTest < matlab.unittest.TestCase
% Common continuous-theory inequalities and the position-only observability limit.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));setupVehicleLocalization;
        end
    end
    methods (Test)
        function oneWitnessCoversBothMissingSensorCases(testCase)
            r=verifySensorRedundancyTheory();
            testCase.verifyTrue(r.commonLmiVerified);
            testCase.verifyTrue(r.coversGnssAbsent);
            testCase.verifyTrue(r.coversZeroLidarInformation);
            testCase.verifyEqual(r.lidarVertices,128);
            testCase.verifyEqual(r.gnssBernsteinVertices,480);
            testCase.verifyLessThan(max(r.worstLidarEigenvalue,r.worstGnssEigenvalue),-.009);
        end
        function zeroMotionCannotBeAdmittedAsFullStateGnssAdequacy(testCase)
            r=verifySensorRedundancyTheory();w=r.witness;w.domain.minimumGnssSpeed=0;
            testCase.verifyError(@()verifySensorRedundancyTheory(Witness=w), ...
                'VehicleLocalization:UnobservableGnssHeading');
        end
        function storedWitnessNeedsNoSdpSolver(testCase)
            originalPath=path;cleanup=onCleanup(@()path(originalPath));
            root=fileparts(fileparts(mfilename('fullpath')));restoredefaultpath;addpath(root);setupVehicleLocalization;
            testCase.verifyEqual(exist('sdpvar','file'),0);
            r=verifySensorRedundancyTheory();testCase.verifyTrue(r.commonLmiVerified);
        end
        function changedManeuverInputsRejectTheWitness(testCase)
            r=verifySensorRedundancyTheory();w=r.witness;w.inputs.maximumSpeed=17;
            testCase.verifyError(@()verifySensorRedundancyTheory(Witness=w), ...
                'VehicleLocalization:StaleRedundancyWitness');
        end
        function losingTheDirectionChannelInvalidatesTheGnssProof(testCase)
            r=verifySensorRedundancyTheory();w=r.witness;w.directionGain=1e-10;
            testCase.verifyError(@()verifySensorRedundancyTheory(Witness=w), ...
                'VehicleLocalization:UnverifiedRedundancyTheory');
        end
        function parkedHeadingsHaveIdenticalAvailableOutputs(testCase)
            c=mncavFullObserverConfig();d=c.gnss.outputPoint.bodyOffset(:);
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
            before=mncavFullObserverConfig();r=verifySensorRedundancyTheory();after=mncavFullObserverConfig();
            testCase.verifyEqual(after.invariantDesign,before.invariantDesign);
            testCase.verifyFalse(r.appliesToDefaultRuntimeGains);
            testCase.verifyGreaterThan(norm(r.witness.P-before.invariantDesign.P,'fro'),1);
            runtime=invariantObserverSupport.runtime(after);
            testCase.verifyFalse(runtime.fullContinuousStateCertified);
        end
    end
end
