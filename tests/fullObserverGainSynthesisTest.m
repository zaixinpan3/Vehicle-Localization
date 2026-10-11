classdef fullObserverGainSynthesisTest < matlab.unittest.TestCase
% fullObserverGainSynthesisTest The MnCAV gains are solved from the ISS LMI.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));
            setupVehicleLocalization;
        end
    end
    methods (Test)
        function storedGainsReproduceTheSynthesis(testCase)
            cfg=mncavFullObserverConfig();fresh=mncavFullObserverConfig(LoadSynthesizedGains=false);
            design=fullObserverSupport.synthesizeFullObserverGains(fresh);
            testCase.verifyEqual(cfg.gains,design.gains,RelTol=1e-4);
            testCase.verifyEqual(cfg.gnss.positionGain,design.gnssPositionGain,RelTol=1e-4);
            testCase.verifyEqual(string(cfg.gainSource),"config/mncavSynthesizedGains.json");
        end
        function synthesizedGainsCarryTheirOwnCertificate(testCase)
            cfg=mncavFullObserverConfig();d=fullObserverSupport.designFullObserverGains(cfg);
            v=d.continuousCertificate.verification;
            testCase.verifyTrue(v.certified);
            testCase.verifyGreaterThanOrEqual(v.verifiedDecayRate,cfg.synthesis.decayRate);
        end
        function fasterDecayNeedsLargerGains(testCase)
            cfg=mncavFullObserverConfig(LoadSynthesizedGains=false);
            slow=fullObserverSupport.synthesizeFullObserverGains(cfg);
            cfg.synthesis.decayRate=2*cfg.synthesis.decayRate;
            fast=fullObserverSupport.synthesizeFullObserverGains(cfg);
            testCase.verifyTrue(all(fast.gains>slow.gains));
            testCase.verifyLessThan(fast.radiusBound,slow.radiusBound);
        end
        function gnssGainComesFromTheWeakInformationScenario(testCase)
            cfg=mncavFullObserverConfig(LoadSynthesizedGains=false);
            with=fullObserverSupport.synthesizeFullObserverGains(cfg);
            cfg.synthesis.minimumGnssWeight=0;cfg.synthesis.weakPositionStrength=cfg.iss.minimumPositionStrength;
            without=fullObserverSupport.synthesizeFullObserverGains(cfg);
            testCase.verifyGreaterThan(with.gnssPositionGain,.1);
            testCase.verifyLessThan(without.gnssPositionGain,1e-3);
        end
        function noisierVelocityMeasurementLowersItsGain(testCase)
            cfg=mncavFullObserverConfig(LoadSynthesizedGains=false);
            base=fullObserverSupport.synthesizeFullObserverGains(cfg);
            cfg.synthesis.noise.velocityMps=4*cfg.synthesis.noise.velocityMps;
            noisy=fullObserverSupport.synthesizeFullObserverGains(cfg);
            testCase.verifyLessThan(noisy.gains(2),base.gains(2));
        end
        function staleStoredGainsAreRejected(testCase)
            cfg=mncavFullObserverConfig(LoadSynthesizedGains=false);
            testCase.verifyEmpty(cfg.gains);
            testCase.verifyError(@()mncavFullObserverConfig(SynthesisOverride=struct('decayRate',cfg.synthesis.decayRate*3)), ...
                'VehicleLocalization:StaleSynthesizedGains');
        end
    end
end
