classdef precisionStudyTest < matlab.unittest.TestCase
% precisionStudyTest: Physical controls for continuous context measurements.
    methods (Test)
        function isolatedShaftHasNoCompetingVerticalSupport(testCase)
            [p,h]=precisionStudyTest.shaft();
            f=measureVerticalContext(p,true(size(p,1),1),h);
            testCase.verifyGreaterThan(f.height18_60,2.5);
            testCase.verifyEqual(f.mean18_60,1,'AbsTol',1e-12);
            testCase.verifyEqual(f.probeFraction45,1,'AbsTol',1e-12);
        end
        function neighboringShaftReducesContextDominance(testCase)
            [p,h]=precisionStudyTest.shaft();q=[p;p+[.45 0 0]];
            f=measureVerticalContext(q,true(size(q,1),1),h);
            testCase.verifyEqual(f.height18_60,0,'AbsTol',1e-12);
            testCase.verifyLessThan(f.probeFraction45,.55);
            testCase.verifyGreaterThan(f.probeMaximum45,.9);
        end
        function verticalTranslationDoesNotIntroducePhase(testCase)
            [p,h]=precisionStudyTest.shaft();a=measureVerticalContext(p,true(size(p,1),1),h);
            p=p+[4 -7 .173];h.axisXY=h.axisXY+[4 -7];h.axisZ=h.axisZ+.173;
            h.minimumZ=h.minimumZ+.173;h.maximumZ=h.maximumZ+.173;
            b=measureVerticalContext(p,true(size(p,1),1),h);
            testCase.verifyEqual(struct2array(a),struct2array(b),'AbsTol',1e-10);
        end
        function emptyStructuralSupportIsFinite(testCase)
            [p,h]=precisionStudyTest.shaft();f=measureVerticalContext(p,false(size(p,1),1),h);
            testCase.verifyTrue(all(isfinite(struct2array(f))));
            testCase.verifyEqual(f.height25_75,0);
        end
        function wholeHeightContextIncludesStructureAboveShaft(testCase)
            [p,h]=precisionStudyTest.shaft();z=linspace(4,9,201)';
            q=[p;repmat([.45 0],numel(z),1),z];
            local=measureVerticalContext(q,true(size(q,1),1),h);
            whole=measureVerticalContext(q,true(size(q,1),1),h,false);
            testCase.verifyEqual(local.probeFraction45,1,'AbsTol',1e-12);
            testCase.verifyLessThan(whole.probeFraction45,.5);
        end
    end
    methods (Static,Access=private)
        function [p,h]=shaft()
            z=linspace(0,3,121)';angle=(0:120)'*pi/3;
            p=[.05*cos(angle),.05*sin(angle),z];
            h=struct('axisXY',[0 0],'axisZ',1.5,'slopeXY',[0 0],'minimumZ',0,'maximumZ',3);
        end
    end
end
