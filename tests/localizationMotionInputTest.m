classdef localizationMotionInputTest < matlab.unittest.TestCase
% localizationMotionInputTest LiDAR translation cannot rewrite motion inputs.
    properties (TestParameter)
        timing=struct('synchronous',"synchronous",'historical',"historical_transport")
    end
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));setupVehicleLocalization;
        end
    end
    methods (Test)
        function positionDisagreementDoesNotLearnVelocity(testCase,timing)
            f=fixture(timing);a=run(f);
            f.data.lidar.pose(:,1:2)=[8*f.time,.2*f.time];b=run(f);
            testCase.verifyEqual(b.velocity,a.velocity,AbsTol=1e-12);
            testCase.verifyEqual(b.velocity,repmat([8.4,-.25],numel(f.time),1),AbsTol=1e-10);
            testCase.verifyGreaterThan(norm(b.position(end,:)-a.position(end,:)),1);
            testCase.verifyEqual(b.lateral,f.lateral);
        end
        function outageUsesSuppliedMotionWithoutLearnedCompensation(testCase,timing)
            f=fixture(timing);f.data.lidar.pose(:,1:2)=[8*f.time,.2*f.time];
            f.data.lidar.valid(f.time>=4)=false;r=run(f);
            testCase.verifyEqual(diff(r.position(end-9:end,:)), ...
                repmat([.84,-.025],9,1),AbsTol=1e-10);
            testCase.verifyEqual(r.diagnostics.mode(end-9:end),zeros(10,1));
        end
        function onlineSeedUsesSuppliedWheelAndLateralVelocity(testCase)
            f=fixture("synchronous");f.data=rmfield(f.data,'lidar');
            f.data.lidarMatcher=@(k,seed,aid) onlineResult(k,seed,aid,f.time);r=run(f);
            testCase.verifyEqual(r.matchingSeeds(2:end,1:2)-r.position(1:end-1,:), ...
                diff(f.time).*[8.4,-.25],AbsTol=1e-10);
            testCase.verifyGreaterThan(norm(r.position(end,:)-[8.4*f.time(end),-.25*f.time(end)]),1);
        end
        function removedLearnerHasNoConfigurationOrDiagnostics(testCase)
            f=fixture("synchronous");r=run(f);
            testCase.verifyFalse(isfield(f.cfg,'bias'));
            testCase.verifyFalse(any(isfield(r.diagnostics, ...
                {'lidarLongitudinalVelocityBias','lidarVelocityBias','lidarBiasUpdates','motionBiasSource'})));
        end
    end
end

function f=fixture(timing)
    t=(0:.1:6).';n=numel(t);z=zeros(n,1);
    h=struct('time',t,'longitudinalSpeed',8.4+z,'longitudinalAcceleration',z, ...
        'lateralAcceleration',z,'yawRate',z);
    l=struct('time',t,'pose',[8.4*t,-.25*t,z],'valid',true(n,1), ...
        'information',repmat(1e6*eye(3),1,1,n),'delay',0);
    cfg=fullObserverConfig();cfg.timing=timing;cfg.initialState=[0;8.4;0;0;-.25;0;0];
    f=struct('time',t,'data',struct('highRate',h,'lidar',l),'cfg',cfg, ...
        'lateral',struct('time',t,'lateralVelocity',-.25+z,'sideSlipAngleRate',z));
end
function r=run(f)
    r=runFullLocalizationObserver(f.data,struct(),f.cfg,LateralInputs=f.lateral);
end
function r=onlineResult(k,~,~,t)
    r=struct('poseXYTheta',[8*t(k),.2*t(k),0],'accepted',true,'information',1e6*eye(3));
end
