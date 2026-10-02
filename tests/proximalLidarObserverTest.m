classdef proximalLidarObserverTest < matlab.unittest.TestCase
% proximalLidarObserverTest Convex update, uncertainty and bias feedback contracts.
    methods (TestClassSetup)
        function paths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(root));setupVehicleLocalization;
        end
    end
    methods (Test)
        function quadraticMatchesInformationSolution(testCase)
            [m,G,C,cfg]=kernel();cfg.huberThreshold=Inf;
            [d,P,a]=computeProximalLidarCorrection(m,zeros(3,1),G,C,cfg);
            expected=(C\eye(9)+G.'*a.precision*G)\(G.'*a.precision*m.pose);
            testCase.verifyEqual(d,expected,AbsTol=1e-12);
            testCase.verifyEqual(P,(C\eye(9)+G.'*a.precision*G)\eye(9),AbsTol=1e-12);
        end
        function rescalingGeometryChangesNothing(testCase)
            [m,G,C,cfg]=kernel();[d,P,a]=computeProximalLidarCorrection(m,zeros(3,1),G,C,cfg);
            m.information=m.information*1e6;[d2,P2,a2]=computeProximalLidarCorrection(m,zeros(3,1),G,C,cfg);
            testCase.verifyEqual(d,d2,AbsTol=1e-12);testCase.verifyEqual(P,P2,AbsTol=1e-12);
            testCase.verifyEqual(a.precision,a2.precision,AbsTol=1e-9);
        end
        function residualAndPoseRepresentationsAgree(testCase)
            [m,G,C,cfg]=kernel();d=computeProximalLidarCorrection(m,zeros(3,1),G,C,cfg);
            J=chol(m.information);r=struct('residual',-J*m.pose,'jacobian',J,'weights',ones(3,1),'linearizationPose',zeros(3,1));
            dr=computeProximalLidarCorrection(r,zeros(3,1),G,C,cfg);
            testCase.verifyEqual(d,dr,AbsTol=1e-12);
            r.jacobian=7*r.jacobian;r.residual=7*r.residual;
            ds=computeProximalLidarCorrection(r,zeros(3,1),G,C,cfg);testCase.verifyEqual(d,ds,AbsTol=1e-12);
        end
        function largeOutlierHasBoundedInfluence(testCase)
            [m,G,C,cfg]=kernel();m.information=diag([10,0,0]);m.pose=[100;0;0];
            [d,P,a]=computeProximalLidarCorrection(m,zeros(3,1),G,C,cfg);
            testCase.verifyLessThan(a.robustWeight,1);testCase.verifyLessThan(a.stationarityResidual,1e-10);
            testCase.verifyEqual(d(1),C(1,1)*cfg.huberThreshold/cfg.poseStd(1),AbsTol=1e-10);
            testCase.verifyEqual(P,C,AbsTol=1e-12); % Zero tail curvature along the clipped scalar residual.
        end
        function noiseFreeProximalMapIsFirmlyNonexpansive(testCase)
            [m,G,C,cfg]=kernel();m.pose=zeros(3,1);x=[.8;0;0;-.7;0;0;.02;0;0];m.pose=-G*x;
            [d,~,a]=computeProximalLidarCorrection(m,zeros(3,1),G,C,cfg);
            testCase.verifyLessThanOrEqual((x+d).'*(C\(x+d))+d.'*(C\d),x.'*(C\x)+1e-10);
            testCase.verifyLessThan(a.stationarityResidual,1e-10);
        end
        function updatedCurvatureMetricAlsoDissipates(testCase)
            [m,G,~,cfg]=kernel();stream=RandStream('mt19937ar','Seed',20261002);
            for k=1:30
                A=randn(stream,9);D=diag(cfg.initialStd);C=D*(A*A.'+eye(9))*D;
                x=randn(stream,9,1);x(7)=.01*x(7);m.pose=-G*x;
                [d,P]=computeProximalLidarCorrection(m,zeros(3,1),G,C,cfg);
                testCase.verifyGreaterThan(min(eig(P)),0);
                testCase.verifyLessThanOrEqual((x+d).'*(P\(x+d)),x.'*(C\x)+1e-8);
            end
        end
        function zeroGeometryWithdrawsExactly(testCase)
            [m,G,C,cfg]=kernel();m.information=zeros(3);
            [d,P,a]=computeProximalLidarCorrection(m,zeros(3,1),G,C,cfg);
            testCase.verifyEqual(d,zeros(9,1),AbsTol=0);testCase.verifyEqual(P,C,AbsTol=0);testCase.verifyEqual(a.rank,0);
        end
        function rejectedReliabilityWithdrawsExactly(testCase)
            [m,G,C,cfg]=kernel();m.frameReliability=0;
            [d,P]=computeProximalLidarCorrection(m,zeros(3,1),G,C,cfg);
            testCase.verifyEqual(d,zeros(9,1),AbsTol=0);testCase.verifyEqual(P,C,AbsTol=0);
        end
        function directionalReliabilityRetainsUpstreamCoordinates(testCase)
            [m,G,C,cfg]=kernel();m.information=diag([10,12,1600]);m.directionReliability=[0;1;1];
            d=computeProximalLidarCorrection(m,zeros(3,1),G,C,cfg);
            testCase.verifyEqual(d(1),0,AbsTol=1e-14);testCase.verifyNotEqual(d(4),0);
        end
        function geometricNullspaceDoesNotBecomeAMeasurement(testCase)
            [m,G,C,cfg]=kernel();m.information=diag([0,12,0]);m.pose=[2;.1;1];
            [d,~,a]=computeProximalLidarCorrection(m,zeros(3,1),G,C,cfg);
            testCase.verifyEqual(d([1,7]),zeros(2,1),AbsTol=0);testCase.verifyEqual(a.rank,1);
        end
        function largerPredictionUncertaintyIncreasesCorrection(testCase)
            [m,G,C,cfg]=kernel();cfg.huberThreshold=Inf;
            d=computeProximalLidarCorrection(m,zeros(3,1),G,C,cfg);
            large=computeProximalLidarCorrection(m,zeros(3,1),G,100*C,cfg);
            testCase.verifyGreaterThan(norm(large([1,4])),norm(d([1,4])));
        end
        function biasedMotionIsLearnedAndHeldThroughOutage(testCase)
            [data,lateral,cfg]=straight();data.highRate.longitudinalSpeed(:)=8.3;
            data.lidar.valid(data.highRate.time>=15)=false;
            e=runSynchronousLocalizationObserver(data,cfg,lateral);
            testCase.verifyEqual(e.motionBias(end,1),-.3,AbsTol=.002);
            testCase.verifyEqual(e.motionBias(151:end,:),repmat(e.motionBias(150,:),numel(e.time)-150,1),AbsTol=0);
            testCase.verifyLessThan(norm(e.position(end,:)-[8*e.time(end),0]),.02);
        end
        function invalidPayloadCannotAffectThePast(testCase)
            [data,lateral,cfg]=straight();a=runSynchronousLocalizationObserver(data,cfg,lateral);
            data.lidar.valid(151:end)=false;data.lidar.pose(151:end,:)=NaN;b=runSynchronousLocalizationObserver(data,cfg,lateral);
            testCase.verifyEqual(a.z(1:150,:),b.z(1:150,:),AbsTol=0);
            testCase.verifyEqual(b.diagnostics.positionCorrection(151:end,3:4),zeros(numel(b.time)-150,2),AbsTol=0);
            testCase.verifyEqual(b.diagnostics.virtualPoseUpdates,0);
        end
        function absentLidarDoesNotInventMotionBias(testCase)
            [data,lateral,cfg]=straight();data=rmfield(data,'lidar');e=runSynchronousLocalizationObserver(data,cfg,lateral);
            testCase.verifyEqual(e.motionBias,zeros(numel(e.time),2),AbsTol=0);
            testCase.verifyEqual(e.position(:,1),8*e.time,AbsTol=1e-10);
        end
        function uncertaintyFlowMatchesFiniteDifference(testCase)
            cfg=proximalLidarConfig();g=[4,4,12,4];dt=.1;r=.15;J=[0,-1;1,0];
            A=[(1+dt*g(2))*eye(2),-dt*eye(2);-dt*r^2*eye(2),(1+dt*g(3))*eye(2)-2*dt*r*J];
            x=[1;8;0;2;.2;1;.4;.1;-.1];u=[8;.2];acc=[0;1];yaw=x(7)+r*dt;R=[cos(yaw),-sin(yaw);sin(yaw),cos(yaw)];
            [~,F]=propagateProximalUncertainty(eye(9),dt,A,R,u+x(8:9),acc,zeros(2),nan(2),g,cfg);
            numeric=zeros(9);step=1e-6;
            for j=1:9,e=zeros(9,1);e(j)=step;numeric(:,j)=(meanStep(x+e,dt,r,A,u,acc,g)-meanStep(x-e,dt,r,A,u,acc,g))/(2*step);end
            testCase.verifyEqual(F,numeric,AbsTol=2e-9);
        end
        function gnssAlignmentDerivativeMatchesActualFlow(testCase)
            [data,lateral,cfg]=straight();data=rmfield(data,'lidar');
            names=fieldnames(data.highRate);for j=1:numel(names),data.highRate.(names{j})=data.highRate.(names{j})(1:2);end
            names=fieldnames(lateral);for j=1:numel(names),lateral.(names{j})=lateral.(names{j})(1:2);end
            cfg.initialState=[1;8;.2;2;.1;.8;.3];data.highRate.yawRate(:)=.12;
            data.highRate.lateralAcceleration(:)=.8;
            data.gnss=struct('time',data.highRate.time,'position',[3,3;3.8,3.1],'information',repmat([100,2;2,60],1,1,2),'valid',true(2,1),'delay',0);
            original=runSynchronousLocalizationObserver(data,cfg,lateral);F=eye(9);step=1e-6;
            for j=1:9
                plus=cfg;minus=cfg;dp=data;dm=data;lp=lateral;lm=lateral;
                if j<=7
                    plus.initialState(j)=plus.initialState(j)+step;minus.initialState(j)=minus.initialState(j)-step;
                elseif j==8
                    dp.highRate.longitudinalSpeed=dp.highRate.longitudinalSpeed+step;dm.highRate.longitudinalSpeed=dm.highRate.longitudinalSpeed-step;
                else
                    lp.lateralVelocity=lp.lateralVelocity+step;lm.lateralVelocity=lm.lateralVelocity-step;
                end
                a=runSynchronousLocalizationObserver(dp,plus,lp);b=runSynchronousLocalizationObserver(dm,minus,lm);
                F(1:7,j)=(a.z(2,:)-b.z(2,:)).'/(2*step);
            end
            dt=.1;I=original.diagnostics.gnssInformationAtObserverPoint(:,:,2);
            W=I/(I+cfg.gnss.gainInformationScale*eye(2));W=(W+W.')/2;K=cfg.gnss.positionGain*W;
            B=(eye(2)+dt*K)\(dt*K);Q=dt*diag(cfg.lidar.proximal.processStd.^2);Q([1,4],[1,4])=Q([1,4],[1,4])+B*(I\eye(2))*B.';
            expected=F*diag(cfg.lidar.proximal.initialStd.^2)*F.'+Q;
            testCase.verifyEqual(original.diagnostics.uncertaintyDiagonal(2,:).',diag(expected),AbsTol=1e-8);
        end
    end
end
function [m,G,C,cfg]=kernel()
    cfg=proximalLidarConfig();G=zeros(3,9);G(1,1)=1;G(2,4)=1;G(3,7)=1;C=diag(cfg.initialStd.^2);
    m=struct('pose',[.2;-.1;.01],'information',[10,1,.3;1,12,-.2;.3,-.2,1600]);
end
function [data,lateral,cfg]=straight()
    cfg=proximalFullObserverConfig();cfg.initialState=[0;8;0;0;0;0;0];t=(0:.1:20).';z=zeros(size(t));n=numel(t);
    data=struct('highRate',struct('time',t,'longitudinalSpeed',8+z,'longitudinalAcceleration',z,'lateralAcceleration',z,'yawRate',z), ...
        'lidar',struct('time',t,'pose',[8*t,z,z],'valid',true(n,1),'delay',0,'information',repmat(diag([10,12,1600]),1,1,n)));
    lateral=struct('time',t,'lateralVelocity',z,'sideSlipAngleRate',z);
end
function y=meanStep(x,dt,r,A,u,acc,g)
    yaw=x(7)+r*dt;R=[cos(yaw),-sin(yaw);sin(yaw),cos(yaw)];
    va=A\[x([2,5])+dt*g(2)*R*(u+x(8:9));x([3,6])+dt*g(3)*R*acc];
    p=x([1,4])+dt/2*(x([2,5])+va(1:2));y=[p(1);va(1);va(3);p(2);va(2);va(4);yaw;x(8:9)];
end
