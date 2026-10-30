function runBessafaMississippiBaseline(folder,tmp)
% Fresh source-family LMI baseline, Eq.41/42/49/55 of Bessafa et al. (2026).
% Same prepared motion/GNSS and initial state; no LiDAR is available to this
% source observer. Compare it with proposed without LiDAR for modality fairness.
% The source's zero tangential-acceleration output is retained. Its constant
% speed/steering assumptions are not asserted for the real trajectories.
    arguments
        folder string="output/mississippi_baselines_20261030"
        tmp string="/tmp/pdvl_mississippi_baselines"
    end
    root=setupVehicleLocalization();maxNumCompThreads(2);ext=fullfile(fileparts(root),'RobustVehicleLocalization','external');addpath(genpath(fullfile(ext,'YALMIP')));addpath(genpath(fullfile(ext,'sedumi')));
    A=kron(eye(2),[0,1,0;0,0,1;0,0,0]);C=eye(6);C=C([1,4],:);degrees=[1,2,3,1,2,3];sigma=160;lambda=1;margin=1e-5;vmin=1;vmax=30;amax=4;rmax=.8;
    % Global Lipschitz extension: project a to |a|<=amax, clip r to rmax,
    % r=(v x a)/max(|v|^2,vmin^2). Then |v|*|D_v r|<=amax/vmin
    % and |v|*|D_a r|<=1. Thus f=-r^2*v has derivative bounds below.
    kf=hypot(rmax^2+2*rmax*amax/vmin,2*rmax);
    entries=[1,2,1;1,5,1;2,2,amax;2,3,vmax;2,5,amax;2,6,vmax;3,2,amax/vmin^2;3,3,1/vmin;3,5,amax/vmin^2;3,6,1/vmin];
    H=cell(1024,1);for k=0:1023,h=zeros(3,6);for j=1:10,h(entries(j,1),entries(j,2))=(2*bitget(k,j)-1)*entries(j,3)*sigma^(degrees(entries(j,2))-4);end;H{k+1}=h;end
    yalmip('clear');P=sdpvar(6,6);X=sdpvar(2,2);Y=sdpvar(2,6,'full');Z=sdpvar(3,6,'full');pmax=sdpvar(1);xmax=sdpvar(1);
    constraints=[P>=eye(6),P<=pmax*eye(6),X>=margin*eye(2),X<=xmax*eye(2),Y(:)>=0];
    for i=1:6,for j=i+1:6,constraints=[constraints,P(i,j)<=0];end;end %#ok<AGROW>
    for j=1:3,constraints=[constraints,Z(j,:)>=1.01*sum(Y,1)+.001];end %#ok<AGROW>
    for k=1:1024,F=P*A-Y.'*C-Z.'*H{k};constraints=[constraints,[F+F.'+lambda*eye(6),Y.';Y,-X]<=-margin*eye(8)];end %#ok<AGROW>
    result=optimize(constraints,pmax+xmax,sdpsettings('solver','sedumi','verbose',0));assert(result.problem==0,'VehicleLocalization:BessafaSynthesis','Source LMI solve failed: %s',result.info);
    P=value(P);X=value(X);Y=value(Y);Z=value(Z);K=P\Y.';N=P\Z.';theta=max(1.01*sigma,1.25*2*kf*max(eig(P))/lambda);scale=theta.^degrees;L=diag(scale)*K;M=diag(scale)*N/theta^3;
    worst=-Inf;for k=1:1024,F=P*A-Y.'*C-Z.'*H{k};worst=max(worst,max(eig([F+F.'+lambda*eye(6),Y.';Y,-X])));end
    assert(min(eig(P))>0 && worst<-margin*.8 && theta*lambda>2*kf*max(eig(P)),'VehicleLocalization:BessafaVerification','Source certificate failed numerical verification.');
    summary=struct('theta',theta,'sigma',sigma,'kf',kf,'worstLmiEigenvalue',worst,'vertices',1024,'vehicle',mncavVehicleConfig(),'gainPolicy',"Fresh source-family LMI; no persisted gains",'inputReconstruction',"Linear adjacent sensor/GNSS endpoints when both fixes valid; no packet after the current endpoint. No stale GNSS interpolation through outages",'scope',"Conservative source-family member with Lipschitz extension, not authors code or best attainable tuning; actual maneuvers/low speed violate source assumptions");rows={};
    for drive=1:2
        b=load(fullfile(tmp,"drive"+drive+"_paired.mat"),'data','lateral','reference');h=b.data.highRate;l=b.lateral;ref=b.reference;t=h.time-h.time(1);n=numel(t);v=[cos(ref(1,3)),-sin(ref(1,3));sin(ref(1,3)),cos(ref(1,3))]*[h.longitudinalSpeed(1);l.lateralVelocity(1)];origin=ref(1,1:2).';initial=[0;v(1);0;0;v(2);0];cfg=mncavSmallOffsetObserverConfig();lever=cfg.gnss.outputPoint.bodyOffset(:);yaw=ref(1,3)+cumtrapz(t,h.yawRate);gnss=b.data.gnss.position-[cos(yaw)*lever(1)-sin(yaw)*lever(2),sin(yaw)*lever(1)+cos(yaw)*lever(2)];
        for scenario=["both","gnss_loss","no_gnss"]
            valid=b.data.gnss.valid;if scenario=="gnss_loss",valid(t>=.35*t(end) & t<.65*t(end))=false;end;if scenario=="no_gnss",valid(:)=false;end
            timer=tic;
            [pose,failure]=simulate(initial,t,h,l,ref,gnss,origin,valid,1e-7,.02);seconds=toc(timer);
            if scenario=="both"
                [fine,fineFailure]=simulate(initial,t,h,l,ref,gnss,origin,valid,1e-9,.01);
                discrepancy=vecnorm(fine(:,1:2)-pose(:,1:2),2,2);yawDifference=atan2(sin(fine(:,3)-pose(:,3)),cos(fine(:,3)-pose(:,3)));
                summary.integrationCheck(drive)=struct('drive',drive,'maximumPositionDifferenceM',max(discrepancy),'maximumHeadingDifferenceDeg',rad2deg(max(abs(yawDifference))),'fineFailure',fineFailure);
                assert(all(isfinite(discrepancy)) && max(discrepancy)<1e-3 && max(abs(yawDifference))<deg2rad(.1),'VehicleLocalization:BessafaIntegration','Source baseline is not numerically converged.');
            end
            err=vecnorm(pose(:,1:2)-ref(:,1:2),2,2);angle=atan2(sin(pose(:,3)-ref(:,3)),cos(pose(:,3)-ref(:,3)));scores=[Inf,Inf,Inf,Inf];complete=all(isfinite(pose),'all');if complete,scores=[rms(err),prctile(err,95),max(err),rad2deg(rms(angle))];end
            rows(end+1,:)={drive,scenario,n,scores(1),scores(2),scores(3),scores(4),complete,seconds,failure}; %#ok<AGROW>
            fprintf('BESSAFA drive%d %s RMSE %.6g max %.6g complete %d\n',drive,scenario,scores(1),scores(3),complete);
        end
    end
    writetable(cell2table(rows,VariableNames={'drive','scenario','frames','rmseM','p95M','maximumM','headingRmseDeg','complete','runtimeS','failure'}),fullfile(folder,'bessafa_metrics.csv'));
    f=fopen(fullfile(folder,'bessafa_summary.json'),'w');fprintf(f,'%s',jsonencode(summary,PrettyPrint=true));fclose(f);
    function [pose,failure]=simulate(state,t,h,l,ref,gnss,origin,valid,tolerance,maxStep)
        n=numel(t);pose=nan(n,3);pose(1,:)=ref(1,:);held=ref(1,3);failure="";
        try
            for frameIndex=2:n
                vy=0;beta=0;if l.velocityValid(frameIndex),vy=l.lateralVelocity(frameIndex);beta=l.sideSlipAngle(frameIndex);end
                measurement=[hypot(h.longitudinalSpeed(frameIndex),vy);0;h.yawRate(frameIndex)];position=gnss(frameIndex,:).'-origin;
                previousVy=0;if l.velocityValid(frameIndex-1),previousVy=l.lateralVelocity(frameIndex-1);end
                before=[hypot(h.longitudinalSpeed(frameIndex-1),previousVy);0;h.yawRate(frameIndex-1)];positionBefore=position;
                if valid(frameIndex) && valid(frameIndex-1),positionBefore=gnss(frameIndex-1,:).'-origin;end
                dt=t(frameIndex)-t(frameIndex-1);
                [~,states]=ode15s(@(tau,x)rhs(x,before+(measurement-before)*(tau/dt),positionBefore+(position-positionBefore)*(tau/dt),valid(frameIndex)),[0,dt],state,odeset('RelTol',tolerance,'AbsTol',tolerance/10,'MaxStep',maxStep));state=states(end,:).';
                if any(~isfinite(state)) || norm(state([2,5]))>100,failure="nonfinite state or speed >100 m/s";break;end
                if norm(state([2,5]))>=vmin,angle=atan2(state(5),state(2))-beta;held=held+atan2(sin(angle-held),cos(angle-held));end
                pose(frameIndex,:)=[state([1,4]).'+origin.',held];
            end
        catch error
            failure=string(error.identifier)+": "+string(error.message);
        end
    end
    function dx=rhs(x,m,p,available)
        v=x([2,5]);a=x([3,6]);a=a*min(1,amax/max(norm(a),eps));r=max(-rmax,min(rmax,(v(1)*a(2)-v(2)*a(1))/max(v.'*v,vmin^2)));hvalue=[norm(v);min(1,vmax/max(norm(v),eps))*dot(v,a);r];
        dx=A*x+M*(m-hvalue);dx([3,6])=dx([3,6])-r^2*v;if available,dx=dx+L*(p-C*x);end
    end
end
