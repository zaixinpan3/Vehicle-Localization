function diagnoseFrame851()
% diagnoseFrame851 Explain the 68 cm longitudinal matching error at frame 851.
% Uses the 2026-09-24 production chain outputs. Reference poses seed only the
% evaluation ceiling and the basin scans; they never enter the production path.
    setupVehicleLocalization;dest=fileparts(mfilename('fullpath'));outDir='output/frame851_diagnosis_20260924';
    if ~isfolder(outDir),mkdir(outDir);end
    %% ---- frame851.m
    C=load('output/mncav_coarse_localization_20260924/sources.mat','fixed','sources','calls');c=C.calls;cfg=distributionRegistrationConfig();
    ref=[c.referenceX,c.referenceY,c.referencePsi];seed=[c.predictedX,c.predictedY,c.predictedPsi];out=[c.x,c.y,c.psi];
    body=@(p,k)[(p(1)-ref(k,1))*cos(ref(k,3))+(p(2)-ref(k,2))*sin(ref(k,3)),-(p(1)-ref(k,1))*sin(ref(k,3))+(p(2)-ref(k,2))*cos(ref(k,3))];
    fprintf('%5s %6s %6s %6s | %7s %7s %7s | %5s %5s %5s %5s\n','frame','seedE','matchE','acc','lon','lat','dpsi','curb','pole','sign','sim');
    for k=830:2:858
        s=C.sources{k}.components;names=string(s.semanticName);b=body(out(k,:),k);
        fprintf('%5d %6.1f %6.1f %6d | %7.1f %7.1f %7.2f | %5d %5d %5d %5.2f\n',k,100*norm(seed(k,1:2)-ref(k,1:2)),100*norm(out(k,1:2)-ref(k,1:2)),c.accepted(k), ...
            100*b(1),100*b(2),rad2deg(atan2(sin(out(k,3)-ref(k,3)),cos(out(k,3)-ref(k,3)))),nnz(names=="curb"),nnz(names=="pole"),nnz(names=="trafficSign"),c.similarity(k));
    end
    k=851;r0=matchLocalProbabilityCloud(C.fixed,C.sources{k},seed(k,:),cfg,[]);rr=matchLocalProbabilityCloud(C.fixed,C.sources{k},ref(k,:),cfg,[]);
    fprintf('\nframe 851 recursive: err %.1f cm, sim %.3f, iters %d, conv %d | reference-seeded: err %.1f cm, sim %.3f\n',100*norm(r0.poseXYTheta(1:2)-ref(k,1:2)),r0.similarity,r0.iterations,r0.converged,100*norm(rr.poseXYTheta(1:2)-ref(k,1:2)),rr.similarity);
    fprintf('body-frame error of recursive solution: lon %.1f lat %.1f cm; of reference-seeded: lon %.1f lat %.1f cm\n',100*body(r0.poseXYTheta,k),100*body(rr.poseXYTheta,k));
    disp('information (recursive) eig:');disp(eig(r0.information).');disp('information (ref-seeded) eig:');disp(eig(rr.information).');
    % per-class correspondence comparison (recursive vs reference-seeded)
    for nm=["curb","pole","trafficSign"]
        a=r0.correspondences(r0.correspondences.semanticName==nm,:);b=rr.correspondences(rr.correspondences.semanticName==nm,:);
        [~,ia,ib]=intersect(a.source,b.source);ch=nnz(a.globalTarget(ia)~=b.globalTarget(ib));
        fprintf('%-12s recursive: %d pairs, sumW %.4f, mean chi2 %.2f | ref: %d pairs, sumW %.4f, mean chi2 %.2f | changed targets %d/%d\n',nm,height(a),sum(a.robustWeight),mean(a.squaredStandardizedResidual),height(b),sum(b.robustWeight),mean(b.squaredStandardizedResidual),ch,numel(ia));
    end
    disp(r0.classDiagnostics);
    % basin scan: seeds on a body-frame grid around the reference
    [LON,LAT]=meshgrid(-1.5:0.25:1.5,-1.5:0.25:1.5);R=[cos(ref(k,3)) -sin(ref(k,3));sin(ref(k,3)) cos(ref(k,3))];E=nan(size(LON));
    for i=1:numel(LON)
        p=ref(k,:)+[(R*[LON(i);LAT(i)]).',0];r=matchLocalProbabilityCloud(C.fixed,C.sources{k},p,cfg,[]);
        if r.accepted,b=body(r.poseXYTheta,k);E(i)=100*b(2);end
    end
    fprintf('\nconverged LATERAL error (cm) vs seed offset; rows = seed lateral %.2f..%.2f, cols = seed longitudinal %.2f..%.2f\n',LAT(1,1),LAT(end,1),LON(1,1),LON(1,end));disp(round(E));
    seedB=body(seed(k,:),k);fprintf('seed body offset at 851: lon %.1f lat %.1f cm\n',100*seedB);
    % where did the seed go wrong: body-frame errors of accepted outputs 836-851
    fprintf('lateral error of accepted outputs 836..851: ');for j=836:851,b=body(out(j,:),j);fprintf('%d:%.0f ',j,100*b(2));end;fprintf('\n');
    save(fullfile(outDir,'lateral_basin.mat'),'E','LON','LAT','r0','rr');
    %% ---- odo851.m
    M=load('output/mncav_coarse_localization_20260924/matching/report.mat','report');c=M.report.calls;m=M.report.deadReckoning{:,{'x','y','psi'}};
    E=load('output/mncav_coarse_localization_20260924/observer/experiment.mat','data','lateral','reference');h=E.data.highRate;t=h.time;ref=[c.referenceX,c.referenceY,c.referencePsi];
    fprintf('%5s %6s %7s %7s %7s | %6s %6s %6s | %6s %6s\n','frame','dt','odoLon','refLon','dLon','wheel','refV','ax','r','vyOut');
    cum=0;
    for k=826:856
        dt=c.timeSeconds(k)-c.timeSeconds(k-1);R=[cos(m(k-1,3)) sin(m(k-1,3))];do=(m(k,1:2)-m(k-1,1:2))*R.';
        Rr=[cos(ref(k-1,3)) sin(ref(k-1,3))];dr=(ref(k,1:2)-ref(k-1,1:2))*Rr.';cum=cum+(do-dr);
        refV=norm(ref(k,1:2)-ref(k-1,1:2))/dt;
        fprintf('%5d %6.3f %7.3f %7.3f %7.1f | %6.2f %6.2f %6.2f %6.3f %6.2f\n',k,dt,do,dr,100*(do-dr),h.longitudinalSpeed(k),refV,h.longitudinalAcceleration(k),h.yawRate(k),E.lateral.lateralVelocity(k));
    end
    fprintf('cumulative odometry-minus-reference longitudinal step 826..856: %.1f cm\n',100*cum);
    % whole-drive: per-frame longitudinal odometry step error statistics, and where it is largest
    n=height(c);dl=zeros(n,1);
    for k=2:n
        R=[cos(m(k-1,3)) sin(m(k-1,3))];do=(m(k,1:2)-m(k-1,1:2))*R.';Rr=[cos(ref(k-1,3)) sin(ref(k-1,3))];dr=(ref(k,1:2)-ref(k-1,1:2))*Rr.';dl(k)=do-dr;
    end
    fprintf('whole drive per-frame longitudinal step error: mean %.2f cm, RMS %.2f cm; mean over 830-850: %.2f cm\n',100*mean(dl),100*rms(dl),100*mean(dl(830:850)));
    [~,idx]=sort(abs(movmean(dl,15)),'descend');fprintf('largest 15-frame mean |step error| around frames: %s\n',mat2str(idx(1:8).'));
    %% ---- basin851.m
    C=load('output/mncav_coarse_localization_20260924/sources.mat','fixed','sources','calls');c=C.calls;cfg=distributionRegistrationConfig();
    ref=[c.referenceX,c.referenceY,c.referencePsi];seed=[c.predictedX,c.predictedY,c.predictedPsi];
    body=@(p,k)[(p(1)-ref(k,1))*cos(ref(k,3))+(p(2)-ref(k,2))*sin(ref(k,3)),-(p(1)-ref(k,1))*sin(ref(k,3))+(p(2)-ref(k,2))*cos(ref(k,3))];
    % body-frame information (longitudinal, lateral) for frames 830..856 from the recursive solves
    fprintf('%5s %9s %9s %9s | %s\n','frame','I_lon','I_lat','I_yaw','longitudinal basin: converged lon error (cm) for seed lon offsets -150:25:150 cm');
    for k=[830:4:846 848:1:855]
        r=matchLocalProbabilityCloud(C.fixed,C.sources{k},seed(k,:),cfg,[]);Rb=[cos(ref(k,3)) -sin(ref(k,3));sin(ref(k,3)) cos(ref(k,3))];
        I=r.information(1:2,1:2);Ib=Rb.'*I*Rb;
        R=Rb;e=nan(1,13);offs=-1.5:0.25:1.5;
        for i=1:13
            p=ref(k,:)+[(R*[offs(i);0]).',0];q=matchLocalProbabilityCloud(C.fixed,C.sources{k},p,cfg,[]);
            if q.accepted,b=body(q.poseXYTheta,k);e(i)=100*b(1);end
        end
        fprintf('%5d %9.1f %9.1f %9.1f | %s\n',k,Ib(1,1),Ib(2,2),r.information(3,3),mat2str(round(e)));
    end
    k=851;r0=matchLocalProbabilityCloud(C.fixed,C.sources{k},seed(k,:),cfg,[]);rr=matchLocalProbabilityCloud(C.fixed,C.sources{k},ref(k,:),cfg,[]);
    mp=C.fixed.components.mean;sm=C.sources{k}.components.mean;
    % map poles/signs within 40 m: body-frame positions (to see spacing/aliasing)
    names=string(C.fixed.components.semanticName);
    for nm=["pole","trafficSign"]
        idx=find(names==nm);P=zeros(numel(idx),2);for j=1:numel(idx),P(j,:)=body(mp(idx(j),:),k);end
        keep=abs(P(:,1))<40&abs(P(:,2))<25;[~,o]=sort(P(keep,1));Q=P(keep,:);Q=Q(o,:);ids=idx(keep);ids=ids(o);
        fprintf('\nmap %s within 40 m (body lon, lat): ',nm);fprintf('%d[%.1f,%.1f] ',[ids.';Q.']);fprintf('\n');
    end
    % figure
    fig=figure('Visible','off','Color','w','Position',[100 100 1100 800]);theme(fig,'light');hold on;
    cls=["curb","pole","trafficSign"];col=[.6 .6 .6;.85 .2 .2;.1 .4 .9];
    for j=1:3,idx=names==cls(j);P=(mp(idx,:)-ref(k,1:2))*[cos(ref(k,3)) -sin(ref(k,3));sin(ref(k,3)) cos(ref(k,3))];scatter(P(:,1),P(:,2),18,col(j,:),'filled',DisplayName="map "+cls(j));end
    sn=string(C.sources{k}.components.semanticName);
    for j=1:3
        idx=sn==cls(j);
        for pose={r0.poseXYTheta,rr.poseXYTheta}
            p=pose{1};R=[cos(p(3)) -sin(p(3));sin(p(3)) cos(p(3))];W=sm(idx,:)*R.'+p(1:2);B=(W-ref(k,1:2))*[cos(ref(k,3)) -sin(ref(k,3));sin(ref(k,3)) cos(ref(k,3))];
            if isequal(p,r0.poseXYTheta),scatter(B(:,1),B(:,2),40,col(j,:),'x',LineWidth=1.2,DisplayName="source at recursive pose ("+cls(j)+")");
            else,scatter(B(:,1),B(:,2),40,col(j,:),'o',LineWidth=1.0,DisplayName="source at reference-seeded pose ("+cls(j)+")");end
        end
    end
    plot(0,0,'kp',MarkerSize=12,MarkerFaceColor='k',DisplayName='reference pose');b0=body(r0.poseXYTheta,k);plot(b0(1),b0(2),'k^',MarkerSize=10,MarkerFaceColor='y',DisplayName='recursive pose');
    axis equal;xlim([-40 40]);ylim([-25 25]);grid on;xlabel('longitudinal (m, body frame at reference)');ylabel('lateral (m)');
    title('Frame 851: map components and source clouds at the two solutions');legend(Location='eastoutside',FontSize=8);
    exportgraphics(fig,fullfile(dest,'frame851_scene.png'),Resolution=130);close(fig);
%% ---- map component duplication near the vehicle
c=C.fixed.components;
for g={[1264 1278 1263 1277],[1279 1286 1280 1287],[1033 1051 1052 1053]}
    ids=g{1};fprintf('\n');
    for i=ids
        fprintf('%4d %-12s dmean [%6.3f %6.3f] sd [%.3f %.3f] w %.5f id %s\n',i,string(c.semanticName(i)), ...
            c.mean(i,1)-c.mean(ids(1),1),c.mean(i,2)-c.mean(ids(1),2),sqrt(c.covariance(1,1,i)),sqrt(c.covariance(2,2,i)), ...
            c.mixtureWeight(i),string(c.componentId(i)));
    end
end
end
