function design = designImprovedObserverGains(cfg)
% designImprovedObserverGains Construct gains and a constant ISS certificate.
% GNSS uses observable chains and the triangular yaw injection. LiDAR retains
% the reference K/N and synthesizes P/Q/R for the declared delay and norm box.
% Simultaneous gain/functional optimization is not claimed to be convex.
    arguments
        cfg (1,1) struct = improvedObserverConfig()
    end
    data=buildImprovedObserverCertificateData(cfg);
    root=fileparts(fileparts(mfilename('fullpath')));
    stored=jsondecode(fileread(fullfile(root,'config','continuousObserverCertificate.json')));
    source=stored.(cfg.mode);
    design=struct('kind',"continuous-mo-hgo-v1",'mode',cfg.mode,'theta',cfg.observer.theta);
    if cfg.mode=="gnss"
        design.K=[source.K;zeros(1,2)];design.N=zeros(7,4);
        design.N(1:6,1:3)=source.N;design.N(7,4)=-cfg.observer.yawGain*design.theta^2;
        F=data.A(1:6,1:6)-source.K*data.Cg;identity=eye(6);
        operator=kron(eye(6),F.')+kron(F.',eye(6));
        design.P=reshape(operator\(-identity(:)),6,6);design.P=(design.P+design.P.')/2;
    else
        assert(exist('sdpvar','file')==2,'VehicleLocalization:MissingSolver','YALMIP must be on the path.');
        design.K=source.K;design.N=source.N;design.rate=cfg.synthesis.rate;
        delay=cfg.measurement.fixedLidarDelay;theta=cfg.observer.theta;
        A0=theta*data.A;Ad=-theta*design.K*data.C;
        uncertainty=data.modelPerturbation+norm(design.N,2)*data.outputBound4+ ...
            theta*norm(design.K,2)*norm(data.C,2)*(1-cfg.lidar.minimumPoseWeight);
        yalmip('clear');
        P=sdpvar(7,7,'symmetric');Q=sdpvar(7,7,'symmetric');R=sdpvar(7,7,'symmetric');
        g=sdpvar(1);margin=sdpvar(1);pBound=sdpvar(1);rBound=sdpvar(1);
        block=continuousObserverDelayLmi(A0,Ad,P,Q,R,g,delay,design.rate);
        constraints=[P>=1e-5*eye(7),Q>=1e-5*eye(7),R>=1e-5*eye(7), ...
            P<=pBound*eye(7),R<=rBound*eye(7),trace(P)==7, ...
            trace(Q)+trace(R)<=1000,g>=1e-5,g<=1000,margin>=1e-6, ...
            block<=-(margin+2*uncertainty*(pBound+delay*rBound))*eye(28)];
        result=optimize(constraints,-margin,sdpsettings('solver',char(cfg.synthesis.solver),'verbose',0));
        assert(result.problem==0,'VehicleLocalization:InfeasibleCertificate', ...
            'Constant-matrix LiDAR synthesis failed: %s',result.info);
        design.P=double(P);design.Q=double(Q);design.R=double(R);design.g=double(g);
        for name=["P","Q","R"],design.(name)=(design.(name)+design.(name).')/2;end
    end
    design.verification=verifyImprovedObserverDesign(design,cfg);design.certified=design.verification.certified;
    assert(design.certified,'VehicleLocalization:InfeasibleCertificate','Recovered matrices failed independent verification.');
    if strlength(cfg.synthesis.outputFolder)>0
        if ~isfolder(cfg.synthesis.outputFolder),mkdir(cfg.synthesis.outputFolder);end
        save(fullfile(cfg.synthesis.outputFolder,cfg.synthesis.saveFileName),'design');
    end
end
