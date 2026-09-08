function design = designImprovedObserverGains(cfg)
% designImprovedObserverGains Synthesize the production timer certificate.
% The base pose gain is retained from the recorded full-pose design. YALMIP
% solves for a timer metric; independent exhaustive verification decides
% feasibility even when the solver returns a numerical-difficulty status.
    arguments
        cfg (1,1) struct = improvedObserverConfig()
    end
    root = fileparts(fileparts(mfilename('fullpath')));
    data = jsondecode(fileread(fullfile(root,'config','poseObserverCertificate.json')));
    fixture = cfg;
    fixture.K = data.K;
    fixture.N = cfg.observer.invariantGain;
    timer=designAnisotropicPoseCertificate(fixture, ...
        MinimumWeight=cfg.lidar.certificateMinimumPoseWeight);
    assert(timer.passed,'VehicleLocalization:InfeasibleCertificate', ...
        'No verified timer certificate was found for the requested configuration.');
    design = struct('kind',"aperiodic-anisotropic-pose-v2",'K',data.K,'N',fixture.N, ...
        'P',timer.P(:,:,1),'theta',cfg.observer.theta,'sigma',cfg.observer.theta, ...
        'scalingExponents',cfg.observer.scalingExponents,'timer',timer, ...
        'knownInputIncludedExactly',true,'cfg',cfg);
    design.verification = verifyImprovedObserverDesign(design,cfg);
    design.certified = design.verification.certified;
    assert(design.certified,'VehicleLocalization:InfeasibleCertificate','Final timer verification failed.');
    if strlength(cfg.synthesis.outputFolder)>0
        if ~isfolder(cfg.synthesis.outputFolder), mkdir(cfg.synthesis.outputFolder); end
        save(fullfile(cfg.synthesis.outputFolder,cfg.synthesis.saveFileName),'design');
    end
end
