function record=synthesizeMncavObserverGains()
% synthesizeMncavObserverGains Solve the MnCAV observer gains from the ISS LMI.
% Reads the declared synthesis inputs of mncavFullObserverConfig (ISS domain,
% noise and error scales, decay rate), solves the synthesis LMI
% (fullObserverSupport.synthesizeFullObserverGains), re-verifies the
% certificate with the solved gains and writes config/mncavSynthesizedGains.json.
% Run with YALMIP and SeDuMi on the path. Rerun after changing any input.
    root=setupVehicleLocalization();
    cfg=mncavFullObserverConfig(LoadSynthesizedGains=false);
    design=fullObserverSupport.synthesizeFullObserverGains(cfg);
    cfg.gains=design.gains;cfg.gnss.positionGain=design.gnssPositionGain;
    verified=fullObserverSupport.designFullObserverGains(cfg);
    assert(verified.fullContinuousStateCertified,'VehicleLocalization:InfeasibleFullCertificate','Synthesized gains failed verification.');
    [~,commit]=system('git -C "'+string(root)+'" rev-parse HEAD');
    record=struct('label',"MnCAV full-observer gains solved from the continuous ISS LMI", ...
        'gains',design.gains,'gainOrder',["positionGain","velocityGain","accelerationGain","headingGain"], ...
        'gnssPositionGain',design.gnssPositionGain,'lyapunovWeights',design.weights,'radiusBound',design.radiusBound, ...
        'verifiedDecayRate',verified.continuousCertificate.verification.verifiedDecayRate, ...
        'inputs',jsondecode(jsonencode(cfg.synthesis)),'route',design.route,'solverInfo',design.solverInfo, ...
        'synthesizedOn',string(datetime('now','TimeZone','America/Chicago','Format','yyyy-MM-dd''T''HH:mm:ssXXX')), ...
        'codeCommit',strtrim(string(commit)));
    path=fullfile(root,'config','mncavSynthesizedGains.json');
    f=fopen(path,'w');assert(f>=0);fprintf(f,'%s\n',jsonencode(record,PrettyPrint=true));fclose(f);
    fprintf('synthesized gains [kp kv ka kpsi] = %s, kG = %.4f, decay %.3f, radius bound %.3f\n', ...
        mat2str(design.gains,5),design.gnssPositionGain,design.decayRate,design.radiusBound);
end
