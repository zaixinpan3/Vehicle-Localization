function [design,cfg]=synthesizeMncavInvariantDesign(cfg)
% Solve and verify temporary gains from this run's fully configured inputs.
% Returns the design and a ready-to-run configuration; reads/writes no gains
% file and always re-solves, including when cfg already contains a design.
    arguments
        cfg (1,1) struct=mncavFullObserverConfig()
    end
    root=setupVehicleLocalization();external=fullfile(fileparts(root),'RobustVehicleLocalization','external');
    addpath(genpath(fullfile(external,'YALMIP')));addpath(genpath(fullfile(external,'sedumi')));
    design=localizationSupport.synthesize(cfg);
    cfg.invariantDesign=design;cfg.gainSource="fresh-lmi";
    v=design.verification;s=v.nominalFusion.worstEigenvalue;
    fprintf('Common metric, %d drift vertices, %d LMIs. Worst eigenvalues: GNSS %.9g, LiDAR %.9g, both %.9g.\n', ...
        v.vertices,v.matrixInequalities,s.gnssOnly,s.lidarOnly,s.both);
end
