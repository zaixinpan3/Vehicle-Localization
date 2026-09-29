function validateSoftPointAssociation(cfg)
% validateSoftPointAssociation Validate local ambiguity and anchor controls.
    fields={'temperature','radius','minimumPosterior','hardWithUnmergedAnchors'};
    valid=isstruct(cfg)&&isscalar(cfg)&&all(isfield(cfg,fields));
    if valid
        values=cellfun(@(f)cfg.(f),fields,UniformOutput=false);
        valid=all(cellfun(@(v)isnumeric(v)&&isscalar(v)&&isreal(v)&&isfinite(v),values));
    end
    if valid
        valid=cfg.temperature>0 && cfg.radius>0 && cfg.minimumPosterior>.5 && cfg.minimumPosterior<=1 ...
            && cfg.hardWithUnmergedAnchors>=2 && cfg.hardWithUnmergedAnchors==fix(cfg.hardWithUnmergedAnchors);
    end
    assert(valid,'VehicleLocalization:InvalidSoftPointAssociation', ...
        'Use positive temperature/radius, posterior in (0.5,1], and at least two integer anchors.');
end
