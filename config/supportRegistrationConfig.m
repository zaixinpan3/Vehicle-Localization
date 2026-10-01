function cfg=supportRegistrationConfig()
% supportRegistrationConfig Explicit selection of partial-support matching.
    cfg=distributionRegistrationConfig();cfg.method="supportD2D";
end
